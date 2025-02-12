// Extract video from rosbags
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <rosbag/bag.h>
#include <rosbag/query.h>
#include <rosbag/view.h>

#include <boost/program_options.hpp>

#include <iostream>

#include <fmt/chrono.h>
#include <rosfmt/full.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <turbojpeg.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

template <class F> class final_act {
public:
  explicit final_act(F f) noexcept : f_(std::move(f)), invoke_(true) {}

  final_act(final_act &&other) noexcept
      : f_(std::move(other.f_)), invoke_(other.invoke_) {
    other.invoke_ = false;
  }

  final_act(const final_act &) = delete;
  final_act &operator=(const final_act &) = delete;

  ~final_act() noexcept {
    if (invoke_)
      f_();
  }

private:
  F f_;
  bool invoke_;
};

struct ImgBuffer {
  sensor_msgs::ImageConstPtr ptr;

  static void free(void *opaque, uint8_t *) {
    delete reinterpret_cast<ImgBuffer *>(opaque);
  }
};

AVFrame *frameFromMsg(const sensor_msgs::ImageConstPtr &img) {
  AVFrame *ret = av_frame_alloc();

  ret->width = img->width;
  ret->height = img->height;

  if (img->encoding == sensor_msgs::image_encodings::RGB8) {
    ret->format = AV_PIX_FMT_RGB24;

    ImgBuffer *imgBuffer = new ImgBuffer{img};
    ret->buf[0] = av_buffer_create(const_cast<uint8_t *>(img->data.data()),
                                   img->data.size(), &ImgBuffer::free,
                                   imgBuffer, AV_BUFFER_FLAG_READONLY);
    ret->linesize[0] = img->step;
    ret->data[0] = ret->buf[0]->data;
    ret->extended_data[0] = ret->data[0];
  } else {
    fmt::print(stderr, "Unsupported encoding {}\n", img->encoding);
    std::exit(1);
  }

  return ret;
}

std::string averror(int code) {
  static thread_local char buf[1024];

  av_strerror(code, buf, sizeof(buf));

  return buf;
}

const std::map<std::string, AVCodecID> CODECS = {{"jpeg", AV_CODEC_ID_MJPEG},
                                                 {"jpg", AV_CODEC_ID_MJPEG},
                                                 {"h264", AV_CODEC_ID_H264},
                                                 {"h265", AV_CODEC_ID_HEVC}};

const std::map<std::string, AVPixelFormat> RAW_PIXEL_FORMATS = {
    {sensor_msgs::image_encodings::RGB8, AV_PIX_FMT_RGB24},
    {sensor_msgs::image_encodings::BGR8, AV_PIX_FMT_BGR24},
    {sensor_msgs::image_encodings::MONO8, AV_PIX_FMT_GRAY8}};

int main(int argc, char **argv) {
  namespace po = boost::program_options;

  po::options_description desc{"Options"};

  // clang-format off
  desc.add_options()("help", "This help message")
      ("topic", po::value<std::string>()->required(), "Topic to extract")
      ("output", po::value<std::string>()->value_name("out.mp4")->required(), "Output file")
  ;
  // clang-format on

  po::options_description hidden("Hidden options");
  hidden.add_options()("bag", po::value<std::string>()->required(),
                       "input file");

  po::options_description cmdline_options;
  cmdline_options.add(desc).add(hidden);

  po::positional_options_description positional;
  positional.add("bag", 1);

  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv)
                .options(cmdline_options)
                .positional(positional)
                .run(),
            vm);

  if (vm.count("help")) {
    std::cout << "Usage: rosbag_extract_video <options> <bag file>\n";
    std::cout << desc << "\n";
    return 1;
  }

  po::notify(vm);

  /* allocate the output media context */
  std::string outputFilename = vm["output"].as<std::string>();
  AVFormatContext *oc{};
  const char *format = nullptr;
  if (outputFilename == "/dev/stdout")
    format = "nut";

  avformat_alloc_output_context2(&oc, NULL, format, outputFilename.c_str());
  if (!oc) {
    fmt::print(
        stderr,
        "Could not deduce output format from file extension. Using MP4.\n");
    avformat_alloc_output_context2(&oc, NULL, "mp4", outputFilename.c_str());
  }
  if (!oc)
    return 1;

  fmt::print(stderr, "Using output format '{}'\n", oc->oformat->name);
  if (oc->oformat->name == std::string{"mp4"}) {
    if (int err =
            av_opt_set(oc, "movflags", "+faststart", AV_OPT_SEARCH_CHILDREN)) {
      fmt::print(stderr, "Could not set movflags: {}\n", err);
      return 1;
    }
  }

  final_act _deallocContext([&] { avformat_free_context(oc); });

  AVStream *stream{};
  stream = avformat_new_stream(oc, NULL);
  if (!stream) {
    fmt::print(stderr, "Could not allocate stream\n");
    return 1;
  }

  stream->id = 0;
  stream->time_base = (AVRational){1, 1000000000ULL};

  stream->codecpar = avcodec_parameters_alloc();
  stream->codecpar->codec_type = AVMEDIA_TYPE_VIDEO;

  std::string bagFilename = vm["bag"].as<std::string>();
  fmt::print(stderr, "Opening bag file {}...\n", bagFilename);

  rosbag::Bag bag{bagFilename};

  rosbag::View view{bag, rosbag::TopicQuery{vm["topic"].as<std::string>()}};

  AVPacket *packet = av_packet_alloc();
  final_act _deallocPacket([&] { av_packet_free(&packet); });

  ros::Time t0;

  auto determinePTS = [&](const ros::Time &bagTime, const std_msgs::Header& header) {
    ros::Time stamp = (header.stamp != ros::Time(0)) ? header.stamp : bagTime;

    if (t0 == ros::Time(0))
      t0 = stamp;

    return stamp - t0;
  };

  // Pass 1: Determine stream parameters
  {
    AVCodecContext *codecCtx{};
    final_act _deallocParser([&] { avcodec_free_context(&codecCtx); });

    for (auto &msg : view) {
      if (auto imgMsg = msg.instantiate<sensor_msgs::CompressedImage>()) {

        if (!codecCtx) {
          auto codecIt = CODECS.find(imgMsg->format);
          if (codecIt == CODECS.end()) {
            fmt::print(stderr, "Error: Unknown compressed encoding '{}'\n",
                       imgMsg->format);
            return 1;
          }

          AVCodec *codec = avcodec_find_decoder(codecIt->second);
          if (!codec) {
            fmt::print(stderr,
                       "Error: Could not open ffmpeg codec for decoding '{}'\n",
                       imgMsg->format);
            return 1;
          }

          codecCtx = avcodec_alloc_context3(codec);

          int ret = avcodec_open2(codecCtx, codec, nullptr);
          if (ret != 0) {
            fmt::print(stderr,
                       "Could not open ffmpeg codec for decoding '{}'\n",
                       imgMsg->format);
            return 1;
          }
        }

        // Create AVBufferRef for this packet
        auto ptr = new sensor_msgs::CompressedImagePtr{imgMsg};
        AVBufferRef *buffer = av_buffer_create(
            imgMsg->data.data(), imgMsg->data.size(),
            [](void *ptr, uint8_t *) -> void {
              delete reinterpret_cast<sensor_msgs::CompressedImagePtr *>(ptr);
            },
            ptr, AV_BUFFER_FLAG_READONLY);

        ros::Duration rosPTS = determinePTS(msg.getTime(), imgMsg->header);

        packet->buf = buffer;
        packet->data = imgMsg->data.data();
        packet->size = imgMsg->data.size();
        packet->stream_index = stream->index;
        packet->pts = rosPTS.toNSec();
        packet->dts = rosPTS.toNSec();

        if (avcodec_send_packet(codecCtx, packet) != 0) {
          fmt::print(stderr, "Could not send packet to decoder");
          return 1;
        }

        if (codecCtx->width > 0 && codecCtx->height > 0) {
          avcodec_parameters_from_context(stream->codecpar, codecCtx);
          break;
        }
      } else if (auto imgMsg = msg.instantiate<sensor_msgs::Image>()) {
        stream->codecpar->codec_id = AV_CODEC_ID_RAWVIDEO;
        stream->codecpar->codec_tag = 0;
        stream->codecpar->width = imgMsg->width;
        stream->codecpar->height = imgMsg->height;

        auto pixIt = RAW_PIXEL_FORMATS.find(imgMsg->encoding);
        if (pixIt == RAW_PIXEL_FORMATS.end()) {
          fmt::print(stderr, "Unknown image encoding '{}'\n", imgMsg->encoding);
          return 1;
        }

        stream->codecpar->format = pixIt->second;
        stream->codecpar->codec_tag =
            avcodec_pix_fmt_to_codec_tag(pixIt->second);

        break;
      }
    }

    if (stream->codecpar->width <= 0) {
      fmt::print(stderr, "Could not determine codec parameters from "
                         "messages. Maybe wrong topic?\n");
      return 1;
    }
  }

  // Open output stream
  if (!(oc->oformat->flags & AVFMT_NOFILE)) {
    if (int ret = avio_open(&oc->pb, outputFilename.c_str(), AVIO_FLAG_WRITE)) {
      fmt::print(stderr, "Could not open '{}': {}\n", outputFilename.c_str(),
                 averror(ret));
      std::exit(1);
    }
  }

  // Write the stream header, if any.
  int ret = avformat_write_header(oc, nullptr);
  if (ret < 0) {
    fmt::print(stderr, "Error occurred when writing output header: {}\n",
               averror(ret));
    std::exit(1);
  }

  av_dump_format(oc, 0, outputFilename.c_str(), 1);

  auto numImages = view.size();
  int counter = 0;

  ros::Duration lastTimestamp;

  for (auto &msg : view) {
    fmt::print(stderr, "\rmsg {}/{} ({}%) \033[K", counter, numImages,
               counter * 100 / numImages);
    counter++;

    if (auto imgMsg = msg.instantiate<sensor_msgs::CompressedImage>()) {
      // Create AVBufferRef for this packet
      auto ptr = new sensor_msgs::CompressedImagePtr{imgMsg};
      AVBufferRef *buffer = av_buffer_create(
          imgMsg->data.data(), imgMsg->data.size(),
          [](void *ptr, uint8_t *) -> void {
            delete reinterpret_cast<sensor_msgs::CompressedImagePtr *>(ptr);
          },
          ptr, AV_BUFFER_FLAG_READONLY);

      ros::Duration rosPTS = determinePTS(msg.getTime(), imgMsg->header);

      packet->buf = buffer;
      packet->data = imgMsg->data.data();
      packet->size = imgMsg->data.size();
      packet->stream_index = stream->index;
      packet->pts = rosPTS.toNSec();
      packet->dts = rosPTS.toNSec();
      lastTimestamp = rosPTS;

      int ret = av_interleaved_write_frame(oc, packet);
      av_packet_unref(packet);
      if (ret < 0) {
        fmt::print(stderr, "Error while writing output packet\n");
        return 1;
      }
    } else if (auto imgMsg = msg.instantiate<sensor_msgs::Image>()) {
      // Create AVBufferRef for this packet
      auto ptr = new sensor_msgs::ImagePtr{imgMsg};
      AVBufferRef *buffer = av_buffer_create(
          imgMsg->data.data(), imgMsg->data.size(),
          [](void *ptr, uint8_t *) -> void {
            delete reinterpret_cast<sensor_msgs::ImagePtr *>(ptr);
          },
          ptr, AV_BUFFER_FLAG_READONLY);

      ros::Duration rosPTS = determinePTS(msg.getTime(), imgMsg->header);

      packet->buf = buffer;
      packet->data = imgMsg->data.data();
      packet->size = imgMsg->data.size();
      packet->stream_index = stream->index;
      packet->pts = rosPTS.toNSec();
      packet->dts = rosPTS.toNSec();
      lastTimestamp = rosPTS;

      int ret = av_interleaved_write_frame(oc, packet);
      av_packet_unref(packet);
      if (ret < 0) {
        fmt::print(stderr, "Error while writing output packet\n");
        return 1;
      }
    }
  }
  fmt::print(stderr, "\n");

  // Flush queue
  av_interleaved_write_frame(oc, nullptr);

  av_write_trailer(oc);

  if (!(oc->oformat->flags & AVFMT_NOFILE))
    avio_closep(&oc->pb);

  fmt::print(stderr, "done!\n");
  fmt::print(stderr, "Extracted {}s of video.\n", lastTimestamp.toSec());

  // A lot of std::chrono magic to get local/UTC time
  ros::Time endTime = t0 + lastTimestamp;
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      startTimeC(std::chrono::nanoseconds(t0.toNSec()));
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds>
      endTimeC(std::chrono::nanoseconds(endTime.toNSec()));

  std::chrono::seconds startTimeS =
      std::chrono::duration_cast<std::chrono::seconds>(
          startTimeC.time_since_epoch());
  std::time_t startTimeSC(startTimeS.count());
  std::tm startTimeB;
  std::tm startTimeBUTC;
  localtime_r(&startTimeSC, &startTimeB);
  gmtime_r(&startTimeSC, &startTimeBUTC);

  std::chrono::seconds endTimeS =
      std::chrono::duration_cast<std::chrono::seconds>(
          endTimeC.time_since_epoch());
  std::time_t endTimeSC(endTimeS.count());
  std::tm endTimeB;
  std::tm endTimeBUTC;
  localtime_r(&endTimeSC, &endTimeB);
  gmtime_r(&endTimeSC, &endTimeBUTC);

  fmt::print(stderr,
             "Start time:     {:%Y-%m-%d %H:%M:%S} ({}) / {:%Y-%m-%d %H:%M:%S} "
             "(UTC)\n",
             startTimeB, daylight ? tzname[1] : tzname[0], startTimeBUTC);
  fmt::print(stderr,
             "End time:       {:%Y-%m-%d %H:%M:%S} ({}) / {:%Y-%m-%d %H:%M:%S} "
             "(UTC)\n",
             endTimeB, daylight ? tzname[1] : tzname[0], endTimeBUTC);

  return 0;
}
