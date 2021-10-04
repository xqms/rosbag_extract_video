// Extract video from rosbags
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/query.h>

#include <boost/program_options.hpp>

#include <iostream>

#include <rosfmt/full.h>

extern "C"
{
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include <turbojpeg.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>

template <class F>
class final_act
{
public:
    explicit final_act(F f) noexcept
      : f_(std::move(f)), invoke_(true) {}

    final_act(final_act&& other) noexcept
     : f_(std::move(other.f_)),
       invoke_(other.invoke_)
    {
        other.invoke_ = false;
    }

    final_act(const final_act&) = delete;
    final_act& operator=(const final_act&) = delete;

    ~final_act() noexcept
    {
        if (invoke_) f_();
    }

private:
    F f_;
    bool invoke_;
};

struct ImgBuffer
{
    sensor_msgs::ImageConstPtr ptr;

    static void free(void* opaque, uint8_t*)
    {
        delete reinterpret_cast<ImgBuffer*>(opaque);
    }
};

AVFrame* frameFromMsg(const sensor_msgs::ImageConstPtr& img)
{
    AVFrame* ret = av_frame_alloc();

    ret->width = img->width;
    ret->height = img->height;

    if(img->encoding == sensor_msgs::image_encodings::RGB8)
    {
        ret->format = AV_PIX_FMT_RGB24;

        ImgBuffer* imgBuffer = new ImgBuffer{img};
        ret->buf[0] = av_buffer_create(const_cast<uint8_t*>(img->data.data()), img->data.size(), &ImgBuffer::free, imgBuffer, AV_BUFFER_FLAG_READONLY);
        ret->linesize[0] = img->step;
        ret->data[0] = ret->buf[0]->data;
        ret->extended_data[0] = ret->data[0];
    }
    else
    {
        fmt::print(stderr, "Unsupported encoding {}\n", img->encoding);
        std::exit(1);
    }

    return ret;
}

std::string averror(int code)
{
    static thread_local char buf[1024];

    av_strerror(code, buf, sizeof(buf));

    return buf;
}

int main(int argc, char** argv)
{
    namespace po = boost::program_options;

    po::options_description desc{"Options"};
    desc.add_options()
        ("help", "This help message")
        ("topic", po::value<std::string>()->required(), "Topic to extract")
        ("output", po::value<std::string>()->value_name("out.mp4")->required(), "Output file")
        ("codec", po::value<std::string>()->value_name("H264|H265"), "Codec to use")
    ;

    po::options_description hidden("Hidden options");
    hidden.add_options()
        ("bag", po::value<std::string>()->required(), "input file")
    ;

    po::options_description cmdline_options;
    cmdline_options.add(desc).add(hidden);

    po::positional_options_description positional;
    positional.add("bag", 1);

    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
        options(cmdline_options).positional(positional).run(), vm);

    if(vm.count("help"))
    {
        std::cout << "Usage: rosbag_extract_video <options> <bag file>\n";
        std::cout << desc << "\n";
        return 1;
    }

    po::notify(vm);

    /* allocate the output media context */
    std::string outputFilename = vm["output"].as<std::string>();
    AVFormatContext* oc{};
    avformat_alloc_output_context2(&oc, NULL, NULL, outputFilename.c_str());
    if(!oc)
    {
        fmt::print(stderr, "Could not deduce output format from file extension. Using MP4.\n");
        avformat_alloc_output_context2(&oc, NULL, "mp4", outputFilename.c_str());
    }
    if(!oc)
        return 1;

    if(int err = av_opt_set(oc, "movflags", "+faststart", AV_OPT_SEARCH_CHILDREN))
    {
        fmt::print(stderr, "Could not set movflags: {}\n", err);
        return 1;
    }

    final_act _deallocContext([&] { avformat_free_context(oc); });

    // Figure out video codec
    AVCodecID codecID = oc->oformat->video_codec;
    if(codecID == AV_CODEC_ID_NONE)
    {
        fmt::print(stderr, "Output format does not support video\n");
        return 1;
    }

    if(vm.count("codec"))
    {
        std::string codecName = vm["codec"].as<std::string>();
        if(codecName == "H264")
            codecID = AV_CODEC_ID_H264;
        else if(codecName == "H265")
            codecID = AV_CODEC_ID_HEVC;
        else
        {
            fmt::print(stderr, "Unknown codec {}\n", codecName);
            return 1;
        }
    }

    AVCodec* codec{};
    AVCodecContext *codecContext{};
    codec = avcodec_find_encoder(codecID);
    if(!codec)
    {
        fmt::print(stderr, "Could not find encoder for '{}'\n",
            avcodec_get_name(codecID));
        return 1;
    }

    AVStream* stream{};
    stream = avformat_new_stream(oc, NULL);
    if(!stream)
    {
        fmt::print(stderr, "Could not allocate stream\n");
        return 1;
    }

    stream->id = 0;


    std::string bagFilename = vm["bag"].as<std::string>();
    fmt::print("Opening bag file {}...\n", bagFilename);

    rosbag::Bag bag{bagFilename};

    rosbag::View view{bag, rosbag::TopicQuery{vm["topic"].as<std::string>()}};

    tjhandle decoder = tjInitDecompress();

    AVFrame* frame = av_frame_alloc();

    ros::Time t0;

    SwsContext* swsContext{};

    auto writePackets = [&](){
        int ret = 0;
        while(ret >= 0)
        {
            AVPacket pkt = { 0 };

            ret = avcodec_receive_packet(codecContext, &pkt);
            if(ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
                break;
            else if (ret < 0)
            {
                fprintf(stderr, "Error encoding a frame\n");
                exit(1);
            }

            /* rescale output packet timestamp values from codec to stream timebase */
            av_packet_rescale_ts(&pkt, codecContext->time_base, stream->time_base);
            pkt.stream_index = stream->index;

            /* Write the compressed frame to the media file. */
            ret = av_interleaved_write_frame(oc, &pkt);
            av_packet_unref(&pkt);
            if(ret < 0)
            {
                fprintf(stderr, "Error while writing output packet\n");
                exit(1);
            }
        }
    };

    auto writeFrame = [&](AVFrame* frame){
        // Open stream if required
        if(!codecContext)
        {
            fmt::print("Init encoder\n");
            codecContext = avcodec_alloc_context3(codec);
            if(!codecContext)
            {
                fmt::print(stderr, "Could not alloc an encoding context\n");
                std::exit(1);
            }

            codecContext->codec_id = codecID;

            /* Resolution must be a multiple of two. */
            codecContext->width = frame->width;
            codecContext->height = frame->height;
            stream->time_base = (AVRational){ 1, 1000000ULL };
            codecContext->time_base = stream->time_base;

            codecContext->pix_fmt = (AVPixelFormat)frame->format;

            /* Some formats want stream headers to be separate. */
            if(oc->oformat->flags & AVFMT_GLOBALHEADER)
                codecContext->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;

            AVDictionary* dict{};
            av_dict_set(&dict, "preset", "medium", 0);
            av_dict_set(&dict, "crf", "23", 0);

            if(int ret = avcodec_open2(codecContext, codec, &dict))
            {
                fmt::print(stderr, "Could not open video codec: {}\n", averror(ret));
                std::exit(1);
            }

            char* buf{};
            av_opt_serialize(codecContext->priv_data, 0, AV_OPT_SERIALIZE_SKIP_DEFAULTS, &buf, '=', ';');
            fmt::print("Codec options: {}\n", buf);

            /* copy the stream parameters to the muxer */
            if(avcodec_parameters_from_context(stream->codecpar, codecContext) < 0)
            {
                fprintf(stderr, "Could not copy the stream parameters\n");
                exit(1);
            }

            fmt::print("Output mux:\n");
            av_dump_format(oc, 0, outputFilename.c_str(), 1);
            fmt::print("Mux end\n");

            /* open the output file, if needed */
            if(!(oc->oformat->flags & AVFMT_NOFILE))
            {
                if(int ret = avio_open(&oc->pb, outputFilename.c_str(), AVIO_FLAG_WRITE))
                {
                    fmt::print(stderr, "Could not open '{}': {}\n", outputFilename.c_str(), averror(ret));
                    std::exit(1);
                }
            }

            /* Write the stream header, if any. */
            if(int ret = avformat_write_header(oc, nullptr))
            {
                fmt::print(stderr, "Error occurred when opening output file: {}\n", averror(ret));
                std::exit(1);
            }
        }

        if(avcodec_send_frame(codecContext, frame) < 0)
        {
            fmt::print(stderr, "Could not send frame to output codec\n");
            std::exit(1);
        }

        writePackets();
    };

    auto numImages = view.size();
    int counter = 0;

    for(auto& msg : view)
    {
        fmt::print("\rmsg {}/{} ({}%) \033[K", counter, numImages, counter * 100 / numImages);
        fflush(stdout);
        counter++;

        if(auto imgMsg = msg.instantiate<sensor_msgs::CompressedImage>())
        {
            int width, height, jpegSubsamp, jpegColorspace;
            if(tjDecompressHeader3(decoder, imgMsg->data.data(), imgMsg->data.size(), &width, &height, &jpegSubsamp, &jpegColorspace) != 0)
            {
                fmt::print(stderr, "Could not decode JPEG header\n");
                continue;
            }

            if(frame->width != width || frame->height != height)
            {
                switch(jpegSubsamp)
                {
                    case TJSAMP_420:
                        frame->format = AV_PIX_FMT_YUV420P;
                        break;
                    case TJSAMP_444:
                        frame->format = AV_PIX_FMT_YUV444P;
                        break;
                    case TJSAMP_422:
                        frame->format = AV_PIX_FMT_YUV422P;
                        break;
                    default:
                        fmt::print(stderr, "Unknown JPEG chrominance subsampling {}\n", jpegSubsamp);
                        return 1;
                }

                frame->width = width;
                frame->height = height;

                if(av_frame_get_buffer(frame, 0) < 0)
                {
                    fmt::print(stderr, "Could not allocate the video frame data\n");
                    return 1;
                }
            }

            if(av_frame_make_writable(frame) < 0)
            {
                fmt::print(stderr, "Could not make frame writable\n");
                return 1;
            }

            if(tjDecompressToYUVPlanes(decoder, imgMsg->data.data(), imgMsg->data.size(), frame->data, width, frame->linesize, height, 0) != 0)
            {
                fmt::print(stderr, "Could not decompress JPEG\n");
                continue;
            }

            if(t0 == ros::Time(0))
                t0 = imgMsg->header.stamp;

            ros::Duration rosPTS = imgMsg->header.stamp - t0;

            // PTS is in µsec
            frame->pts = rosPTS.toNSec() / 1000ULL;

            writeFrame(frame);
        }
        else if(auto imgMsg = msg.instantiate<sensor_msgs::Image>())
        {
            AVFrame* imgFrame = frameFromMsg(imgMsg);

            if(t0 == ros::Time(0))
                t0 = imgMsg->header.stamp;

            ros::Duration rosPTS = imgMsg->header.stamp - t0;

            // PTS is in µsec
            imgFrame->pts = rosPTS.toNSec() / 1000ULL;

            if(imgFrame->format == AV_PIX_FMT_YUV420P)
            {
                writeFrame(imgFrame);
            }
            else
            {
                // Convert to YUV420P
                if(frame->width != imgFrame->width || frame->height != imgFrame->height)
                {
                    frame->width = imgFrame->width;
                    frame->height = imgFrame->height;
                    frame->format = AV_PIX_FMT_YUV420P;

                    av_frame_get_buffer(frame, 0);

                    if (!swsContext)
                    {
                        swsContext = sws_getContext(
                            imgFrame->width, imgFrame->height,
                            (AVPixelFormat)imgFrame->format,
                            frame->width, frame->height,
                            AV_PIX_FMT_YUV420P,
                            SWS_BICUBIC, NULL, NULL, NULL);
                    }

                    if(!swsContext)
                    {
                        fprintf(stderr,
                                "Could not initialize the conversion context\n");
                        exit(1);
                    }
                }

                av_frame_make_writable(frame);
                sws_scale(swsContext,
                    (const uint8_t * const *) imgFrame->data,
                    imgFrame->linesize, 0, imgFrame->height, frame->data,
                    frame->linesize
                );
                frame->pts = imgFrame->pts;

                writeFrame(frame);
            }

            av_frame_unref(imgFrame);
        }
    }
    fmt::print("\n");

    // Flush encoder
    avcodec_send_frame(codecContext, nullptr);
    writePackets();

    av_write_trailer(oc);

    if(!(oc->oformat->flags & AVFMT_NOFILE))
        avio_closep(&oc->pb);

    fmt::print("done!\n");
    return 0;
}
