rosbag_extract_video
====================

This tool allows extraction of video from a ROS 1 bagfile. It does not re-encode the video and keeps timestamps exactly the same.

```
rosrun rosbag_extract_video --topic /my/topic --output video.nut input.bag
```

In order to transcode the resulting `.nut` file, use a command like
```
ffmpeg -vsync passthrough -i video.nut -c:v libx264 -preset:v medium video.mp4
```
to keep the timestamps intact.
