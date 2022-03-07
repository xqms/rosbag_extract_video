rosbag_extract_video
====================

This tool allows extraction of video from a ROS 1 bagfile. It does not re-encode the video and keeps timestamps exactly the same.

In order to encode the resulting `.nut` file, use a command like
```
ffmpeg -vsync passthrough -i input.nut -c:v libx264 -preset:v medium output.mp4
```

