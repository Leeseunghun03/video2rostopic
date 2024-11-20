# video2rostopic

## Overview
This ROS 2 package converts a video file into a stream of `sensor_msgs/Image` messages and publishes them on a specified topic

---

## Features
- Converts video files (e.g., `.mp4`, `.avi`) to ROS 2 image topics.
- Configurable topic names and publishing rates.
- Supports a variety of video formats via OpenCV.

---

## Requirements
- ROS 2 (tested with Humble)
- OpenCV

## Usage
```
ros2 launch image_pub image_pub.launch.py
```

