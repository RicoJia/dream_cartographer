# Visual Slam Implementation Notes

## Setup

### Data

For our mini visual slam course, we will be using part of the [TUM dataset](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats) From rosbags

- `/camera/rgb/camera_info (sensor_msgs/CameraInfo)` contains the intrinsic camera parameters for the RGB camera, as reported by the OpenNI driver
- `/camera/rgb/image_color (sensor_msgs/Image)` contains the color image from the RGB camera
- `/tf (tf/tfMessage)`  ground-truth data

### Infrastructure

To maximally reuse the existing infrastructure, I created a infrastructure in [Simple Robotics Cpp Utils](https://github.com/RicoJia/SimpleRoboticsUtils/tree/master/simple_robotics_cpp_utils). Such as:
    - BagParser


## Camera Model Recap

### RGB Camera Model

### Depth Camera Model


