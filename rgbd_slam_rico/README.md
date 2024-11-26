# RGBD SLAM Rico

This repo contains my implementation of a mini RGBD SLAM system, inspired by [RGBD SLAM v2 from Endres et al.](https://felixendres.github.io/rgbdslam_v2/)

## Usage

1. Build the docker image `docker build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -t dream-rgbd-rico .`

2. Start the docker container: `./start_rgbd_slam_docker.sh`

3. In the container, do

```bash
catkin build -j5
source devel/setup.bash
roslaunch rgbd_slam_rico rgbd_slam_rico.launch 
```

4. By default, this program saves the results in `data/rgbd_rico_slam_output.pcd`. To visualize, one can:

```bash
python3 scripts/visualize_pointcloud_open3d.py
```

- Rviz is not being used because it's less CPU-friendly and crashes often.

5. There's a script for recording rosbags.

- `roslaunch realsense2_camera rs_camera.launch align_depth:=true`. It listens to the topic there
- `python3 scripts/record_realsense_topics_in_bag.py`

## Dependencies

- [SimpleRobotics](https://github.com/RicoJia/SimpleRoboticsUtils)

## Implementation Notes

[I wrote a blog series about the theory and implementation of this VSLAM system](https://ricojia.github.io/2024/07/09/rgbd-slam-pnp.html)

## Benchmarking

The benchmarking dataset comes from the [TUM dataset](https://cvg.cit.tum.de/data/datasets/rgbd-dataset).
According to the [download page](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download). The true trajectory is also provided (It's a TODO for now)

## Additional Tools

**In a Python 3.10 environment** (which unfortunately is not the docker container provided, due to ROS1 Noetic constraints), there is a tool that can mask out humans. To use it:

```bash
pip3 install ricomodels
python3 scripts/remove_objects_processing.py
```

The result is not perfect, one can still see diluted human figures. This is using PyTorch's DeepLabV3+ model. Dataset: Sequence 'freiburg3_sitting_static'

<div style="text-align: center;">
    <p align="center">
        <img src="https://github.com/user-attachments/assets/2010269c-471b-44ad-a0bd-16944d26e54a" height="200" alt="" style="display: inline-block;"/>
        <img src="https://github.com/user-attachments/assets/7c7c0dde-260a-46ee-ae15-60b17682f38c" height="200" alt="" style="display: inline-block;"/>
    </p>
</div>

Overall:

<div style="text-align: center;">
    <p align="center">
       <figure>
            <img src="https://github.com/user-attachments/assets/d81aec5b-91d7-45b1-9480-d4e71f169864" height="300" alt=""/>
       </figure>
    </p>
</div>

## Results

Without Optimization, the raw inputs are RGB and Depth images (which visualizes into a mumble jumble :/). The scene looks like below:

<div style="text-align: center;">
<p align="center">
            <img src="https://github.com/user-attachments/assets/31545387-b3b3-4b33-8fd0-9f1d5d6f4801" height="200" alt=""  style="display: inline-block;"/>
            <img src="https://github.com/user-attachments/assets/e3de1652-795b-4277-bc34-885f74a40597" height="200" alt=""  style="display: inline-block;"/>
</p>
</div>

This mini RGBD SLAM system first performs the front end, then the backend. It's not parallelized, or partially implemented on a GPU (My laptop has an Intel i5 core with no GPU, that's why). Well, it's a proof of concept mostly for learning purposes still. The

Below image shows the final result after:

1. Rough frame-to-frame estimates are given by the front end (`cv::solvePnPRansac` based)
2. Pose Graph bundle adjustment gets performed by `g2o`.
3. Downsample the resultant point cloud into a coarser voxel grid

<a href="https://youtu.be/jCsX9R2aa-I?si=R_DvmD0d8iOKy40Q">
    <img src="https://github.com/user-attachments/assets/20988112-f742-41ee-9e2e-44e87d02be19" height="300" alt=""/>
</a>

Here is the result from more datasets: (Left: 'freiburg1_rpy')

<p>
    <img src="https://github.com/user-attachments/assets/e41b2b86-2468-4b03-b6eb-b6a97b3cc113" height="300" alt=""/>
    <img src="https://github.com/user-attachments/assets/3c6bb9f1-4a48-44ed-9801-b540b51b4162" height="300" alt=""/>
</p>
