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

## Benchmarking

The benchmarking dataset comes from the [TUM dataset](https://cvg.cit.tum.de/data/datasets/rgbd-dataset). 
According to the [download page](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download), I'm using an xyz dataset. The true trajectory is also provided (It's a TODO for now)

## Results

Without Optimization, the raw inputs are RGB and Depth images (which visualizes into a mumble jumble :/). The scene looks like below:

<img src="https://github.com/user-attachments/assets/31545387-b3b3-4b33-8fd0-9f1d5d6f4801" alt="Image 1" style="width:45%; display: inline-block;"/>
<img src="https://github.com/user-attachments/assets/e3de1652-795b-4277-bc34-885f74a40597" alt="Image 1" style="width:45%; display: inline-block;"/>

This mini RGBD SLAM system first performs the front end, then the backend. It's not parallelized, or partially implemented on a GPU (My laptop has an Intel i5 core with no GPU, that's why). Well, it's a proof of concept mostly for learning purposes still. The 

Below image shows the final result after:

1. Rough frame-to-frame estimates are given by the front end (`cv::solvePnPRansac` based)
2. Pose Graph bundle adjustment gets performed by `g2o`.
3. Downsample the resultant point cloud into a coarser voxel grid

**Click on the image to see a video demo**

<a href="https://youtu.be/jCsX9R2aa-I?si=R_DvmD0d8iOKy40Q">
    <img src="https://github.com/user-attachments/assets/20988112-f742-41ee-9e2e-44e87d02be19" height="300" alt=""/>
</a>

Here is the result from more datasets: (Left: 'freiburg1_rpy')

<p>
    <img src="https://github.com/user-attachments/assets/e41b2b86-2468-4b03-b6eb-b6a97b3cc113" height="300" alt=""/>
    <img src="https://github.com/user-attachments/assets/3c6bb9f1-4a48-44ed-9801-b540b51b4162" height="300" alt=""/>
</p>

## Further Reading

[I wrote a blog series about the theory and implementation of the VSLAM system that I built.](https://ricojia.github.io/2024/07/09/rgbd-slam-pnp.html)
