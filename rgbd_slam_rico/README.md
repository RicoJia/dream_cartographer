# RGBD SLAM Rico

This repo contains 1. my implementation of his RGBD SLAM 2. RGBD SLAM V2 (Endres et al.). 


## Usage
1. Build the docker image `docker build --build-arg USER_ID=$(id -u) --build-arg GROUP_ID=$(id -g) -t dream-rgbd-rico .`

2. Start the docker container: `./start_rgbd_slam_docker.sh`

## Benchmarking

The benchmarking dataset comes from the [TUM dataset](https://cvg.cit.tum.de/data/datasets/rgbd-dataset). 
According to the [download page](https://cvg.cit.tum.de/data/datasets/rgbd-dataset/download), I'm using an xyz dataset. The true trajectory