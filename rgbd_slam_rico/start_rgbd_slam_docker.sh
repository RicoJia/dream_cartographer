#!/bin/bash
# if [[ "$EUID" -ne 0 ]]; then echo "Please run this script as sudo" ; exit 1; fi
# this is hardcoded mounted volume, 
xhost +local:root
if [[ -f "data/rgbd_dataset_freiburg1_xyz.bag" ]]; then 
    echo "No need to download dataset rgbd_dataset_freiburg1_xyz.bag..."
else
    echo "Downloading dataset rgbd_dataset_freiburg1_xyz.bag..."
    wget -P data https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag
fi
SIMPLE_ROBOTICS_UTILS_DIR="/home/$USER/file_exchange_port/The-Dream-Robot/src/SimpleRoboticsUtils"
sudo docker run --name my_ros_container --rm -e DISPLAY=$DISPLAY \
    -v /etc/passwd:/etc/passwd:ro \
    -v /etc/group:/etc/group:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /home/${USER}/file_exchange_port/dream_cartographer_ws:/home/${USER}/dream_cartographer_ws \
    -v ~/.ssh:/root/.ssh \
    -v ${SIMPLE_ROBOTICS_UTILS_DIR}/simple_robotics_ros_utils:/home/${USER}/dream_cartographer_ws/src/simple_robotics_ros_utils \
    -v ${SIMPLE_ROBOTICS_UTILS_DIR}/simple_robotics_cpp_utils:/home/${USER}/dream_cartographer_ws/src/simple_robotics_cpp_utils \
    --user $(id -u):$(id -g) -e XAUTHORITY=/root/.Xauthority \
    --dns 8.8.8.8 --dns 8.8.4.4 -it --network="host" --privileged dream-rgbd-rico
    