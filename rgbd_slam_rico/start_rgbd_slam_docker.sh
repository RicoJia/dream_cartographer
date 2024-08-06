#!/bin/bash
# if [[ "$EUID" -ne 0 ]]; then echo "Please run this script as sudo" ; exit 1; fi
# this is hardcoded mounted volume, 
xhost +local:root

declare -A bags_link_lookup #?
bags_link_lookup["data/rgbd_dataset_freiburg1_xyz.bag"]="https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_xyz.bag"
bags_link_lookup["data/rgbd_dataset_freiburg1_rpy.bag"]="https://cvg.cit.tum.de/rgbd/dataset/freiburg1/rgbd_dataset_freiburg1_rpy.bag"

echo "Download bag files if they are missing ..."
for key in "${!bags_link_lookup[@]}"; do
    if [[ -f "${key}" ]]; then 
        echo "No need to download dataset ${key}..."
    else
        echo "Downloading dataset ${key}"
        wget -P data "${bags_link_lookup[$key]}"
    fi
done
echo "Done Downloading"

# IMAGE_NAME=junior-project-rico
IMAGE_NAME=dream-rgbd-rico

SIMPLE_ROBOTICS_UTILS_DIR="/home/$USER/file_exchange_port/The-Dream-Robot/src/SimpleRoboticsUtils"
sudo docker run --name ${IMAGE_NAME}-container --rm -e DISPLAY=$DISPLAY \
    -v /etc/passwd:/etc/passwd:ro \
    -v /etc/group:/etc/group:ro \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v $HOME/.Xauthority:/root/.Xauthority \
    -v /home/${USER}/file_exchange_port/dream_cartographer_ws:/home/${USER}/dream_cartographer_ws \
    -v ~/.ssh:/root/.ssh \
    -v ${SIMPLE_ROBOTICS_UTILS_DIR}:/home/${USER}/dream_cartographer_ws/src/SimpleRoboticsUtils \
    -v ${SIMPLE_ROBOTICS_UTILS_DIR}/simple_robotics_common_udev_rules:/etc/udev/rules.d \
    -v /dev/bus/usb:/dev/bus/usb \
    -v /home/${USER}/.ros:/home/rico/.ros/ \
    --user $(id -u):$(id -g) -e XAUTHORITY=/root/.Xauthority \
    --dns 8.8.8.8 --dns 8.8.4.4 -it --network="host" --privileged ${IMAGE_NAME}
