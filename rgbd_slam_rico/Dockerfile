# DOCKER_BUILDKIT=1 docker build --build-arg USER_ID=$(id -u) --build-arg USER_NAME=$(whoami) --build-arg GROUP_ID=$(id -g) -t dream-rgbd-rico .
# sudo docker run --name my_ros_container --rm -e DISPLAY=$DISPLAY -v /etc/passwd:/etc/passwd:ro -v /etc/group:/etc/group:ro --user $(id -u):$(id -g) -v /tmp/.X11-unix:/tmp/.X11-unix:ro -v $HOME/.Xauthority:/root/.Xauthority -e XAUTHORITY=/root/.Xauthority -v /home/${USER_NAME}/file_exchange_port/dream_cartographer_ws:/home/${USER_NAME}/dream_cartographer_ws -v ~/.ssh:/root/.ssh --dns 8.8.8.8 --dns 8.8.4.4 -it --network="host" --privileged dream-rgbd-rico

# This base image may/may not play well with pyrealsense through pip install

FROM ros:noetic-perception
# install rtabmap packages
# ARG CACHE_DATE=2016-01-01
ENV DEBIAN_FRONTEND=noninteractive

# Note: technically, we want to reduce the amount of layers. But for faster builds, we have multiple layers
RUN apt-get update && apt-get install -y \
    vim \
    ccache \
    # camera service
    python3-pip \
    python3-catkin-tools \
    v4l-utils \
    wget \
    unzip \
    git \
    # useful for git complete
    bash-completion \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get update && apt-get install -y \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-rviz \
    ros-noetic-xacro \
    ros-noetic-tf2 \
    ros-noetic-tf \
    ros-noetic-tf2-eigen \
    ros-noetic-tf2-ros \ 
    ros-noetic-rtabmap \
    ros-noetic-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/

# Ceres and Google dependencies
RUN apt-get update && apt-get install -y \
    liblapack-dev \
    libsuitesparse-dev \
    libcxsparse3 \
    libgflags-dev \
    libgoogle-glog-dev \
    libceres-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    gdb \
    libsuitesparse-dev

RUN pip3 install open3d

# usbutils is for lsusb
RUN apt-get update \
 && apt-get install -y \
    ros-noetic-librealsense2* \
    ros-noetic-realsense2-* \
    usbutils \
 && rm -rf /var/lib/apt/lists/*

# Create a user to match the host's UID/GID and create necessary directories
# Ros needs to access /home/${USER_NAME}/.ros/ So we need to set up the permission properly there. 
ARG USER_ID=1000
ARG GROUP_ID=1000
ARG USER_NAME=$(hostname)
RUN echo "Building as user ${USER_NAME}"
RUN groupadd -g ${GROUP_ID} ${USER_NAME} && \
    useradd -m -u ${USER_ID} -g ${GROUP_ID} -s /bin/bash ${USER_NAME} && \
    mkdir -p /home/${USER_NAME}/.ros && \
    chown -R ${USER_NAME}:${USER_NAME} /home/${USER_NAME}

ENV CCACHE_DIR=/ccache
ENV CC="ccache gcc"
ENV CXX="ccache g++"
ENV PATH="/usr/lib/ccache:$PATH"
ENV CUSTOM_INSTALL_PATH="/home/${USER_NAME}/custom_installs"
# Create the ccache directory with appropriate permissions
RUN mkdir -p $CCACHE_DIR && chmod -R 777 $CCACHE_DIR
# RUN mkdir -p ${CUSTOM_INSTALL_PATH}

# Create and add content to the .inputrc file
RUN echo '"\e[A": history-search-backward' >> /home/${USER_NAME}/.inputrc && \
    echo '"\e[B": history-search-forward' >> /home/${USER_NAME}/.inputrc && \
    echo "source /opt/ros/noetic/setup.bash" >> /etc/bash.bashrc

ENV USER_NAME=${USER_NAME}

# Switch to user
USER ${USER_NAME}
WORKDIR /home/${USER_NAME}/dream_cartographer_ws

COPY entrypoint_rgbd_slam_docker.sh /entrypoint_rgbd_slam_docker.sh
# RUN chmod +x /entrypoint_rgbd_slam_docker.sh
ENTRYPOINT ["/entrypoint_rgbd_slam_docker.sh"]
# must follow ENTRYPOINT, otherwise the container won't execute bash
CMD ["bash"]    
