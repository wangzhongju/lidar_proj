# update date: 20240108

# need to install 12.0.0-cudnn8-devel-ubuntu20.04 image first
# sudo docker pull nvidia/cuda:12.0.0-cudnn8-devel-ubuntu20.04

ARG CUDA=12.0.0-cudnn8-devel-ubuntu20.04
FROM nvidia/cuda:$CUDA
ENV DEBIAN_FRONTEND=noninteractive

RUN sed -i 's#http://archive.ubuntu.com/#http://mirrors.tuna.tsinghua.edu.cn/#' /etc/apt/sources.list && \
    sh -c '. /etc/lsb-release && echo "deb http://mirrors.ustc.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list' && \
    rm /etc/apt/sources.list.d/cuda.list && \
    apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654 && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt update && apt install -y software-properties-common && \
    apt-add-repository universe && \
    apt update && apt install -y \
    python3-pip python2 tmux iputils-ping vim \
    libeigen3-dev libgoogle-glog-dev libopencv-dev libpcap-dev \
    ros-noetic-desktop-full \
    libyaml-cpp-dev libcv-bridge-dev libavdevice-dev ffmpeg \
    libpcl-dev libomp-dev libboost-all-dev libglm-dev libglfw3-dev libpng-dev \
    nlohmann-json3-dev libjpeg-dev  \
    libopenblas-dev python3-pcl pcl-tools \
    libprotobuf-dev protobuf-compiler python3-tk dvipng  \
    texlive-fonts-recommended texlive-fonts-extra \
    python-tk python-yaml && \
    rm -rf /var/lib/apt/lists/*
    
RUN sudo apt-get update && \
    sudo apt-get install -y ros-noetic-grid-map
    
ARG user=sti
ARG passwd=123456
ARG uid=1000
ARG gid=1000
ENV USER=$user
ENV UID=$uid
ENV GID=$gid
RUN useradd --create-home -m $USER && \
        echo "$USER:$passwd" | chpasswd && \
        usermod --shell /bin/bash $USER && \
        # usermod -aG sudo $USER && \
        mkdir -p /etc/sudoers.d && \
        echo "$USER ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/$USER && \
        chmod 0440 /etc/sudoers.d/$USER && \
        # Replace 1000 with your user/group id
        usermod  --uid $uid $USER && \
        groupmod --gid $uid $USER
RUN ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime && \
    chmod 1777 /tmp
USER sti
WORKDIR /home/sti
