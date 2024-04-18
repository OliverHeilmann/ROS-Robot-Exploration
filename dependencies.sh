#!/bin/bash
# Ensure the script exits on first error
set -e

# Update package listings
sudo apt-get update

# Install required packages
sudo apt-get install -y \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    libopencv-dev \
    ros-$ROS_DISTRO-ament-cmake \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-teleop-twist-keyboard \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-teleop-twist-joy \
    ros-$ROS_DISTRO-joy \
    joystick \
    jstest-gtk \
    evtest \
    ros-$ROS_DISTRO-image-geometry \
    qt6-wayland \
    libunwind8-dev \
    libgoogle-glog-dev \
    mesa-utils \
    x11-apps \
    libgl1-mesa-glx \
    libgl1-mesa-dri
