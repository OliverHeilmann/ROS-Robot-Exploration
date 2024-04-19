#!/bin/bash
# Ensure the script exits on first error
set -e

# Function to retry commands
function retry {
    local n=0
    local try=$1
    local cmd="${@: 2}"
    [[ $# -le 1 ]] && {
        echo "Usage: $0 <retry_number> <command>"
        return 1
    }
    until [[ $n -ge $try ]]
    do
        echo "Attempt $((n+1)):"
        $cmd && break || {
            if [[ $n -lt $try ]]; then
                ((n++))
                echo "Command failed. Attempting retry $n..."
                sleep 3;
            else
                echo "The command has failed after $n attempts."
                return 1
            fi
        }
    done
}

# Update package listings
retry 3 sudo apt-get update

# Install required packages
retry 3 sudo apt-get install -y \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    python3-ament-package \
    libopencv-dev \
    ros-$ROS_DISTRO-ament-cmake \
    ros-$ROS_DISTRO-desktop \
    ros-$ROS_DISTRO-cv-bridge \
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
    x11-xserver-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri
