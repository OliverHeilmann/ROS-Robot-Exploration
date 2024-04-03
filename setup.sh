# Assumed that ROS2 Humbe and Gazebo Harmonic are installed already!

# Install here if not:
#   - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
#   - https://gazebosim.org/docs/harmonic/install_ubuntu

# Setup submodules
git submodule init

# Replace ".bash" with your shell if you're not using bash
# Possible values are: setup.bash, setup.sh, setup.zsh
source /opt/ros/humble/setup.bash

# Setup Husarian submodule
sudo apt-get update
sudo apt install ros-dev-tools
vcs import modules < modules/rosbot_xl_ros/rosbot_xl/rosbot_xl_hardware.repos
vcs import modules < modules/rosbot_xl_ros/rosbot_xl/rosbot_xl_simulation.repos
sudo rosdep init
export ROS_DISTRO=humble
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path modules --rosdistro $ROS_DISTRO -y