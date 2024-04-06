# Assumed that Gazebo Harmonic are installed already!

# Install here if not:
#   - ROS2: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
#   - Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install_ubuntu

export ROS_DISTRO=humble

# Install ROS2 Humble
sudo apt update && sudo apt install software-properties-common lsb-release -y
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-$ROS_DISTRO-ros-base ros-$ROS_DISTRO-ament-cmake python3-pip python3-colcon-common-extensions python3-vcstool python3-rosdep
sudo rosdep init
source /opt/ros/$ROS_DISTRO/setup.bash

# Setup Husarian submodule
sudo apt-get update
sudo apt install ros-dev-tools
vcs import modules < modules/rosbot_xl_ros/rosbot_xl/rosbot_xl_hardware.repos
vcs import modules < modules/rosbot_xl_ros/rosbot_xl/rosbot_xl_simulation.repos
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path modules --rosdistro $ROS_DISTRO -y || exit 1

# Build the workspace in top level directory (gitignore is set to ignore build directories)
export HUSARION_ROS_BUILD=simulation
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install || exit 1

# Permanently add ROS2 workspace to the environment (if install was successful)
echo 'source ~/ROS-Robot-Exploration/install/setup.bash' >> ~/.bashrc

# Add alias for running the simulation (use ROSBOT_SIM to run the simulation)
echo "alias ROSBOT_SIM='ros2 launch rosbot_xl_gazebo simulation.launch.py'" >> ~/.bashrc
. ~/.bashrc