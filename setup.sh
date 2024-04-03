# Assumed that ROS2 Humbe and Gazebo Harmonic are installed already!

# Install here if not:
#   - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
#   - https://gazebosim.org/docs/harmonic/install_ubuntu

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
rosdep install -i --from-path modules --rosdistro $ROS_DISTRO -y || exit 1

# Build the workspace in top level directory (gitignore is set to ignore build directories)
export HUSARION_ROS_BUILD=simulation
source /opt/ros/humble/setup.bash
colcon build --symlink-install || exit 1

# Permanently add ROS2 workspace to the environment (if install was successful)
echo 'source ~/ROS-Robot-Exploration/install/setup.bash' >> ~/.bashrc

# Add alias for running the simulation (use ROSBOT_SIM to run the simulation)
echo "alias ROSBOT_SIM='ros2 launch rosbot_xl_gazebo simulation.launch.py'" >> ~/.bashrc
. ~/.bashrc