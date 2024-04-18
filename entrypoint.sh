#!/bin/bash

# Allow local X11 server to be accessed
xhost +local:root

# Check if the workspace directory exists and has the expected structure
if [ ! -d "$TEMP_ROS_WS" ]; then
    echo "Creating ROS workspace directories at ${TEMP_ROS_WS}"
    mkdir -p ${TEMP_ROS_WS}
fi

# Check if ROS_DISTRO is set, set to default if not
if [ -z ${ROS_DISTRO+x} ]; then
    echo "ROS_DISTRO is not set. Using default value 'humble"
    export ROS_DISTRO=humble
    exit 1
fi

# cd into the ROS-Robot-Exploration directory if not already there
target_dir="ROS-Robot-Exploration"
current_dir=$(basename "$(pwd)")
if [ "$current_dir" = "$target_dir" ]; then
    echo "Already in the ROS-Robot-Exploration directory."
elif [ -d "$target_dir" ]; then
    echo "ROS-Robot-Exploration directory found. Changing to it."
    cd "$target_dir"
else
    echo "ROS-Robot-Exploration directory not found. Exiting."
    exit 1
fi

# Setup Husarian submodule
export HUSARION_ROS_BUILD=simulation
sudo apt-get update
sudo apt install ros-dev-tools -y
vcs import modules < modules/rosbot_xl_ros/rosbot_xl/rosbot_xl_hardware.repos
vcs import modules < modules/rosbot_xl_ros/rosbot_xl/rosbot_xl_simulation.repos
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install -i --from-path modules --rosdistro $ROS_DISTRO -y || exit 1

# # Install ROS packages with rosdep from ROS-Robot-Exploration dir
rosdep install --from-paths src --ignore-src -r -y --rosdistro $ROS_DISTRO

# Source ROS setup, put in .bashrc if not already there
if ! grep -q "source /opt/ros/${ROS_DISTRO}/setup.bash" ~/.bashrc; then
    echo "Adding source /opt/ros/${ROS_DISTRO}/setup.bash to ~/.bashrc"
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
fi

# Source the workspace setup, put in .bashrc if not already there
if ! grep -q "source ${TEMP_ROS_WS}/install/setup.bash" ~/.bashrc; then
    echo "Adding source ${TEMP_ROS_WS}/install/setup.bash to ~/.bashrc"
    echo "source ${TEMP_ROS_WS}/install/setup.bash" >> ~/.bashrc
fi

# Add alias for running the simulation (use ROSBOT_SIM to run the simulation)
if ! grep -q "alias ROSBOT_SIM='ros2 launch rosbot_xl_gazebo simulation.launch.py'" ~/.bashrc; then
    echo "Adding alias ROSBOT_SIM='ros2 launch rosbot_xl_gazebo simulation.launch.py' to ~/.bashrc"
    echo "alias ROSBOT_SIM='ros2 launch rosbot_xl_gazebo simulation.launch.py'" >> ~/.bashrc
fi

# Update the workspace/ environment with the new packages
source ${TEMP_ROS_WS}/install/setup.bash

# Now everything is set up, build the mounted project workspace
rm -rf build/ install/ log/
colcon build --symlink-install
source ${TEMP_ROS_WS}/install/setup.bash

# Execute the command specified as CMD in Dockerfile or as command in docker run
exec "$@"