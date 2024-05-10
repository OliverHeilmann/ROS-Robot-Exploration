#!/bin/bash

# Allow local X11 server to be accessed
xhost +local:root || echo "xhost not found. Continuing without it."

# Check if TEMP_ROS_WS is set, set to default if not
if [ -z ${TEMP_ROS_WS+x} ]; then
    echo "TEMP_ROS_WS is not set. Using default value '/ROS-Robot-Exploration'"
    export TEMP_ROS_WS=/ROS-Robot-Exploration

# Check if the workspace directory exists and has the expected structure
elif [ ! -d "$TEMP_ROS_WS" ]; then
    echo "Creating ROS workspace directories at ${TEMP_ROS_WS}"
    mkdir -p ${TEMP_ROS_WS}
fi

# Check if ROS_DISTRO is set, if not, find the appropriate ROS_DISTRO
if [ -z ${ROS_DISTRO+x} ]; then
    echo "ROS_DISTRO is not set. Finding the appropriate ROS_DISTRO (make sure it is installed)."
    export ROS_DISTRO=$(ls /opt/ros/ | grep -v latest | sort -r | head -n 1) || echo "ROS_DISTRO not found, is ROS installed? Exiting." && exit 1
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

# install dependencies from dependencies.sh
chmod +x dependencies.sh
./dependencies.sh

# Setup Husarian submodule
export HUSARION_ROS_BUILD=simulation
sudo apt-get update
sudo apt install ros-dev-tools -y
vcs import modules < modules/rosbot_xl_ros/rosbot_xl/rosbot_xl_hardware.repos
vcs import modules < modules/rosbot_xl_ros/rosbot_xl/rosbot_xl_simulation.repos
sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO

# Install ROS packages with rosdep for imported ROS-Robot-Exploration src modules
rosdep install --from-paths modules --ignore-src -r -y --rosdistro $ROS_DISTRO || exit 1

# Install ROS packages with rosdep for custom ROS-Robot-Exploration src modules
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
source /opt/ros/${ROS_DISTRO}/setup.bash|| exit 1

# # Now everything is set up, build the mounted project workspace
colcon build --symlink-install
source ${TEMP_ROS_WS}/install/setup.bash

# Execute the command specified as CMD in Dockerfile or as command in docker run
exec "$@"