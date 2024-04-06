# build the workspace package 'brightness' only - exit if build fails
colcon build --packages-select rosbot --symlink-install || exit 1

# Run the brightness node using ros2 run
# ros2 run brightness brightness_node --ros-args -r /image:=/camera/color/image_raw

# Run the brightness node using launch file
ros2 launch rosbot brightness.yaml 

# Open the Gazebo Simulator in a new terminal
ROSBOT_SIM

# Open the Rviz2
rvis2 -d src/rosbot/rviz/rosbot.rviz