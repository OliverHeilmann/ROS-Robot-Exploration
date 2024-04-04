# ros2 launch launch/talker-listener.yaml

colcon build --packages-select brightness --symlink-install
# ros2 run brightness brightness_node --ros-args -r /image:=/camera/color/image_raw
ros2 launch launch/brightness.yaml 