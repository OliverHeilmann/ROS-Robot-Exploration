# ROS-Robot-Exploration

This project aims to build a robotic system using ROS2 and Husarion ROSbot for autonomous exploration and key item detection in an area.

## Requirements

- ROS2: Make sure you have ROS2 installed on your system. You can follow the installation instructions from the official ROS2 documentation: [ROS2 Humble Installation Guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

- Husarion ROSbot: You do **NOT** need a Husarion ROSbot for this project. If you wish to purchase one however, you can find more information from the official Husarion ROSbot website: [Husarion ROSbot](https://husarion.com/tutorials/)

## Dependencies

- Python: The project uses Python for programming. Make sure you have Python installed on your system. You can download Python from the official Python website: [Python Downloads](https://www.python.org/downloads/)

- C++: The project also uses C++ for programming. Make sure you have a C++ compiler installed on your system. You can install the GNU C++ compiler by running the following command:

    ```bash
    sudo apt-get install g++
    ```

- ROS2 Packages: Install the necessary ROS2 packages by running the following command:

    ```bash
    sudo apt-get install ros-<distro>-<package-name>
    ```

    Replace `<distro>` with the ROS2 distribution you are using (e.g., foxy, galactic) and `<package-name>` with the name of the required ROS2 package.

## Getting Started

1. Clone the repository:

     ```bash
     git clone https://github.com/OliverHeilmann/ROS-Robot-Exploration
     ```

2. Build the project:

     ```bash
     cd ROS-Robot-Exploration
     colcon build
     ```

3. Launch the exploration and detection nodes:

     ```bash
     source install/setup.bash
     ros2 launch exploration_detection.launch.py
     ```

4. Follow the on-screen instructions to interact with the robotic system.

## Useful Commands
```sh
# To list all the nodes
ros2 node list
ros2 node info [the-node]

# To list all the topics
ros2 topic list
ros2 topic info [the-node]
ros2 topic echo [the-topic]

# To visualize the ROS graph
rqt_graph

# Created alias for launching the Gazebo simulation
ROSBOT_SIM

# To launch the simulation
rviz2

# To build a specific package
colcon build --packages-select [the-package]

# Install a ROS package
sudo apt-get install ros-$ROS_DISTRO-[the-package-name]

# Call empty service (in this case to save the map)
ros2 service call /save std_srvs/srv/Empty {}
```

## License
This project is licensed under the [MIT License](LICENSE).