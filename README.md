# ROS-Robot-Exploration

This project aims to build robotic systems using ROS2 and physics simulators such as Gazebo. So far, the Husarion ROSbot has been used for autonomous exploration and mapping. Next, a similar system build is achieved with a Clearpath Robotics robot (`a200_0000`). The project is still in progress and will be updated as new features are added.

## Progress So Far...

![Clearpath 1](images/Clearpath1.png)
![RViz Screenshot 6](images/Explore1.png)
![RViz Screenshot 6](images/Explore2.png)
![RViz Screenshot 6](images/Explore3.png)
![RViz Screenshot 6](images/Navigation.png)
![RViz Screenshot 5](images/amcl.png)
![RViz Screenshot 4](images/2D-Slam.png)
![RViz Screenshot 3](images/transforms.png)
![RViz Screenshot 2](images/KCF-tracking.png)

## Getting Started
### Using Host Machine
To run this project on your host machine, you will need to have ROS2 humble installed on a Ubuntu 22.04 system. If you do not have ROS2 installed, you can follow the instructions on the [ROS2 humble website](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

Once you have ROS2 installed, you can clone this repository and build the project using the following commands:

1. Clone the repository:

    ```bash
    git clone https://github.com/OliverHeilmann/ROS-Robot-Exploration --recurse-submodules
    cd ROS-Robot-Exploration
    ```

2. Install the necessary dependencies and build the project:

    ```bash
    chmod +x entrypoint.sh
    ./entrypoint.sh
    ```

3. Launch Gazebo:

    ```bash
    ROSBOT_SIM
    ```

4. Launch the Exploration ROSbot code (new terminal):

    ```bash
    ros2 launch rosbot explore.launch.py use_gazebo:=true
    ```


### Using Docker
Here, we are using Docker to make the setup process easier. The Docker container will have ROS2 installed and the ROSbot simulation running. Additionally, all the GUI tools will be forwarded to the host machine so you can run this project on any machine that supports Docker! There are a few things to setup if you are using a Mac or Windows machine first however (see below).

### Linux
Nothing to do here! Just ensure you have Docker installed on your machine.

### MacOS
To forward X11 from inside a docker container to a host running macOS

1. Install XQuartz: https://www.xquartz.org/
2. Launch XQuartz.  Under the XQuartz menu, select Preferences
3. Go to the security tab and ensure "Allow connections from network clients" is checked.
4. Restart XQuartz and Docker Desktop (if running). If you experience any issues, restart your computer as well or follow this [Gist](https://gist.github.com/cschiewek/246a244ba23da8b9f0e7b11a68bf3285) guide.

### Windows
WIP! Need to check if X11 forwarding works on Windows. If not, will need to use VcXsrv or similar.

### Building and Running the Project
1. Clone the repository:

    ```bash
    git clone https://github.com/OliverHeilmann/ROS-Robot-Exploration --recurse-submodules
    cd ROS-Robot-Exploration
    ```

2. Build the Docker container using the provided Dockerfile. *Note: This step will take around 20 minutes to build dependencies and the project*:

    ```bash
    chmod +x run.sh
    ./run.sh
    ```

3. Launch Gazebo in a new terminal (Note: works on Ubuntu 22.04, some isses with X11 forwarding Ubuntu 20.04):

    ```bash
    docker exec -it ros_robot_exploration /bin/bash

    # Then inside the container
    ROSBOT_SIM

    # Alternatively, launch headless (may work on Ubuntu 20.04 depending on your setup)
    ros2 launch rosbot_xl_gazebo simulation.launch.py headless:=True
    ```

4. Launch the Exploration ROSbot code (new terminal):

    ```bash
    docker exec -it ros_robot_exploration /bin/bash

    # Then inside the container
    ros2 launch rosbot explore.launch.py use_gazebo:=true
    ```

<!-- 4. Drive the robot around using your keyboard. Run the teleop_twist_keyboard executable as shown below:
    
    ```bash
    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ``` -->

That's it! You should now have a Gazebo simulation running with the ROSbot exploring the environment.

*Note: If you are experiencing issues stopping the docker container, try using the command below (taken from [Stack Overflow issue](https://stackoverflow.com/questions/47223280/docker-containers-can-not-be-stopped-or-removed-permission-denied-error)):*

```bash
 sudo aa-remove-unknown
```

## Useful Information and Guides
### Networking with ROS2

ROS will work over LAN so, if you have two devices on the same network, you will be able to publish and subscribe to the same topics i.e. they are visible to one another. See example below:
```sh
# ROSbot: Create a /msg topic and send send at rate=1s
ros2 topic pub -r 1 /msg std_msgs/msg/String data:\ 'Hello, ROSbot here'

# Laptop: Echo the /msg topic
ros2 topic echo /msg
```

Using `ROS_DISCOVERY_SERVER` segregates 'robot networks' within the same LAN. Instead of depending on the standard multicast-based LAN discovery, which is the default for DDS, you have the option to transition to the Discovery Server approach for DDS discovery.
     
```sh
# ROSbot: Start the Discovery Server
fastdds discovery --server-id 0 --port 11888

export ROS_DISCOVERY_SERVER="10.5.10.130:11888"
ros2 daemon stop # reload ROS 2 daemon

ros2 run demo_nodes_cpp talker
```

```sh
# Laptop: Start the Discovery Server
export ROS_DISCOVERY_SERVER="10.5.10.130:11888"
ros2 daemon stop # reload ROS 2 daemon

ros2 run demo_nodes_cpp listener
```

If you want to access ROS2 nodes over the internet, [Husarian](https://husarion.com/tutorials/ros2-tutorials/6-robot-network/#connecting-ros-2-via-internet) provides a tutorial on how to do this using their cloud service. It is essentially a VPN service that allows you to connect to your robot from anywhere in the world.

Some interesting suggestions for security across the ROS2 network are mentioned in this [LinkedIn](https://www.linkedin.com/advice/0/how-can-you-secure-your-ros-system-from-cyber-threats-hgw0c) article. including [rosauth](https://wiki.ros.org/rosauth), [sros](https://docs.ros.org/en/rolling/Tutorials/Advanced/Security/Introducing-ros2-security.html), or [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite/blob/ros2/README.md).

### Transformations

In separate terminals, run the following commands to create static transformations between the map, robot, and camera frames and visualize them in RViz:
```sh
# Create static transformations between the map, robot, and camera frames
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id robot --x 1 --y -1 --yaw 1.6

# Create static transformations between the robot and camera frames
ros2 run tf2_ros static_transform_publisher --frame-id robot --child-frame-id reverse_camera --z 0.2 --yaw 3.14 

# Visualize the transformations in RViz
rviz2 -d src/rosbot/rviz/tf.rviz 
```

Alternatively, you can create a launch file to run all the commands at once. Below is an example of the contents of this launch file:
```xml
<launch>
    <node pkg="tf2_ros" type="static_transform_publisher" name="map_to_robot" args="--frame-id map --child-frame-id robot --x 1 --y -1 --yaw 1.6" />
    <node pkg="tf2_ros" type="static_transform_publisher" name="robot_to_camera" args="--frame-id robot --child-frame-id reverse_camera --z 0.2 --yaw 3.14" />
</launch>
```

Another example shows how a transformation can occur over time, where the robot moves in a circle about the map centre coordinates. Use the launch file as shown below to observe this:
```sh
ros2 launch rosbot tf_broadcaster.yaml
```

### SLAM and AMCL
To perform SLAM, you can use the `slam_toolbox` package. This package provides a set of tools for 2D and 3D SLAM. To install the package, run the following command:

```sh
sudo apt-get install ros-<distro>-slam-toolbox
```

To run the SLAM toolbox, use the following command:

```sh
ros2 launch rosbot slam.launch.py use_sim_time:=true

# To visualize the map in RViz
rviz2 -d src/rosbot/rviz/slam.rviz

# To load a saved map, navigate to the map directory and run the below command
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=[your-map-name].yaml

# Then run the map server to load it into RVIZ
ros2 run nav2_util lifecycle_bringup map_server
```

To use `Adaptive Monte Carlo Location` with ROS2, we can use the `amcl` package. This package provides a probabilistic localisation system for a robot moving in 2D. To run this package, use the following command (after running Gazebo):

```sh
# Launch Gazebo simulator
ROSBOT_SIM

# To launch the AMCL package
ros2 launch rosbot amcl.launch.py
```

### Navigation
To perform navigation, you can use the `nav2` package. This package provides a set of tools for 2D and 3D navigation. To install the package, run the following command:

```sh
sudo apt-get install ros-$ROS_DISTRO-navigation2
```

Launch the navigation stack using the following command:
```sh
# To launch the navigation stack
ros2 launch rosbot navigation.launch.py
```

More detail (and links to even further detail) can be found on [Husarion Docs](https://husarion.com/tutorials/ros2-tutorials/9-navigation/) but, for my own reference, below are some high level comments on navigation2 principles:

- `amcl` - adaptive Monte Carlo location used for localisation of the robot. It uses a particle filter to estimate the robot's position on a *known* map.
- `behavior_server` - to configure recovery behavior, for instance when the robot is stuck.
- `bt_navigator` - allows you to change the behavior of services to create a unique robot behavior (default behavior tree will be used).
- `controller_server` - reates a local costmap and implements the server for handling the controller requests. Consider how the controller is responsible for taking actions which will move the robot to the goal, but ensuring that the robot avoids obstacles and stays on the intended path. An example is `Regulated Pure Pursuit` which adjusts "steering" based on a look ahead distance to the intended path (some similarities to PID controllers for smoothing robot motion).
- `local_costmap/global_costmap` - The costmap is created based on the provided map and data from sensors such as cameras and laser scanners that measure whether there are obstacles in the way. Then it creates a special map where each spot has a "cost" value. `local_costmap` is used for local planning and `global_costmap` is used for global planning, where local planning is used to avoid obstacles and global planning is used to find the shortest path to the goal.
- `Smoother Server` - creates smoother path plans to be more continuous and feasible. This may be useful for robots with non-holonomic constraints e.g. ackermann steering. Consider inflating the costmap to account for the robot's footprint and turning radius.
- `planner_server` - related to the selection and fine-tuning of the global path,
- `waypoint_follower` - related to following a multi-point route,
- `velocity_smoother` - to smooth out the robot's motion.


### Joystick Robot Control
Use a joystick to control the robot. The `joy` package provides a node that interfaces with a joystick and publishes the joystick commands to the `/joy` topic. To use the package, run the following commands:

```sh
# See if you have a connected controller visible by the system
evtest

# See which controllers are connected, and which ID you want to use
ros2 run joy joy_enumerate_devices

# Run the joy node to publish joystick commands to the /joy topic
ros2 run joy joy_node

# To see the joystick commands, run the following command
ros2 topic echo /joy

# When you have the joystick commands, you can use the teleop_twist_joy package to control the robot. Use the Left Trigger and Left Stick to control the robot's linear and angular velocities.
ros2 launch rosbot joystick.launch.py
```

### Exploration
To perform exploration, you can use the `explore_lite` package. This package provides a lightweight exploration algorithm for a robot moving in 2D. To install the package, run the following command (after running Gazebo):

```sh
ros2 launch rosbot explore.launch.py use_gazebo:=true
```

### Clearpath Robotics
To use the Clearpath Robotics simulation environment within this workspace, you will have to  add a `robot.yaml` file to the `modules/clearpath/` directory, which can be found in the [Clearpath Documentation](https://docs.clearpathrobotics.com/docs/ros/config/yaml/overview#sample) for example.

Now launch the simulation using the following command:

```sh
# Make sure to add the path to your clearpath directory - this repo has it in the following location
ros2 launch clearpath_gz simulation.launch.py setup_path:=$HOME/ROS-Robot-Exploration/modules/clearpath/

# Change the world to your desired world
ros2 launch clearpath_gz simulation.launch.py world:=my_world
```

<!--
### Useful Commands
```sh
# clone a repository as a submodule (ensure it is not gitignored)
git submodule add [the-repository-to-clone]  [the-directory-to-clone-into]

# To update the submodules
git submodule update --init --recursive --remote

ln -s ../modules modules

```

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
rviz2 -d ~/[path-to-rviz-file]/rosbot.rviz

# To inspect logged information with a UI
ros2 run plotjuggler plotjuggler

# To build a specific package
colcon build --packages-select [the-package]

# To source the ROS workspace after building new packages
source ~/ROS-Robot-Exploration/install/setup.bash

# Install a ROS package
sudo apt-get install ros-$ROS_DISTRO-[the-package-name]

# Create a ROS package
ros2 pkg create [package-name] --build-type ament_cmake --dependencies [the-dependencies]

# List your packages
colcon list

# Call empty service (in this case to save the map)
ros2 service call /save std_srvs/srv/Empty {}
ros2 service call /image_counter std_srvs/srv/Trigger {}

# Get specific field information from an echo terminal command
ros2 topic echo /odometry/filtered --field pose.pose
```
-->

## License
This project is licensed under the [MIT License](LICENSE).