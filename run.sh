# /bin/bash

# If you want to use a docker container to do all tests/ dev work, use this script to set up the container and run the simulation.
# I suggest that you use VS Code's docker extension to run things in the container however due to the GUI benefits!

# Install ROS2 and Gazebo here if you don't want to use the container approach:
#   - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
#   - https://gazebosim.org/docs/harmonic/install_ubuntu

# Then run the following commands in the terminal:
#   sh entrypoint.sh

# Build the docker container from the Dockerfile.ROS2 file
docker build -t ros_robot_exploration -f Dockerfile.ROS2 .

# Allow permissions to X11 for showing GUI through docker
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
    xhost +local:root
    export CONTAINER=ros_robot_exploration
    docker run -dt --rm  \
        --name $CONTAINER \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --device /dev/dri:/dev/dri \
        -e TEMP_ROS_WS=/ROS-Robot-Exploration \
        -v $(pwd):/ROS-Robot-Exploration \
        -v $(pwd)/entrypoint.sh:/usr/local/bin/entrypoint.sh \
        ros_robot_exploration:latest

elif [[ "$OSTYPE" == "darwin"* ]]; then
    brew install socat
    nohup socat TCP-LISTEN:6000,reuseaddr,fork UNIX-CLIENT:\"$DISPLAY\" &
    export CONTAINER=ros_robot_exploration
    export DISPLAY=host.docker.internal:0
    docker run -dt --rm \
        --name $CONTAINER \
        --env="DISPLAY=$DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        -e TEMP_ROS_WS=/ROS-Robot-Exploration \
        -v $(pwd):/ROS-Robot-Exploration \
        -v $(pwd)/entrypoint.sh:/usr/local/bin/entrypoint.sh \
        ros_robot_exploration:latest

elif [[ "$OSTYPE" == "msys" ]]; then
    echo "Windows not yet supported, use Mac or Linux instead!"
    exit 1
else
    echo "OS not supported"
fi

# Attach to the container to check on install progress
docker attach $CONTAINER

# Run two programs, each a separate docker terminal using the command below
#   docker exec -it $CONTAINER /bin/bash

# Alternatively, you can open the container with Visual Studio Code's Docker extension (recommended)

# Terminal 1: 
#   ROSBOT_SIM

# Terminal 2: 
#   ros2 launch rosbot explore.launch.py use_gazebo:=true