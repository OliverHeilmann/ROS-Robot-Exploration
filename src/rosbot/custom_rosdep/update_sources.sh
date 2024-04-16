#!/bin/bash

# Get the current full path to the workspace
WORKSPACE_PATH=$(pwd)

# Define the relative path to your custom rosdep YAML from the workspace
CUSTOM_YAML_REL_PATH="src/rosbot/custom_rosdep/slam-toolbox.yaml"

# Full path to the custom YAML
FULL_CUSTOM_YAML_PATH="yaml file://$WORKSPACE_PATH/$CUSTOM_YAML_REL_PATH"

# Directory where rosdep sources are stored
ROSDEP_SOURCES_DIR="/etc/ros/rosdep/sources.list.d"

# Find the default rosdep sources file, usually named something like '20-default.list'
DEFAULT_ROSDEP_FILE=$(ls $ROSDEP_SOURCES_DIR | grep -m 1 'default')

# Append the custom path to the default rosdep file
if [ ! -z "$DEFAULT_ROSDEP_FILE" ]; then
    echo "$FULL_CUSTOM_YAML_PATH" | sudo tee -a "$ROSDEP_SOURCES_DIR/$DEFAULT_ROSDEP_FILE" > /dev/null
    echo "Custom rosdep source added to $ROSDEP_SOURCES_DIR/$DEFAULT_ROSDEP_FILE"
else
    echo "No default rosdep source file found."
fi

# Update rosdep to recognize changes
sudo rosdep update
