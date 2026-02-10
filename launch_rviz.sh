#!/bin/bash

# RViz Launcher for FRA532 Lab 1 SLAM
echo "Starting RViz with SLAM configuration..."

# Check if ROS2 environment is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 environment not sourced!"
    echo "Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# Source the workspace
source install/setup.bash 2>/dev/null || echo "No workspace to source"

# Launch RViz with configuration
CONFIG_FILE="config/lab1_slam.rviz"

if [ -f "$CONFIG_FILE" ]; then
    echo "Using config: $CONFIG_FILE"
    rviz2 -d $CONFIG_FILE
else
    echo "Config file not found: $CONFIG_FILE"
    echo "Launching RViz with default configuration..."
    rviz2
fi