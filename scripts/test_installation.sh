#!/bin/bash
# Test script to verify all nodes can be launched

echo "========================================="
echo "Testing ROS2 Package Installation"
echo "========================================="
echo ""

# Source the workspace
source /home/ambushee/fibo_mobilerobot/install/setup.bash

echo "✓ Workspace sourced"
echo ""

echo "Checking packages..."
PKG1=$(ros2 pkg list | grep fra532_lab1_part1)
PKG2=$(ros2 pkg list | grep fra532_lab1_part2)
PKG3=$(ros2 pkg list | grep fra532_lab1_part3)

if [ -n "$PKG1" ]; then
    echo "✓ fra532_lab1_part1 found"
else
    echo "✗ fra532_lab1_part1 not found"
fi

if [ -n "$PKG2" ]; then
    echo "✓ fra532_lab1_part2 found"
else
    echo "✗ fra532_lab1_part2 not found"
fi

if [ -n "$PKG3" ]; then
    echo "✓ fra532_lab1_part3 found"
else
    echo "✗ fra532_lab1_part3 not found"
fi

echo ""
echo "Checking executables..."

EXEC1=$(ros2 pkg executables fra532_lab1_part1 | grep wheel_odometry)
EXEC2=$(ros2 pkg executables fra532_lab1_part1 | grep ekf_odometry)
EXEC3=$(ros2 pkg executables fra532_lab1_part2 | grep icp_odometry)

if [ -n "$EXEC1" ]; then
    echo "✓ wheel_odometry executable found"
else
    echo "✗ wheel_odometry executable not found"
fi

if [ -n "$EXEC2" ]; then
    echo "✓ ekf_odometry executable found"
else
    echo "✗ ekf_odometry executable not found"
fi

if [ -n "$EXEC3" ]; then
    echo "✓ icp_odometry executable found"
else
    echo "✗ icp_odometry executable not found"
fi

echo ""
echo "Checking bag files..."

if [ -f "/home/ambushee/fibo_mobilerobot/FRA532_LAB1_DATASET/fibo_floor3_seq00/fibo_floor3_seq00_0.db3" ]; then
    echo "✓ Sequence 00 bag file found"
else
    echo "✗ Sequence 00 bag file not found"
fi

if [ -f "/home/ambushee/fibo_mobilerobot/FRA532_LAB1_DATASET/fibo_floor3_seq01/fibo_floor3_seq01_0.db3" ]; then
    echo "✓ Sequence 01 bag file found"
else
    echo "✗ Sequence 01 bag file not found"
fi

if [ -f "/home/ambushee/fibo_mobilerobot/FRA532_LAB1_DATASET/fibo_floor3_seq02/fibo_floor3_seq02_0.db3" ]; then
    echo "✓ Sequence 02 bag file found"
else
    echo "✗ Sequence 02 bag file not found"
fi

echo ""
echo "========================================="
echo "Test complete!"
echo "========================================="
