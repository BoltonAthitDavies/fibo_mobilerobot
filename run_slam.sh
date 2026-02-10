#!/bin/bash

# Part 3 SLAM Pipeline Runner
# This script demonstrates how to run the complete SLAM pipeline for Lab 1 Part 3

echo "=========================================="
echo "FRA532 LAB1 PART 3 - SLAM Pipeline"
echo "=========================================="

# Check if ROS2 environment is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "ERROR: ROS2 environment not sourced!"
    echo "Please run: source /opt/ros/<distro>/setup.bash"
    exit 1
fi

# Source the workspace
echo "Sourcing workspace..."
source install/setup.bash

# Function to run SLAM with a specific dataset sequence
run_slam_sequence() {
    local sequence=$1
    echo "Running SLAM on sequence: $sequence"
    
    echo "Step 1: Starting SLAM Toolbox..."
    # Start SLAM in background
    ros2 launch fra532_lab1_part3 slam.launch.py use_sim_time:=true &
    SLAM_PID=$!
    sleep 3
    
    echo "Step 2: Starting SLAM Evaluator (for trajectory and map saving)..."
    # Start evaluator in background  
    ros2 run fra532_lab1_part3 slam_evaluator &
    EVAL_PID=$!
    sleep 2
    
    echo "Step 3: Playing bag file..."
    # Play the bag file
    ros2 bag play FRA532_LAB1_DATASET/$sequence/ \
        --clock \
        --qos-profile-overrides-path qos_overrides.yaml \
        --rate 1.0
    
    echo "Step 4: Saving final map..."
    # Save the map
    ros2 run fra532_lab1_part3 map_saver ${sequence}_slam_map
    
    echo "Step 5: Cleaning up..."
    # Stop background processes
    kill $SLAM_PID 2>/dev/null
    kill $EVAL_PID 2>/dev/null
    sleep 2
    
    echo "SLAM completed for $sequence!"
    echo ""
}

# Main execution
echo "Available sequences:"
echo "1. fibo_floor3_seq00 (Empty Hallway)"
echo "2. fibo_floor3_seq01 (Non-Empty with Sharp Turns)"
echo "3. fibo_floor3_seq02 (Non-Empty with Non-Aggressive Motion)"
echo ""

# Check command line argument
if [ $# -eq 0 ]; then
    echo "Usage: $0 <sequence_number>"
    echo "Example: $0 1  (for fibo_floor3_seq00)"
    echo "Example: $0 2  (for fibo_floor3_seq01)"  
    echo "Example: $0 3  (for fibo_floor3_seq02)"
    echo "Example: $0 all  (for all sequences)"
    exit 1
fi

# Run based on user input
case $1 in
    1)
        run_slam_sequence "fibo_floor3_seq00"
        ;;
    2) 
        run_slam_sequence "fibo_floor3_seq01"
        ;;
    3)
        run_slam_sequence "fibo_floor3_seq02" 
        ;;
    all)
        echo "Running SLAM on all sequences..."
        run_slam_sequence "fibo_floor3_seq00"
        sleep 5
        run_slam_sequence "fibo_floor3_seq01"
        sleep 5
        run_slam_sequence "fibo_floor3_seq02"
        ;;
    *)
        echo "Invalid sequence number. Use 1, 2, 3, or 'all'"
        exit 1
        ;;
esac

echo "=========================================="
echo "SLAM Pipeline Completed!"
echo "Check the 'maps' and 'slam_results' directories for outputs."
echo "=========================================="