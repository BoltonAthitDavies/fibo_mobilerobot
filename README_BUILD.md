# FRA532 Lab 1 - ROS2 Implementation Guide

This repository contains the ROS2 implementation for LAB1: Kalman Filter / SLAM.

## Project Structure

```
fibo_mobilerobot/
├── FRA532_LAB1_DATASET/          # ROS2 bag files
│   ├── fibo_floor3_seq00/
│   ├── fibo_floor3_seq01/
│   └── fibo_floor3_seq02/
├── src/
│   ├── fra532_lab1_part1/        # Part 1: EKF Odometry Fusion
│   ├── fra532_lab1_part2/        # Part 2: ICP Odometry Refinement
│   └── fra532_lab1_part3/        # Part 3: SLAM with slam_toolbox
└── README_BUILD.md               # This file
```

## Prerequisites

- ROS2 (Humble or later recommended)
- Python 3
- NumPy
- slam_toolbox (for Part 3)

## Installation

### 1. Install slam_toolbox

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-slam-toolbox
```

### 2. Build the workspace

```bash
cd /home/ambushee/fibo_mobilerobot
colcon build --symlink-install
source install/setup.bash
```

## Usage

### Part 1: EKF Odometry Fusion

This part implements an Extended Kalman Filter to fuse wheel odometry and IMU measurements.

#### Running Part 1

Terminal 1 - Play the ROS2 bag:
```bash
cd /home/ambushee/fibo_mobilerobot
source install/setup.bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/
```

Terminal 2 - Run the odometry nodes:
```bash
source install/setup.bash
ros2 launch fra532_lab1_part1 odometry.launch.py
```

Or run nodes individually:
```bash
# Wheel odometry only
ros2 run fra532_lab1_part1 wheel_odometry

# EKF odometry
ros2 run fra532_lab1_part1 ekf_odometry
```

#### Published Topics

- `/wheel_odom` - Raw wheel odometry (Odometry message)
- `/ekf_odom` - EKF-fused odometry (Odometry message)

### Part 2: ICP Odometry Refinement

This part refines the EKF odometry using LiDAR scan matching with ICP.

#### Running Part 2

Terminal 1 - Play the ROS2 bag:
```bash
cd /home/ambushee/fibo_mobilerobot
source install/setup.bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/
```

Terminal 2 - Run ICP odometry:
```bash
source install/setup.bash
ros2 launch fra532_lab1_part2 icp_odometry.launch.py
```

Or run individually:
```bash
ros2 run fra532_lab1_part2 icp_odometry
```

#### Published Topics

- `/icp_odom` - ICP-refined odometry (Odometry message)

### Part 3: Full SLAM with slam_toolbox

This part performs full SLAM with loop closure using slam_toolbox.

#### Running Part 3

Terminal 1 - Play the ROS2 bag:
```bash
cd /home/ambushee/fibo_mobilerobot
source install/setup.bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/
```

Terminal 2 - Run SLAM:
```bash
source install/setup.bash
ros2 launch fra532_lab1_part3 slam_toolbox.launch.py
```

#### Visualization

To visualize the SLAM output in RViz:
```bash
rviz2
```

Add displays for:
- Map (Topic: `/map`)
- LaserScan (Topic: `/scan`)
- TF frames

## Visualization and Analysis

### View all odometry topics

```bash
ros2 topic list
```

### Record odometry data for analysis

```bash
ros2 bag record /wheel_odom /ekf_odom /icp_odom /slam_pose -o output_bag
```

### Echo odometry data

```bash
# Wheel odometry
ros2 topic echo /wheel_odom

# EKF odometry
ros2 topic echo /ekf_odom

# ICP odometry
ros2 topic echo /icp_odom
```

## Robot Parameters

The implementation uses Turtlebot3 Burger dimensions:
- Wheel radius: 0.033 m
- Wheel separation: 0.160 m

These parameters are used in the differential drive motion model for odometry computation.

## Dataset Sequences

### Sequence 00 - Empty Hallway
Static indoor environment, minimal obstacles
```bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/
```

### Sequence 01 - Non-Empty Hallway with Sharp Turns
Indoor environment with obstacles and sharp turns
```bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq01/
```

### Sequence 02 - Non-Empty Hallway with Smooth Motion
Indoor environment with obstacles, non-aggressive motion
```bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq02/
```

## Troubleshooting

### TF Transform Warnings
If you see TF transform warnings, ensure that:
1. The bag file is playing correctly
2. The nodes are receiving data (check with `ros2 topic hz`)

### No odometry output
Check that the topics are being published:
```bash
ros2 topic list
ros2 topic hz /joint_states
ros2 topic hz /imu
ros2 topic hz /scan
```

### Build errors
Clean and rebuild:
```bash
rm -rf build install log
colcon build --symlink-install
```

## Development Notes

### Part 1 Implementation
- `wheel_odometry.py`: Basic differential drive odometry
- `ekf_odometry.py`: Extended Kalman Filter for sensor fusion

### Part 2 Implementation
- `icp_odometry.py`: Iterative Closest Point for scan matching

### Part 3 Configuration
- `slam_toolbox_params.yaml`: SLAM parameters
- `slam_toolbox.launch.py`: Launch file for SLAM

## Next Steps

1. Run experiments with all three sequences
2. Record trajectory data for comparison
3. Generate plots comparing:
   - Wheel odometry
   - EKF odometry
   - ICP odometry
   - SLAM pose output
4. Create 2D maps from ICP odometry and slam_toolbox
5. Analyze accuracy, drift, and robustness

## License

Apache-2.0

## Maintainer

Ambushee <athit.jake@gmail.com>
