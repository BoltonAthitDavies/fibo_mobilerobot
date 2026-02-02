# ROS2 Project Build Summary

## ✅ Build Status: SUCCESS

The FRA532 Lab 1 project has been successfully built in ROS2 framework!

## Project Overview

This project implements a complete 2D mobile robot localization pipeline with three main components:

### Part 1: EKF Odometry Fusion
- **Package**: `fra532_lab1_part1`
- **Nodes**:
  - `wheel_odometry`: Basic differential drive odometry from wheel encoders
  - `ekf_odometry`: Extended Kalman Filter fusing wheel odometry and IMU data
- **Topics Published**:
  - `/wheel_odom` (nav_msgs/Odometry)
  - `/ekf_odom` (nav_msgs/Odometry)

### Part 2: ICP Odometry Refinement
- **Package**: `fra532_lab1_part2`
- **Node**: `icp_odometry`
- **Topics Published**:
  - `/icp_odom` (nav_msgs/Odometry)
- **Topics Subscribed**:
  - `/scan` (sensor_msgs/LaserScan)
  - `/ekf_odom` (nav_msgs/Odometry) - for initial guess

### Part 3: Full SLAM with slam_toolbox
- **Package**: `fra532_lab1_part3`
- **Configuration**: SLAM Toolbox with loop closure
- **Launch File**: `slam_toolbox.launch.py`
- **Config File**: `slam_toolbox_params.yaml`

## Built Packages

```
fra532_lab1_part1  ✓
fra532_lab1_part2  ✓
fra532_lab1_part3  ✓
```

## Directory Structure

```
fibo_mobilerobot/
├── README.md                      # Original lab instructions
├── README_BUILD.md                # Build and usage instructions
├── FRA532_LAB1_DATASET/          # ROS2 bag files (3 sequences)
├── scripts/
│   ├── plot_trajectories.py     # Python script to plot trajectories
│   └── quick_reference.sh       # Command reference
├── src/
│   ├── fra532_lab1_part1/
│   │   ├── fra532_lab1_part1/
│   │   │   ├── ekf_odometry.py
│   │   │   └── wheel_odometry.py
│   │   ├── launch/
│   │   │   └── odometry.launch.py
│   │   ├── package.xml
│   │   └── setup.py
│   ├── fra532_lab1_part2/
│   │   ├── fra532_lab1_part2/
│   │   │   └── icp_odometry.py
│   │   ├── launch/
│   │   │   └── icp_odometry.launch.py
│   │   ├── package.xml
│   │   └── setup.py
│   └── fra532_lab1_part3/
│       ├── config/
│       │   └── slam_toolbox_params.yaml
│       ├── launch/
│       │   └── slam_toolbox.launch.py
│       ├── package.xml
│       └── setup.py
├── build/                        # Build artifacts
├── install/                      # Install space
└── log/                          # Build logs
```

## Quick Start

### 1. Source the workspace
```bash
cd /home/ambushee/fibo_mobilerobot
source install/setup.bash
```

### 2. Run Part 1 - EKF Odometry
Terminal 1:
```bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/
```

Terminal 2:
```bash
ros2 launch fra532_lab1_part1 odometry.launch.py
```

### 3. Run Part 2 - ICP Odometry
Terminal 1:
```bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/
```

Terminal 2:
```bash
ros2 launch fra532_lab1_part2 icp_odometry.launch.py
```

### 4. Run Part 3 - SLAM
Terminal 1:
```bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/
```

Terminal 2:
```bash
ros2 launch fra532_lab1_part3 slam_toolbox.launch.py
```

## Implementation Details

### Robot Parameters (Turtlebot3 Burger)
- Wheel radius: 0.033 m
- Wheel separation: 0.160 m

### EKF State Vector
- Position: [x, y, θ]
- Velocities: [v, ω]

### ICP Parameters
- Max iterations: 50
- Convergence threshold: 1e-5
- Max correspondence distance: 0.5 m

### SLAM Configuration
- Map resolution: 0.05 m
- Loop closure: Enabled
- Map frame: `map`
- Odom frame: `odom`
- Base frame: `base_link`

## Data Recording

To record odometry data for analysis:
```bash
ros2 bag record /wheel_odom /ekf_odom /icp_odom -o results
```

## Visualization

View in RViz:
```bash
rviz2
```

Add:
- Map (Topic: `/map`)
- LaserScan (Topic: `/scan`)
- TF frames
- Odometry displays

## Testing with Different Sequences

The dataset includes three sequences:

1. **Sequence 00**: Empty hallway (baseline)
   ```bash
   ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/
   ```

2. **Sequence 01**: Sharp turns with obstacles
   ```bash
   ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq01/
   ```

3. **Sequence 02**: Smooth motion with obstacles
   ```bash
   ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq02/
   ```

## Next Steps

1. ✅ Build completed successfully
2. 🔄 Run experiments with all sequences
3. 🔄 Record trajectory data
4. 🔄 Generate comparison plots
5. 🔄 Create 2D maps
6. 🔄 Analyze results and write report

## Troubleshooting

If you encounter issues:

1. **Clean rebuild**:
   ```bash
   rm -rf build install log
   colcon build --symlink-install
   ```

2. **Check topics**:
   ```bash
   ros2 topic list
   ros2 topic hz /scan
   ```

3. **Verify bag file**:
   ```bash
   ros2 bag info FRA532_LAB1_DATASET/fibo_floor3_seq00/
   ```

## Dependencies

- ROS2 (Humble/Iron)
- Python 3
- NumPy
- slam_toolbox

## License

Apache-2.0

## Contact

Maintainer: Ambushee <athit.jake@gmail.com>

---

**Build Date**: February 2, 2026
**Status**: Ready for experiments 🚀
