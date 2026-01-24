# FRA532 Lab1 Part1: EKF Odometry Fusion

## Overview
This package implements Extended Kalman Filter (EKF) based sensor fusion for mobile robot odometry. It fuses wheel odometry computed from `/joint_states` with IMU measurements from `/imu` to produce a more accurate and reliable odometry estimate.

## Package Structure
```
fra532_lab1_part1/
├── fra532_lab1_part1/
│   ├── __init__.py
│   ├── wheel_odometry_node.py    # Computes wheel odometry from joint states
│   ├── ekf_odometry_node.py      # Fuses wheel odom + IMU using EKF
│   └── trajectory_plotter.py     # Records and plots trajectories
├── launch/
│   └── part1_launch.py           # Launch all nodes
├── config/
│   └── robot_params.yaml         # Robot and EKF parameters
├── package.xml
├── setup.py
└── README.md
```

## Nodes

### 1. `wheel_odometry_node`
Computes odometry from wheel encoder data using differential drive kinematics.

**Subscribed Topics:**
- `/joint_states` (sensor_msgs/JointState): Wheel joint positions

**Published Topics:**
- `/wheel_odom` (nav_msgs/Odometry): Raw wheel odometry

**Parameters:**
- `wheel_radius`: Robot wheel radius (m)
- `wheel_separation`: Distance between wheels (m)

### 2. `ekf_odometry_node`
Implements EKF to fuse wheel odometry with IMU angular velocity measurements.

**Subscribed Topics:**
- `/wheel_odom` (nav_msgs/Odometry): Wheel odometry for prediction step
- `/imu` (sensor_msgs/Imu): IMU data for update step

**Published Topics:**
- `/ekf_odom` (nav_msgs/Odometry): Filtered EKF odometry

**Parameters:**
- `process_noise_x`, `process_noise_y`, `process_noise_theta`: Process noise covariance
- `measurement_noise_omega`: IMU measurement noise covariance

### 3. `trajectory_plotter`
Records trajectory data and generates comparison plots.

**Subscribed Topics:**
- `/wheel_odom` (nav_msgs/Odometry)
- `/ekf_odom` (nav_msgs/Odometry)

**Output:**
- CSV files with trajectory data
- PNG plot comparing wheel and EKF trajectories

## Building and Running

### 1. Build the package
```bash
cd ~/FRA532_LAB
colcon build --packages-select fra532_lab1_part1
source install/setup.bash
```

### 2. Play the rosbag
In a separate terminal:
```bash
cd ~/FRA532_LAB/FRA532_LAB1_DATASET
ros2 bag play fibo_floor3_seq00
```

### 3. Launch the nodes
```bash
ros2 launch fra532_lab1_part1 part1_launch.py
```

### 4. Visualize in RViz (optional)
```bash
rviz2
```
Add:
- Odometry displays for `/wheel_odom` and `/ekf_odom`
- Set Fixed Frame to `odom`

### 5. Stop and save
Press `Ctrl+C` to stop the nodes. The trajectory plotter will automatically save CSV files and generate a comparison plot.

## Parameters Tuning

Edit `config/robot_params.yaml` to adjust:
- Robot dimensions (wheel radius, wheel separation)
- Process noise covariance (Q matrix)
- Measurement noise covariance (R matrix)

Higher process noise → trust measurements more
Higher measurement noise → trust prediction more

## Expected Output

1. **Console output**: Real-time logging from all three nodes
2. **CSV files**: 
   - `wheel_trajectory_TIMESTAMP.csv`
   - `ekf_trajectory_TIMESTAMP.csv`
3. **Plot**: `trajectory_comparison_TIMESTAMP.png` showing:
   - XY trajectory comparison
   - Cumulative distance over time

## Notes

- The EKF implementation uses a 3-state model: [x, y, theta]
- IMU angular velocity is used to correct heading estimation
- The wheel odometry uses midpoint integration for better accuracy
- All angles are normalized to [-π, π]

## Next Steps

After completing Part 1, you can:
1. Test on different sequences (seq01, seq02)
2. Compare drift and accuracy
3. Proceed to Part 2: ICP Odometry Refinement
