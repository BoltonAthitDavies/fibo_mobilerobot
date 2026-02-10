# Part 1: EKF Odometry Fusion - Implementation Guide

## Overview
Part 1 implements Extended Kalman Filter (EKF) based sensor fusion for mobile robot odometry. This package fuses wheel odometry (computed from `/joint_states`) with IMU measurements (`/imu`) to provide improved pose estimation.

## Package Structure
```
fra532_lab1_part1/
├── launch/
│   ├── ekf_odometry.launch.py           # Main EKF launch file
│   └── simple_wheel_odometry.launch.py # Wheel-only odometry
├── config/
│   └── ekf_params.yaml                  # EKF configuration parameters  
├── fra532_lab1_part1/
│   ├── ekf_odometry.py                  # Main EKF implementation (TEMPLATE)
│   ├── simple_wheel_odometry.py         # Simple wheel odometry reference
│   └── trajectory_logger.py             # Data logging utility
└── README.md                           # This file
```

## Implementation Tasks

### 🚨 **IMPORTANT: Templates Provided**
The main EKF implementation is provided as **templates with TODO sections**. You need to implement:

1. **EKF Prediction Step** (`prediction_step()`)
   - Implement motion model prediction
   - Predict state covariance forward

2. **Wheel Odometry Measurement Update** (`measurement_update_odom()`)
   - Extract wheel positions from joint_states  
   - Compute odometry measurement
   - EKF measurement update with wheel odometry

3. **IMU Measurement Update** (`measurement_update_imu()`)
   - Extract angular velocity from IMU
   - EKF measurement update with IMU data

### 📐 **EKF State and Models**

**State Vector:** `x = [x, y, θ]ᵀ`

**Motion Model:**
```
x(k+1) = x(k) + Δx
y(k+1) = y(k) + Δy  
θ(k+1) = θ(k) + Δθ
```

**Measurements:**
- Wheel odometry: `[Δx, Δy, Δθ]ᵀ` from differential drive kinematics
- IMU: `ω_z` (angular velocity around z-axis)

## Quick Start

### 1. Build the Package
```bash
colcon build --packages-select fra532_lab1_part1
source install/setup.bash
```

### 2. Test Simple Wheel Odometry
```bash
# Terminal 1: Start simple wheel odometry
ros2 launch fra532_lab1_part1 simple_wheel_odometry.launch.py use_sim_time:=true

# Terminal 2: Play bag file
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ --clock --qos-profile-overrides-path qos_overrides.yaml

# Terminal 3: Monitor output
ros2 topic echo /wheel_odom --once
```

### 3. Implement and Test EKF
After implementing the TODO sections:
```bash
# Terminal 1: Start EKF odometry  
ros2 launch fra532_lab1_part1 ekf_odometry.launch.py use_sim_time:=true

# Terminal 2: Play bag file
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ --clock --qos-profile-overrides-path qos_overrides.yaml

# Terminal 3: Monitor EKF output
ros2 topic echo /ekf_odom --once
```

### 4. Log Trajectories
```bash
# Start trajectory logger
ros2 run fra532_lab1_part1 trajectory_logger

# Run your odometry methods and play bag files
# Trajectories will be saved in 'part1_results/' directory
```

## Configuration

### EKF Parameters (`config/ekf_params.yaml`)
Key parameters to tune:

- **Process Noise (`Q` matrix):**
  - `process_noise.position_x: 0.01`
  - `process_noise.position_y: 0.01` 
  - `process_noise.orientation: 0.1`

- **Measurement Noise (`R` matrices):**
  - `wheel_odom_x/y/theta`: Wheel odometry uncertainty
  - `imu_angular_velocity`: IMU angular velocity noise

- **Frequency:**
  - `prediction_frequency: 20.0` # Hz

## Implementation Hints

### 1. Wheel Odometry Computation
```python
# From joint_states, compute:
delta_left = left_wheel_pos - prev_left_wheel_pos  
delta_right = right_wheel_pos - prev_right_wheel_pos

# Convert to distances
left_dist = delta_left * wheel_radius
right_dist = delta_right * wheel_radius

# Robot motion
distance = (left_dist + right_dist) / 2.0
delta_theta = (right_dist - left_dist) / wheel_separation

# Position update (non-linear motion model)
if abs(delta_theta) > 1e-6:
    # Curved motion
    radius = distance / delta_theta
    dx = radius * (sin(theta + delta_theta) - sin(theta))
    dy = radius * (-cos(theta + delta_theta) + cos(theta))
else:
    # Straight motion
    dx = distance * cos(theta)
    dy = distance * sin(theta)
```

### 2. EKF Prediction Step
```python
# Predict state (simple constant velocity model)
# x_pred = x_prev (no control input assumed)

# Predict covariance 
# P_pred = F * P * F.T + Q
# where F is Jacobian of motion model (3x3 identity for simple case)
```

### 3. EKF Measurement Update
```python
# Innovation: y = z_measured - h(x_pred)
# Kalman gain: K = P * H.T * (H * P * H.T + R)^-1  
# State update: x = x_pred + K * y
# Covariance update: P = (I - K * H) * P
```

## Output Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/wheel_odom` | `nav_msgs/Odometry` | Simple wheel odometry |
| `/ekf_odom` | `nav_msgs/Odometry` | EKF fused odometry |

## Analysis and Comparison

Use the trajectory logger and analysis tools to compare:
- Wheel-only odometry drift
- EKF odometry performance  
- Effect of IMU fusion on accuracy
- Covariance evolution over time

Results will be used for comparison with Part 2 (ICP) and Part 3 (SLAM).

## Troubleshooting

### Common Issues
1. **No joint_states**: Check bag file topics and QoS settings
2. **EKF divergence**: Tune process/measurement noise parameters
3. **High drift**: Check wheel parameters (radius, separation)
4. **IMU not working**: Verify IMU topic and frame alignment

### Debug Commands
```bash
# Check available topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states --once

# Check IMU data  
ros2 topic echo /imu --once

# Verify transforms
ros2 run tf2_tools view_frames.py
```