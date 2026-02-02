# FRA532 Lab 1 - Complete Project Index

## 📁 Project Documentation

This project contains a complete ROS2 implementation for mobile robot localization and SLAM.

### Main Documentation Files

1. **[README.md](README.md)** - Original lab assignment and requirements
2. **[BUILD_SUMMARY.md](BUILD_SUMMARY.md)** - Build status and project overview
3. **[README_BUILD.md](README_BUILD.md)** - Detailed build and usage instructions
4. **[VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md)** - Guide for visualizing results

## 🚀 Quick Start

### First Time Setup
```bash
cd /home/ambushee/fibo_mobilerobot
colcon build --symlink-install
source install/setup.bash
```

### Run the Lab Parts

**Part 1 - EKF Odometry:**
```bash
# Terminal 1
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/

# Terminal 2
ros2 launch fra532_lab1_part1 odometry.launch.py
```

**Part 2 - ICP Odometry:**
```bash
# Terminal 1
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/

# Terminal 2
ros2 launch fra532_lab1_part2 icp_odometry.launch.py
```

**Part 3 - SLAM:**
```bash
# Terminal 1
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/

# Terminal 2
ros2 launch fra532_lab1_part3 slam_toolbox.launch.py
```

## 📦 Packages

### Part 1: fra532_lab1_part1
- **Purpose**: EKF-based sensor fusion
- **Nodes**:
  - `wheel_odometry` - Raw wheel encoder odometry
  - `ekf_odometry` - EKF fusion of wheel + IMU
- **Topics**:
  - Subscribes: `/joint_states`, `/imu`
  - Publishes: `/wheel_odom`, `/ekf_odom`

### Part 2: fra532_lab1_part2
- **Purpose**: LiDAR-based odometry refinement
- **Node**: `icp_odometry`
- **Topics**:
  - Subscribes: `/scan`, `/ekf_odom`
  - Publishes: `/icp_odom`

### Part 3: fra532_lab1_part3
- **Purpose**: Full SLAM with loop closure
- **Launch**: `slam_toolbox.launch.py`
- **Config**: `slam_toolbox_params.yaml`

## 📊 Data

### Dataset Sequences
- **Sequence 00**: Empty hallway (baseline)
- **Sequence 01**: Sharp turns with obstacles
- **Sequence 02**: Smooth motion with obstacles

### Available Topics (from bag files)
- `/joint_states` - Wheel encoder data (20 Hz)
- `/imu` - IMU data (20 Hz)
- `/scan` - LiDAR scans (5 Hz)

## 🛠️ Utility Scripts

Located in `scripts/` directory:

1. **test_installation.sh** - Verify all packages are built correctly
   ```bash
   ./scripts/test_installation.sh
   ```

2. **quick_reference.sh** - Display command reference
   ```bash
   ./scripts/quick_reference.sh
   ```

3. **plot_trajectories.py** - Plot recorded trajectories
   ```bash
   python3 scripts/plot_trajectories.py <bag_file.db3>
   ```

## 📈 Deliverables Checklist

- [ ] Source code (✅ Complete)
- [ ] README (✅ Complete)
- [ ] Trajectory plots:
  - [ ] Wheel odometry
  - [ ] EKF odometry
  - [ ] ICP odometry
  - [ ] SLAM pose output
- [ ] Generated 2D maps:
  - [ ] ICP odometry map
  - [ ] slam_toolbox map
- [ ] Analysis report:
  - [ ] Accuracy comparison
  - [ ] Drift analysis
  - [ ] Robustness discussion

## 🎯 Learning Outcomes Addressed

✅ Implement EKF-based sensor fusion for mobile robots
✅ Apply ICP for LiDAR-based odometry refinement
✅ Understand the role of loop closure in SLAM
✅ Critically evaluate different localization approaches

## 📝 Implementation Details

### Robot Configuration
- Platform: Turtlebot3 Burger
- Wheel radius: 0.033 m
- Wheel separation: 0.160 m

### Algorithms
- **EKF**: 5-state filter [x, y, θ, v, ω]
- **ICP**: Iterative Closest Point scan matching
- **SLAM**: Graph-based SLAM with loop closure

### Coordinate Frames
- `map` - World frame (for SLAM)
- `odom` - Odometry frame
- `base_link` - Robot frame

## 🧪 Testing Workflow

1. **Verify Installation**
   ```bash
   ./scripts/test_installation.sh
   ```

2. **Test Each Part Individually**
   - Part 1: Run and verify `/wheel_odom` and `/ekf_odom`
   - Part 2: Run and verify `/icp_odom`
   - Part 3: Run and verify map generation

3. **Record Data**
   ```bash
   ros2 bag record /wheel_odom /ekf_odom /icp_odom -o results
   ```

4. **Visualize**
   ```bash
   rviz2
   python3 scripts/plot_trajectories.py results_0.db3
   ```

5. **Analyze Results**
   - Compare trajectories
   - Calculate drift
   - Evaluate map quality

## 📚 Additional Resources

### ROS2 Documentation
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [slam_toolbox GitHub](https://github.com/SteveMacenski/slam_toolbox)

### Reference Materials
- Original lab instructions: [README.md](README.md)
- Build guide: [README_BUILD.md](README_BUILD.md)
- Visualization guide: [VISUALIZATION_GUIDE.md](VISUALIZATION_GUIDE.md)

## 🐛 Troubleshooting

### Common Issues

**Build fails:**
```bash
rm -rf build install log
colcon build --symlink-install
```

**No topics visible:**
```bash
ros2 topic list
ros2 topic hz /scan
```

**TF warnings:**
- Check bag is playing
- Verify node is running
- Check frame names

## 📞 Support

- **Maintainer**: Ambushee <athit.jake@gmail.com>
- **Repository**: [GitHub](https://github.com/BoltonAthitDavies/fibo_mobilerobot)

## ✅ Project Status

**Build Status**: ✅ SUCCESS
**All Tests**: ✅ PASSED
**Ready for**: 🚀 EXPERIMENTS

---

**Last Updated**: February 2, 2026
**ROS Distribution**: ROS2 Humble/Iron
**Build System**: colcon
**Language**: Python 3
