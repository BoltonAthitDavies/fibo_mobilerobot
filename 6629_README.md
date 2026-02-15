# FRA532 LAB1 - Deliverables Guide

## Overview
This document describes all deliverables for the FRA532 LAB1: Kalman Filter / SLAM project. The lab is divided into three progressive parts, each building upon previous results to evaluate different odometry and SLAM approaches.

## Deliverables Location
The following deliverables are compiled in [6629_Discussion.pdf](6629_Discussion.pdf):
- Plots of trajectories from wheel odometry, EKF odometry, ICP odometry, and SLAM pose output.
- Generated 2D maps from ICP odometry and slam_toolbox.
- Discussion comparing accuracy, drift, and robustness of each method.

---

## Project Structure

```
fibo_mobilerobot/
├── src/
│   ├── fra532_lab1_part1/          # EKF Odometry Fusion
│   ├── fra532_lab1_part2/          # ICP Odometry Refinement
│   └── fra532_lab1_part3/          # Full SLAM with slam_toolbox
├── FRA532_LAB1_DATASET/            # ROS bag files (3 sequences)
├── rosbag_seq0/                    # Extracted CSV data (Sequence 0)
├── rosbag_seq1/                    # Extracted CSV data (Sequence 1)
├── rosbag_seq2/                    # Extracted CSV data (Sequence 2)
├── config/                         # RViz visualization configs
├── plot.ipynb                      # Jupyter notebook for trajectory plotting
├── analyze_trajectories.py         # Analysis and comparison script
└── README.md                       # Main project README
```

---

## Part 1: EKF Odometry Fusion

### Deliverables
- **Source Code:** `src/fra532_lab1_part1/fra532_lab1_part1/ekf_odometry.py`
- **Launch File:** `src/fra532_lab1_part1/launch/ekf_odometry.launch.py`
- **README:** `src/fra532_lab1_part1/README.md`

### Key Implementation Files
| File | Purpose |
|------|---------|
| `ekf_odometry.py` | Main EKF node fusing wheel + IMU data |
| `simple_wheel_odometry.py` | Reference wheel-only odometry (baseline) |
| `trajectory_logger.py` | Data logging utility for analysis |
| `ekf_params.yaml` | Tunable EKF parameters (Q, R matrices) |

### Output Data
**ROS Topic:** `/ekf_odom` (nav_msgs/Odometry at ~20 Hz)

**Logged Files (when running with logging):**
- `odom_ekf_data.csv` - Full EKF trajectory (all sequences)
- `odom_ekf_data_seq0.csv` - EKF trajectory (Sequence 0)
- `odom_ekf_data_seq1.csv` - EKF trajectory (Sequence 1)
- `odom_ekf_data_seq2.csv` - EKF trajectory (Sequence 2)

**CSV Columns:**
```
timestamp, x, y, theta (radians), vx, vy, angular_velocity
```

### Key Features
- ✅ Wheel odometry computation from joint states (20 Hz)
- ✅ IMU angular velocity corrections (asynchronous)
- ✅ Covariance tracking for uncertainty quantification
- ✅ TF frame broadcasting for visualization

### How to Run
```bash
# Terminal 1: Start wheel odometry and EKF
ros2 launch fra532_lab1_part1 wheel_odom_with_rviz.launch.py use_sim_time:=true
ros2 launch fra532_lab1_part1 ekf_odometry.launch.py use_sim_time:=true

# Terminal 2: Play bag file with clock and QoS overrides
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ \
    --clock \
    --qos-profile-overrides-path qos_overrides.yaml \
    --rate 1.0
```

**Available Dataset Sequences:**
- `fibo_floor3_seq00` - Empty hallway
- `fibo_floor3_seq01` - Hallway with obstacles & sharp turns
- `fibo_floor3_seq02` - Hallway with obstacles & smooth motion

---

## Part 2: ICP Odometry Refinement

### Deliverables
- **Source Code:** `src/fra532_lab1_part2/fra532_lab1_part2/icp_odometry.py`
- **Mapper Node:** `src/fra532_lab1_part2/fra532_lab1_part2/icp_mapper.py`
- **Launch File:** `src/fra532_lab1_part2/launch/icp_odometry.launch.py`
- **README:** `src/fra532_lab1_part2/README.md`

### Key Implementation Files
| File | Purpose |
|------|---------|
| `icp_odometry.py` | Main ICP scan matching node (5 Hz synchronized) |
| `icp_mapper.py` | Occupancy grid mapping from ICP odometry |
| `icp_params.yaml` | ICP tunable parameters (max iterations, convergence) |

### Output Data
**ROS Topics:**
- `/icp_odom` (nav_msgs/Odometry at 5 Hz)
- `/icp_map` (nav_msgs/OccupancyGrid at 1 Hz)

**Logged Files:**
- `odom_icp_data.csv` - Full ICP trajectory
- `odom_icp_data_seq0.csv` - ICP trajectory (Sequence 0)
- `odom_icp_data_seq1.csv` - ICP trajectory (Sequence 1)
- `odom_icp_data_seq2.csv` - ICP trajectory (Sequence 2)

**CSV Columns:**
```
timestamp, x, y, theta (radians), correction_magnitude
```

### Key Features
- ✅ Point-to-point ICP with keyframe buffer (fallback)
- ✅ Point-to-map ICP using occupancy grids (primary)
- ✅ **5 Hz synchronized processing** (rate-limited for determinism)
- ✅ Occupancy grid mapping from LiDAR + odometry
- ✅ RViz visualization of 2D map

### Architecture: 5 Hz Rate Limiting
```
Laser scans (5 Hz)  ─┐
                      ├─→ [Buffer latest] ─→ [5 Hz Timer (0.2s)]
EKF updates (20 Hz)─┐                        └─→ ICP Process Step
                                                  ├─ Run ICP matching
                                                  ├─ Update pose
                                                  └─ Publish /icp_odom
```

### How to Run
```bash
# Terminal 1: Start ICP with mapping and visualization
ros2 launch fra532_lab1_part2 icp_with_rviz.launch.py use_sim_time:=true
ros2 launch fra532_lab1_part2 icp_with_mapping.launch.py use_sim_time:=true

# Terminal 2: Play bag file with clock and QoS overrides
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ \
    --clock \
    --qos-profile-overrides-path qos_overrides.yaml \
    --rate 1.0
```

**Available Dataset Sequences:**
- `fibo_floor3_seq00` - Empty hallway
- `fibo_floor3_seq01` - Hallway with obstacles & sharp turns
- `fibo_floor3_seq02` - Hallway with obstacles & smooth motion

### Map Visualization
- **Topic:** `/icp_map` (OccupancyGrid)
- **Resolution:** 0.05 m/cell
- **Occupancy Values:** 0 = free, 50+ = occupied, -1 = unknown
- **RViz Layer:** "Map" with turbo colormap

---

## Part 3: Full SLAM with slam_toolbox

### Deliverables
- **Launch File:** `src/fra532_lab1_part3/launch/slam.launch.py`
- **Configuration:** ROS2 SLAM Toolbox parameters
- **README:** `src/fra532_lab1_part3/README.md`

### Output Data
**ROS Topics:**
- `/map` (nav_msgs/OccupancyGrid)
- `/slam_toolbox/graph_nodes` (visualization_msgs/MarkerArray)
- `/tf` (pose estimates)

**Logged Files:**
- `odom_slam_(truth)_data_seq*.csv` - SLAM pose trajectory
- SLAM serialized maps (graph files)

### Key Features
- ✅ Loop closure detection
- ✅ Graph optimization (Ceres solver)
- ✅ Dynamic vs static SLAM modes
- ✅ Pose graph visualization

### How to Run
```bash
# Terminal 1: Start SLAM with visualization
ros2 launch fra532_lab1_part3 slam_with_rviz.launch.py use_sim_time:=true
ros2 launch fra532_lab1_part3 slam.launch.py use_sim_time:=true

# Terminal 2 (optional): Start SLAM evaluator
ros2 run fra532_lab1_part3 slam_evaluator

# Terminal 3: Play bag file with clock and QoS overrides
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ \
    --clock \
    --qos-profile-overrides-path qos_overrides.yaml \
    --rate 1.0
```

**Available Dataset Sequences:**
- `fibo_floor3_seq00` - Empty hallway
- `fibo_floor3_seq01` - Hallway with obstacles & sharp turns
- `fibo_floor3_seq02` - Hallway with obstacles & smooth motion

---

## Data Analysis & Visualization

### CSV Data Files
All odometry data is exported to CSV format for analysis. Files are organized by sequence:

```
rosbag_seq0/
├── odom_ekf_data_seq0.csv
├── odom_icp_data_seq0.csv
└── odom_slam_(truth)_data_seq0.csv

rosbag_seq1/
├── odom_ekf_data_seq1.csv
├── odom_icp_data_seq1.csv
└── odom_slam_(truth)_data_seq1.csv

rosbag_seq2/
├── odom_ekf_data_seq2.csv
├── odom_icp_data_seq2.csv
└── odom_slam_(truth)_data_seq2.csv
```

### Data Extraction
```bash
# Extract csv from existing rosbag
python3 ros2bag_to_csv.py FRA532_LAB1_DATASET/fibo_floor3_seq00 rosbag_seq0

# This generates CSV files in rosbag_seq0/ directory
```

### Trajectory Plotting
```bash
# Generate trajectory comparison plots
python3 ./src/plot.py

# Output: PNG plots saved to current directory
```

### Jupyter Notebook: plot.ipynb
The notebook provides comprehensive trajectory visualization and analysis:
- **Plot 1:** XY trajectory comparison (all methods overlaid)
- **Plot 2:** X position vs time
- **Plot 3:** Y position vs time
- **Plot 4:** Heading angle vs time
- **Plot 5:** Cumulative error / drift analysis
- **Plot 6:** Method comparison metrics

**Key Metrics Computed:**
- Trajectory length (total distance traveled)
- Final position error (if ground truth available)
- Drift rate (error/distance)
- Heading consistency

---

## Visualization Configurations

### RViz Config Files
Located in `config/`:

| Config File | Purpose | Topics |
|------------|---------|--------|
| `wheel_odom_viz.rviz` | Wheel odometry baseline | `/joint_states`, `/wheel_odom` |
| `ekf_odom_viz_clean.rviz` | EKF visualization | `/ekf_odom`, TF frames |
| `icp_odom_viz.rviz` | ICP + mapping | `/icp_odom`, `/icp_map` |
| `slam_viz.rviz` | Full SLAM | `/map`, `/slam_toolbox/*` |
| `lab1_slam.rviz` | Comparison view | All odometry + maps |

### Launching RViz
```bash
rviz2 -d config/icp_odom_viz.rviz
```

---

## Expected Results & Metrics

### Sequence 0: Empty Hallway
- **Distance traveled:** ~20 m
- **Turning:** Minimal
- **EKF drift:** ~0.5-1.0 m over full trajectory
- **ICP correction:** Minimal (low feature matching opportunities)
- **Best performer:** Wheel odometry (simplest environment)

### Sequence 1: Hallway with Obstacles & Sharp Turns  
- **Distance traveled:** ~25 m
- **Turning:** Multiple sharp 90° turns
- **EKF drift:** ~1-2 m (challenges with rapid heading changes)
- **ICP refinement:** Significant (features improve scan matching)
- **Best performer:** SLAM (loop closure handles sharp turns)

### Sequence 2: Hallway with Obstacles & Smooth Motion
- **Distance traveled:** ~15 m
- **Turning:** Gentle curves
- **EKF drift:** ~0.3-0.5 m (smooth motion aids filtering)
- **ICP refinement:** Moderate (good feature quality)
- **Best performer:** ICP or SLAM (smooth motion = stable matching)

### Error Metrics
Each method should be evaluated on:
- **ATE (Absolute Trajectory Error):** RMS position error vs ground truth
- **RPE (Relative Pose Error):** Consistency between consecutive poses
- **Drift Rate:** Error accumulation per meter traveled
- **Computation Time:** CPU/memory usage during processing

---

## How to Generate Complete Deliverables

### Step 1: Build All Packages
```bash
colcon build
source install/setup.bash
```

All three packages (Part 1, 2, 3) will be built with a single command.

### Step 2: Extract Data from Bags (Optional)
```bash
python3 ros2bag_to_csv.py FRA532_LAB1_DATASET/fibo_floor3_seq00 rosbag_seq0
python3 ros2bag_to_csv.py FRA532_LAB1_DATASET/fibo_floor3_seq01 rosbag_seq1
python3 ros2bag_to_csv.py FRA532_LAB1_DATASET/fibo_floor3_seq02 rosbag_seq2
```

This pre-extracts CSV data for analysis (optional; data can also be logged during runtime)

### Step 3: Run Parts to Generate Odometry

**For Part 1 (EKF):**
```bash
# Terminal 1: Start EKF
ros2 launch fra532_lab1_part1 wheel_odom_with_rviz.launch.py use_sim_time:=true
ros2 launch fra532_lab1_part1 ekf_odometry.launch.py use_sim_time:=true

# Terminal 2: Play bag file
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ \
    --clock \
    --qos-profile-overrides-path qos_overrides.yaml \
    --rate 1.0
```

**For Part 2 (ICP):**
```bash
# Terminal 1: Start ICP with mapping
ros2 launch fra532_lab1_part2 icp_with_rviz.launch.py use_sim_time:=true
ros2 launch fra532_lab1_part2 icp_with_mapping.launch.py use_sim_time:=true

# Terminal 2: Play bag file
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ \
    --clock \
    --qos-profile-overrides-path qos_overrides.yaml \
    --rate 1.0
```

**For Part 3 (SLAM):**
```bash
# Terminal 1: Start SLAM
ros2 launch fra532_lab1_part3 slam_with_rviz.launch.py use_sim_time:=true
ros2 launch fra532_lab1_part3 slam.launch.py use_sim_time:=true

# Terminal 2 (optional): Start evaluator
ros2 run fra532_lab1_part3 slam_evaluator

# Terminal 3: Play bag file
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ \
    --clock \
    --qos-profile-overrides-path qos_overrides.yaml \
    --rate 1.0
```

### Step 4: Generate Plots
```bash
python3 ./src/plot.py
```

This generates trajectory comparison plots showing all odometry methods overlaid.

### Step 5: Analyze Results
- Compare drift across methods
- Discuss strengths/weaknesses
- Document tuning decisions (Q, R matrices, ICP parameters)

---

## File Summary

### Configuration Files (Tunable Parameters)
| Path | Purpose |
|------|---------|
| `src/fra532_lab1_part1/config/ekf_params.yaml` | EKF noise covariances (Q, R) |
| `src/fra532_lab1_part2/config/icp_params.yaml` | ICP iterations, convergence threshold |
| `qos_overrides.yaml` | ROS2 QoS settings (reliability, history) |

### Generated Output Directories
| Directory | Contents |
|-----------|----------|
| `rosbag_seq{0,1,2}/` | Extracted CSV data from bags |
| `trajectory_comparison/` | PNG plots from analysis |
| `build/` | Colcon build artifacts |
| `install/` | Installed packages |
| `log/` | ROS2 build logs |

---

## Quality Checklist

Before submission, ensure:

- [ ] All three parts build without errors: `colcon build`
- [ ] Part 1 EKF produces `/ekf_odom` topic at ~20 Hz
- [ ] Part 2 ICP produces `/icp_odom` at 5 Hz and `/icp_map` at 1 Hz
- [ ] CSV data generated for all sequences
- [ ] Trajectory plots generated showing all methods
- [ ] Analysis includes error metrics and comparisons
- [ ] README files complete with parameter descriptions
- [ ] Code follows ROS2 conventions (package.xml, launch files)
- [ ] Git history shows development progression

---

## Submission Requirements

### Individual Work (Parts 1 & 2)
Students must submit:
1. Source code (`src/fra532_lab1_part1/` and `src/fra532_lab1_part2/`)
2. README with implementation notes
3. CSV data from all three sequences
4. Trajectory comparison plots
5. Analysis of accuracy/drift for each method
6. Parameter tuning documentation

### Group Work (Part 3)
Teams must submit:
1. Part 3 source code (`src/fra532_lab1_part3/`)
2. SLAM launch files and configuration
3. Generated maps from slam_toolbox
4. Complete trajectory comparison including SLAM
5. Group discussion of loop closure performance

### File Organization
```
submission/
├── src/fra532_lab1_part1/
├── src/fra532_lab1_part2/
├── src/fra532_lab1_part3/ (group)
├── rosbag_seq0/
├── rosbag_seq1/
├── rosbag_seq2/
├── trajectory_comparison/
├── plot.ipynb
├── analyze_trajectories.py
├── ANALYSIS.md (results & discussion)
└── README.md (overview)
```

---

## References & Resources

- **ROS2 Documentation:** https://docs.ros.org/en-foxy/
- **slam_toolbox:** https://github.com/StanleyInnovation/slam_toolbox
- **TurtleBot3 Specs:** https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
- **EKF Theory:** Probabilistic Robotics (Thrun, Burgard, Fox)
- **ICP Algorithm:** "A Method for Registration of 3-D Shapes" (Besl & McKay, 1992)

---

**Last Updated:** February 15, 2026  
**Lab Version:** FRA532 LAB1 v2.0
