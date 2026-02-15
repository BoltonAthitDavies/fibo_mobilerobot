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
|   ├── fibo_floor3_seq00/                
|   ├── fibo_floor3_seq01/                
|   ├── fibo_floor3_seq02/                
├── data/                           # Lab 1 csv data (3 sequences)
|   ├── rosbag_seq0/                # Extracted CSV data (Sequence 0)
|   ├── rosbag_seq1/                # Extracted CSV data (Sequence 1)
|   ├── rosbag_seq2/                # Extracted CSV data (Sequence 2)
|   ├── seq0_v3_output/             # Odometry output data (Sequence 0)
|   ├── seq1_v3_output/             # Odometry output data (Sequence 1)
|   └── seq2_v3_output/             # Odometry output data (Sequence 2)
├── config/                         # RViz visualization configs
├── plot.ipynb                      # Jupyter notebook for trajectory plotting
└── 6629_README.md                       # Main project README
```

---

## How to Generate Complete Deliverables

### Step 1: Build All Packages
```bash
colcon build
source install/setup.bash
```

All three packages (Part 1, 2, 3) will be built with a single command.

### Step 2: Install python packages
```bash
python3 -m pip install -r requirement.txt
```

This pre-extracts CSV data for analysis (optional; data can also be logged during runtime)

### Step 3: Run Parts to Generate Odometry

**For Part 1 (EKF):**
```bash
# Terminal 1: Start EKF with rviz
ros2 launch fra532_lab1_part1 wheel_odom_with_rviz.launch.py use_sim_time:=true
# or Terminal 1: Start EKF without rviz
ros2 launch fra532_lab1_part1 ekf_odometry.launch.py use_sim_time:=true

# Terminal 2 Generate Plots
python3 ./src/plot.py

# Terminal 3: Play bag file
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ \
    --clock \
    --qos-profile-overrides-path qos_overrides.yaml \
    --rate 1.0
```

**For Part 2 (ICP):**
```bash
# Terminal 1: Start EKF without rviz
ros2 launch fra532_lab1_part1 ekf_odometry.launch.py use_sim_time:=true

# Terminal 2: Start ICP with mapping with rviz
ros2 launch fra532_lab1_part2 icp_with_mapping.launch.py use_sim_time:=true
# or Terminal 2: Start ICP with mapping without rviz
ros2 launch fra532_lab1_part2 icp_with_rviz.launch.py use_sim_time:=true

# Terminal 3 Generate Plots
python3 ./src/plot.py

# Terminal 4: Play bag file
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ \
    --clock \
    --qos-profile-overrides-path qos_overrides.yaml \
    --rate 1.0
```

**For Part 3 (SLAM):**
```bash
# Terminal 1: Start SLAM with rviz
ros2 launch fra532_lab1_part3 slam_with_rviz.launch.py use_sim_time:=true
# or Terminal 1: Start SLAM without rviz
ros2 launch fra532_lab1_part3 slam.launch.py use_sim_time:=true

# Terminal 2 Generate Plots
python3 ./src/plot.py

# Terminal 3: Play bag file
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ \
    --clock \
    --qos-profile-overrides-path qos_overrides.yaml \
    --rate 1.0
```

This generates trajectory comparison plots showing all odometry methods overlaid.

---

## Part 1: EKF Odometry Fusion

### Deliverables
- **Source Code:** `src/fra532_lab1_part1/fra532_lab1_part1/ekf_odometry.py`
- **Launch File:** `src/fra532_lab1_part1/launch/ekf_odometry.launch.py`

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
timestamp, x, y, theta (radians)
```

### Key Features
- ✅ Wheel odometry computation from joint states (20 Hz)
- ✅ IMU angular velocity corrections (asynchronous)
- ✅ Covariance tracking for uncertainty quantification
- ✅ TF frame broadcasting for visualization

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

## File Summary

### Configuration Files (Tunable Parameters)
| Path | Purpose |
|------|---------|
| `src/fra532_lab1_part1/config/ekf_params.yaml` | EKF noise covariances (Q, R) |
| `src/fra532_lab1_part2/config/icp_params.yaml` | ICP iterations, convergence threshold |
| `qos_overrides.yaml` | ROS2 QoS settings (reliability, history) |

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

## References & Resources

- **ROS2 Documentation:** https://docs.ros.org/en-foxy/
- **slam_toolbox:** https://github.com/StanleyInnovation/slam_toolbox
- **TurtleBot3 Specs:** https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/
- **EKF Theory:** Probabilistic Robotics (Thrun, Burgard, Fox)
- **ICP Algorithm:** "A Method for Registration of 3-D Shapes" (Besl & McKay, 1992)

---

**Last Updated:** February 15, 2026  