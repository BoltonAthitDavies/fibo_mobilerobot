# Part 3: SLAM Implementation Guide

## Overview
Part 3 implements full SLAM using `slam_toolbox` for the FRA532 Lab 1. This implementation performs simultaneous localization and mapping using LiDAR data with loop closure detection.

## Package Structure
```
fra532_lab1_part3/
├── launch/
│   └── slam.launch.py          # Main SLAM launch file  
├── config/
│   └── slam_params.yaml        # SLAM toolbox parameters
├── fra532_lab1_part3/
│   ├── slam_evaluator.py       # Node for trajectory and map evaluation
│   └── map_saver.py           # Utility for saving maps
└── README.md                  # This file
```

## Dependencies
- `slam_toolbox`: Core SLAM implementation
- `tf2_ros`: Transform handling
- `nav_msgs`: Map and path messages
- `sensor_msgs`: Laser scan messages
- `matplotlib`: Plotting (for evaluation)
- `numpy`: Data processing

## Quick Start

### 1. Build the Package
```bash
colcon build --packages-select fra532_lab1_part3
source install/setup.bash
```

### 2. Run SLAM on a Sequence
Use the provided runner script:
```bash
# Run on sequence 1 (Empty Hallway)
./run_slam.sh 1

# Run on sequence 2 (Non-Empty with Sharp Turns) 
./run_slam.sh 2

# Run on sequence 3 (Non-Empty with Non-Aggressive Motion)
./run_slam.sh 3

# Run on all sequences
./run_slam.sh all
```

### 3. Manual Execution
If you prefer to run components manually:

```bash
# Terminal 1: Start SLAM
ros2 launch fra532_lab1_part3 slam.launch.py use_sim_time:=true

# Terminal 2: Start evaluator (optional)
ros2 run fra532_lab1_part3 slam_evaluator

# Terminal 3: Play bag file 
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/ \
    --clock \
    --qos-profile-overrides-path qos_overrides.yaml \
    --rate 1.0

# Terminal 4: Save map when done
ros2 run fra532_lab1_part3 map_saver my_slam_map
```

## Configuration

### SLAM Parameters (`config/slam_params.yaml`)
Key parameters you may want to adjust:

- `resolution`: Map resolution (default: 0.05m)
- `max_laser_range`: Maximum laser range (default: 20.0m)  
- `minimum_travel_distance`: Min distance to trigger new scan processing (default: 0.5m)
- `do_loop_closing`: Enable/disable loop closure (default: true)
- `loop_match_minimum_response_fine`: Loop closure threshold (default: 0.45)

### Transform Setup
The launch file sets up required transforms:
- `base_footprint -> base_link`: Robot base offset
- `base_link -> laser`: Laser sensor position

## Output Files

### Maps
- Location: `maps/` directory
- Format: `.pgm` (image) + `.yaml` (metadata)
- Example: `slam_map_20231210_143022.pgm` and `slam_map_20231210_143022.yaml`

### Trajectories  
- Location: `slam_results/` directory
- Format: `.txt` (data) + `.png` (plot)
- Contains: timestamp, x, y coordinates

## Evaluation

The `slam_evaluator` node provides:
- Real-time trajectory logging
- Automatic map saving at shutdown
- Trajectory visualization
- Performance metrics

## Troubleshooting

### Common Issues

1. **No map updates**: Check laser data and transform tree
```bash
ros2 topic echo /scan --once
ros2 run tf2_tools view_frames.py
```

2. **Poor loop closure**: Adjust `loop_match_minimum_response_fine` parameter

3. **High drift**: Tune `minimum_travel_distance` and scan matching parameters

4. **Memory issues**: Reduce `scan_buffer_size` or `max_laser_range`

### Debugging Commands
```bash
# Check available topics
ros2 topic list

# Monitor SLAM status  
ros2 topic echo /map_metadata --once

# View transform tree
ros2 run tf2_ros tf2_echo base_link laser

# Check SLAM node status
ros2 node info /slam_toolbox
```

## Results Analysis

Compare SLAM output with other methods:
- Trajectory accuracy vs. ICP odometry
- Map quality and completeness  
- Loop closure effectiveness
- Computational efficiency

The trajectory plots and map files generated can be used for quantitative comparison with Part 1 (EKF) and Part 2 (ICP) results.

## Advanced Usage

### Custom Parameters
Create your own parameter file based on `config/slam_params.yaml` and specify it in the launch:

```bash
ros2 launch fra532_lab1_part3 slam.launch.py \
    slam_params_file:=/path/to/your/params.yaml
```

### Integration with Navigation
The generated maps can be used with the Nav2 stack for autonomous navigation.