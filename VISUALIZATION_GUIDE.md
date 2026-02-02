# Visualization Guide

This guide shows how to visualize the odometry and SLAM results.

## Using RViz2

### Basic Setup

1. Start RViz:
```bash
rviz2
```

2. Configure the following:
   - Fixed Frame: `odom` (or `map` for SLAM)
   - Add displays as shown below

### Display Configuration

#### For Part 1 & 2 (Odometry)

Add these displays:

1. **TF Display**
   - Shows the coordinate frames
   - Frames: `odom`, `base_link`

2. **Odometry Displays** (add multiple)
   - Display Type: `Odometry`
   - Topics:
     - `/wheel_odom` - Color: Red
     - `/ekf_odom` - Color: Blue
     - `/icp_odom` - Color: Green
   - Settings:
     - Shape: Arrow
     - Keep: 1000 (to show trajectory)

3. **LaserScan**
   - Topic: `/scan`
   - Size: 0.05
   - Style: Points

#### For Part 3 (SLAM)

Add these displays:

1. **Map**
   - Display Type: `Map`
   - Topic: `/map`
   - Color Scheme: map

2. **LaserScan**
   - Topic: `/scan`
   - Size: 0.05

3. **TF Display**
   - Frames: `map`, `odom`, `base_link`

### Save RViz Configuration

After configuring, save your RViz setup:
1. File → Save Config As
2. Save to: `~/fibo_mobilerobot/config/lab1.rviz`

## Recording Data for Analysis

### Record Odometry Topics

```bash
# Record all odometry topics
ros2 bag record /wheel_odom /ekf_odom /icp_odom -o results/odometry_comparison

# Record with SLAM
ros2 bag record /wheel_odom /ekf_odom /icp_odom /map /pose -o results/slam_comparison
```

### Record Specific Duration

```bash
# Record for 5 minutes
ros2 bag record /wheel_odom /ekf_odom /icp_odom --duration 300 -o results/seq00
```

## Plotting Trajectories

### Using the Provided Script

```bash
# After recording data
python3 scripts/plot_trajectories.py results/odometry_comparison_0.db3
```

### Using Python (Manual)

```python
import sqlite3
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Load bag file
conn = sqlite3.connect('results/odometry_comparison_0.db3')
cursor = conn.cursor()

# Extract trajectories for each topic
# ... (see plot_trajectories.py for details)

# Plot
plt.figure(figsize=(12, 10))
plt.plot(x_wheel, y_wheel, 'r-', label='Wheel Odometry')
plt.plot(x_ekf, y_ekf, 'b-', label='EKF Odometry')
plt.plot(x_icp, y_icp, 'g-', label='ICP Odometry')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.savefig('trajectory_comparison.png')
plt.show()
```

## Real-time Topic Monitoring

### Echo Odometry Data

```bash
# Wheel odometry
ros2 topic echo /wheel_odom

# EKF odometry
ros2 topic echo /ekf_odom

# ICP odometry
ros2 topic echo /icp_odom
```

### Monitor Topic Frequency

```bash
# Check publishing rate
ros2 topic hz /ekf_odom

# Check all rates
ros2 topic hz /wheel_odom & \
ros2 topic hz /ekf_odom & \
ros2 topic hz /icp_odom
```

### Monitor Topic Bandwidth

```bash
ros2 topic bw /scan
ros2 topic bw /ekf_odom
```

## Using rqt Tools

### rqt_plot - Real-time Plotting

```bash
rqt_plot
```

Then add topics:
- `/wheel_odom/pose/pose/position/x`
- `/wheel_odom/pose/pose/position/y`
- `/ekf_odom/pose/pose/position/x`
- `/ekf_odom/pose/pose/position/y`

### rqt_graph - Node Graph

```bash
rqt_graph
```

This shows:
- Active nodes
- Topic connections
- Data flow

### rqt_bag - Bag File Viewer

```bash
rqt_bag results/odometry_comparison_0.db3
```

Features:
- Visualize recorded data
- Play/pause/step through data
- Export specific sections

## Analyzing Map Quality (Part 3)

### Save Map from SLAM

```bash
# Run this while SLAM is active
ros2 run nav2_map_server map_saver_cli -f my_map
```

This creates:
- `my_map.pgm` - The map image
- `my_map.yaml` - Map metadata

### View Map

```bash
# View the map image
eog my_map.pgm
# or
gimp my_map.pgm
```

### Compare Maps

Save maps from different sequences and compare:
```bash
ros2 run nav2_map_server map_saver_cli -f seq00_map
ros2 run nav2_map_server map_saver_cli -f seq01_map
ros2 run nav2_map_server map_saver_cli -f seq02_map
```

## Performance Metrics

### Calculate Drift

```python
# Compare start and end positions
# Expected: (0, 0) if the robot returns to start

# Wheel odometry drift
wheel_drift = sqrt((x_end - x_start)**2 + (y_end - y_start)**2)

# EKF odometry drift
ekf_drift = sqrt((x_end - x_start)**2 + (y_end - y_start)**2)

# ICP odometry drift
icp_drift = sqrt((x_end - x_start)**2 + (y_end - y_start)**2)
```

### Calculate Path Length

```python
path_length = sum(sqrt((x[i+1]-x[i])**2 + (y[i+1]-y[i])**2) 
                  for i in range(len(x)-1))
```

### Calculate Average Error

If ground truth is available:
```python
rmse = sqrt(mean((x_estimated - x_true)**2 + (y_estimated - y_true)**2))
```

## Visualization Best Practices

1. **Use consistent colors**:
   - Red: Wheel odometry
   - Blue: EKF odometry
   - Green: ICP odometry
   - Yellow: SLAM pose

2. **Set appropriate scales**:
   - Use `axis('equal')` for trajectory plots
   - Adjust plot limits to show full trajectory

3. **Add grid and labels**:
   - Clear axis labels with units
   - Legend with meaningful names
   - Grid for easier reading

4. **Save high-resolution images**:
   ```python
   plt.savefig('plot.png', dpi=300, bbox_inches='tight')
   ```

## Troubleshooting Visualization

### No data in RViz
- Check topics are being published: `ros2 topic list`
- Verify correct Fixed Frame
- Check TF tree: `ros2 run tf2_tools view_frames`

### TF warnings
- Ensure bag is playing: `ros2 bag play <bag_file>`
- Check node is running: `ros2 node list`
- Verify topic names match

### Plot script errors
- Install required packages: `pip3 install matplotlib numpy`
- Check bag file path is correct
- Ensure bag file has the required topics

## Example Workflow

```bash
# Terminal 1: Play bag
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/

# Terminal 2: Run nodes
ros2 launch fra532_lab1_part1 odometry.launch.py

# Terminal 3: Visualize
rviz2

# Terminal 4: Record results
ros2 bag record /wheel_odom /ekf_odom -o results/seq00

# After recording: Plot
python3 scripts/plot_trajectories.py results/seq00_0.db3
```

## Additional Resources

- [RViz User Guide](http://wiki.ros.org/rviz)
- [rqt Documentation](http://wiki.ros.org/rqt)
- [ROS2 Visualization](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
