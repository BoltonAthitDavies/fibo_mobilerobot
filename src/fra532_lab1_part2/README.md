# FRA532 Lab 1 Part 2 - ICP Odometry Refinement

This package provides a ROS2 implementation for ICP-based odometry refinement using EKF odometry as an initial guess.

## Objective

Refine the EKF-based odometry from Part 1 using LiDAR scan matching to improve accuracy and reduce drift.

## Approach

1. **EKF Initial Guess**: Uses EKF odometry from Part 1 as the initial transformation estimate
2. **Consecutive Scan Matching**: Performs ICP between consecutive `/scan` messages
3. **Relative Pose Integration**: Integrates relative transformations to produce refined odometry
4. **Drift Reduction**: Combines wheel odometry, IMU, and LiDAR data for improved accuracy

```
fra532_lab1_part2/
├── fra532_lab1_part2/
│   ├── __init__.py
│   ├── icp_localization.py    # Main ICP localization node
│   └── scan_matcher.py        # Supplementary scan matching node
├── launch/
│   ├── icp_localization.launch.py     # Basic ICP launch
│   └── icp_with_rviz.launch.py       # ICP with RViz visualization
├── config/
│   └── icp_params.yaml        # Configuration parameters
├── setup.py                   # Package setup
├── package.xml               # Package dependencies
└── README.md                 # This file
```

## Nodes

### 1. ICP Odometry Refinement Node (`icp_localization.py`)
- **Main functionality**: Refines EKF odometry using consecutive LiDAR scan matching
- **Subscriptions**:
  - `/scan` (LaserScan): LiDAR scanner data for consecutive matching
  - `/odom` (Odometry): Original wheel odometry for reference 
  - `/ekf_odom` (Odometry): EKF odometry from Part 1 (initial guess)
- **Publications**:
  - `/icp_pose` (PoseWithCovarianceStamped): ICP-refined robot pose
  - `/icp_refined_odom` (Odometry): ICP-refined odometry with reduced drift
  - TF transforms: `map` → `base_footprint`

### 2. Scan Matcher Node (`scan_matcher.py`) - Optional
- **Functionality**: Additional scan-to-scan matching utilities
- **Subscriptions**: `/scan` (LaserScan)
- **Publications**: `/cmd_vel_corrected` (Twist)

## Implementation Tasks

You need to implement the following methods in `icp_localization.py`:

### 1. `run_icp(previous_points, current_points, initial_guess)`
Consecutive scan ICP implementation:
- Use EKF odometry as initial transformation guess
- Iterate between correspondence finding and transformation estimation
- Apply convergence criteria for early termination
- Return relative transformation between consecutive scans

### 2. `find_correspondences(previous_points, current_points)`
Find point correspondences between consecutive scans:
- For each point in current scan, find closest point in previous scan
- Apply distance threshold to reject outliers
- Consider using spatial data structures (KD-tree) for efficiency
- Return correspondence pairs for transformation estimation

### 3. `estimate_transformation(correspondences, previous_points, current_points)`
Estimate relative transformation from correspondences:
- Implement least squares solution for [dx, dy, dtheta]
- Use SVD for robust rotation estimation
- Handle degenerate cases (insufficient correspondences)
- Return relative transformation between scans

## Building and Running

### Build the package:
```bash
cd /home/ambushee/fibo_mobilerobot
colcon build --packages-select fra532_lab1_part2
source install/setup.bash
```

### Run ICP localization:
```bash
# Basic ICP node
ros2 launch fra532_lab1_part2 icp_localization.launch.py

# ICP with RViz visualization
ros2 launch fra532_lab1_part2 icp_with_rviz.launch.py
```

### Monitor topics:
```bash
# View refined odometry output
ros2 topic echo /icp_refined_odom

# Compare with EKF odometry
ros2 topic echo /ekf_odom

# View ICP pose estimates
ros2 topic echo /icp_pose

# Check TF tree
ros2 run tf2_tools view_frames
```

## Expected Improvements

The ICP odometry refinement should provide:
1. **Reduced Drift**: LiDAR-based correction reduces cumulative odometry errors
2. **Better Accuracy**: Consecutive scan matching provides precise relative motion
3. **Robust Estimation**: Combination of EKF initial guess with LiDAR validation
4. **Real-time Performance**: Efficient scan-to-scan matching for online operation

## Configuration

Edit `config/icp_params.yaml` to tune ICP parameters:
- `max_iterations`: Maximum ICP iterations (20-100)
- `convergence_threshold`: Convergence criteria (0.001-0.01)
- `max_correspondence_distance`: Maximum distance for correspondences (0.5-2.0)
- `use_ekf_initial_guess`: Enable EKF odometry as initial guess
- `min_correspondences`: Minimum correspondences required for valid ICP

## Algorithm Tips

1. **Initial Guess Utilization**:
   - Transform current scan using EKF motion estimate before ICP
   - Weight initial guess based on EKF covariance
   - Fall back to zero motion if EKF unavailable

2. **Correspondence Finding**:
   - Use KD-tree for efficient nearest neighbor search
   - Apply distance thresholds to reject outliers
   - Consider point normals for better matching

3. **Transformation Estimation**:
   - Implement SVD-based least squares solution
   - Use Point-to-Point ICP for simplicity
   - Consider Point-to-Line ICP for better convergence

4. **Integration Strategy**:
   - Transform relative motions to global coordinates
   - Maintain running pose estimate
   - Update covariance based on ICP quality

5. **Performance Optimization**:
   - Downsample point clouds for speed
   - Use adaptive correspondence thresholds
   - Implement early termination criteria

## Testing

Test your implementation with:
1. **Bag Files**: Record and playback laser scan + EKF odometry data
2. **Gazebo Simulation**: Test in controlled environment with known ground truth
3. **Real Robot**: Validate on actual robot with various motion patterns

Monitor performance by checking:
- **Drift Reduction**: Compare trajectories with/without ICP refinement
- **Convergence**: Ensure ICP consistently converges within iteration limits
- **Accuracy**: Validate against ground truth or SLAM-based estimates
- **Computational Load**: Monitor processing time per scan

## Data Flow

```
LiDAR (/scan) ──┐
                ├──► ICP Refinement ──► /icp_refined_odom
EKF Odometry ───┘    (consecutive       (reduced drift)
(/ekf_odom)          scan matching)
```