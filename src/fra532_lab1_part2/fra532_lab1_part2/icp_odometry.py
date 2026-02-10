#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import numpy as np
import math
from scipy.spatial import cKDTree

class ICPOdometry(Node):
    """
    Iterative Closest Point (ICP) odometry refinement
    Uses laser scan matching to correct drift in wheel/EKF odometry
    """
    
    def __init__(self):
        super().__init__('icp_odometry')
        
        # ICP parameters
        self.max_iterations = 20
        self.convergence_threshold = 1e-6
        self.max_correspondence_distance = 2.0
        self.min_scan_points = 50
        
        # Current state
        self.current_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.current_velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, omega]
        
        # Previous scan for matching
        self.previous_scan_points = None
        self.previous_time = None
        
        # Initial guess from EKF
        self.ekf_pose = None
        self.last_ekf_time = None
        
        # Publishers and subscribers
        self.icp_odom_pub = self.create_publisher(Odometry, '/icp_odom', 10)
        
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        
        self.ekf_odom_sub = self.create_subscription(
            Odometry, '/ekf_odom', self.ekf_odom_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Flags
        self.initialized = False
        
        self.get_logger().info("ICP Odometry Node Started")
        self.get_logger().info(f"ICP params - Max iter: {self.max_iterations}, Conv threshold: {self.convergence_threshold}")
    
    def ekf_odom_callback(self, msg):
        """Store EKF odometry as initial guess for ICP"""
        
        # Extract pose from EKF odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        quat = msg.pose.pose.orientation
        _, _, theta = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Store for ICP initialization
        self.ekf_pose = np.array([x, y, theta])
        self.last_ekf_time = self.get_clock().now()
        
        # Extract velocities
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y  
        omega = msg.twist.twist.angular.z
        
        self.current_velocity = np.array([vx, vy, omega])
    
    def scan_callback(self, msg):
        """Process laser scan for ICP matching"""
        current_time = self.get_clock().now()
        
        # Convert scan to cartesian points
        scan_points = self.scan_to_cartesian(msg)
        
        if len(scan_points) < self.min_scan_points:
            self.get_logger().warn(f"Insufficient scan points: {len(scan_points)}")
            return
        
        if not self.initialized:
            self.initialize_icp(scan_points, current_time)
            return
        
        # Perform ICP matching
        transformation = self.perform_icp(scan_points)
        
        if transformation is not None:
            # Update pose with ICP result
            self.update_pose_with_icp(transformation, current_time)
            
            # Publish ICP odometry
            self.publish_icp_odometry(current_time)
        
        # Store current scan for next iteration
        self.previous_scan_points = scan_points
        self.previous_time = current_time
    
    def scan_to_cartesian(self, scan_msg):
        """Convert laser scan to cartesian points"""
        points = []
        
        angle = scan_msg.angle_min
        for i, range_val in enumerate(scan_msg.ranges):
            # Skip invalid measurements
            if (range_val < scan_msg.range_min or 
                range_val > scan_msg.range_max or 
                math.isnan(range_val) or 
                math.isinf(range_val)):
                angle += scan_msg.angle_increment
                continue
            
            # Convert to cartesian (in laser frame)
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            points.append([x, y])
            
            angle += scan_msg.angle_increment
        
        return np.array(points)
    
    def initialize_icp(self, scan_points, timestamp):
        """Initialize ICP with first scan"""
        
        # Use EKF pose as initial position if available
        if self.ekf_pose is not None:
            self.current_pose = self.ekf_pose.copy()
            self.get_logger().info(f"ICP initialized with EKF pose: ({self.current_pose[0]:.3f}, {self.current_pose[1]:.3f}, {self.current_pose[2]:.3f})")
        else:
            self.current_pose = np.array([0.0, 0.0, 0.0])
            self.get_logger().info("ICP initialized at origin")
        
        self.previous_scan_points = scan_points
        self.previous_time = timestamp
        self.initialized = True
        
        # Publish initial pose
        self.publish_icp_odometry(timestamp)
    
    def perform_icp(self, current_points):
        """
        Iterative Closest Point algorithm
        Returns 2D transformation [dx, dy, dtheta] or None if failed
        """
        
        if self.previous_scan_points is None:
            return None
        
        # Use EKF odometry as initial guess for transformation
        initial_guess = np.array([0.0, 0.0, 0.0])  # Start with no change
        
        if self.ekf_pose is not None and self.last_ekf_time is not None:
            # Compute expected motion from EKF
            dt = (self.get_clock().now() - self.last_ekf_time).nanoseconds / 1e9
            if 0 < dt < 1.0:  # Reasonable time step
                vx, vy, omega = self.current_velocity
                
                # Predict motion
                dx_pred = vx * dt
                dy_pred = vy * dt
                dtheta_pred = omega * dt
                
                initial_guess = np.array([dx_pred, dy_pred, dtheta_pred])
        
        # ICP iterations
        transformation = initial_guess.copy()
        prev_error = float('inf')
        
        for iteration in range(self.max_iterations):
            # Apply current transformation to current points
            transformed_points = self.transform_points(current_points, transformation)
            
            # Find correspondences
            correspondences, distances = self.find_correspondences(
                transformed_points, self.previous_scan_points)
            
            if len(correspondences) < 10:  # Minimum required correspondences
                self.get_logger().warn(f"ICP failed: insufficient correspondences ({len(correspondences)})")
                return None
            
            # Calculate mean error
            mean_error = np.mean(distances)
            
            # Check convergence
            if abs(prev_error - mean_error) < self.convergence_threshold:
                self.get_logger().debug(f"ICP converged after {iteration+1} iterations, error: {mean_error:.6f}")
                break
            
            prev_error = mean_error
            
            # Compute optimal transformation using SVD
            delta_transform = self.compute_transformation_svd(
                current_points[correspondences[:, 0]], 
                self.previous_scan_points[correspondences[:, 1]])
            
            if delta_transform is None:
                break
            
            # Update transformation
            transformation += delta_transform
            
            # Normalize angle
            transformation[2] = math.atan2(math.sin(transformation[2]), math.cos(transformation[2]))
        
        return transformation
    
    def transform_points(self, points, transformation):
        """Apply 2D transformation to points"""
        dx, dy, dtheta = transformation
        
        cos_theta = math.cos(dtheta)
        sin_theta = math.sin(dtheta)
        
        # Rotation matrix
        R = np.array([[cos_theta, -sin_theta], 
                      [sin_theta, cos_theta]])
        
        # Apply transformation: p' = R*p + t
        transformed = points @ R.T + np.array([dx, dy])
        
        return transformed
    
    def find_correspondences(self, source_points, target_points):
        """Find closest point correspondences using KD-tree"""
        
        # Build KD-tree for target points
        tree = cKDTree(target_points)
        
        # Find nearest neighbors
        distances, indices = tree.query(source_points, distance_upper_bound=self.max_correspondence_distance)
        
        # Filter valid correspondences
        valid_mask = (distances < self.max_correspondence_distance) & (indices < len(target_points))
        valid_source_idx = np.where(valid_mask)[0]
        valid_target_idx = indices[valid_mask]
        valid_distances = distances[valid_mask]
        
        # Create correspondence pairs
        correspondences = np.column_stack([valid_source_idx, valid_target_idx])
        
        return correspondences, valid_distances
    
    def compute_transformation_svd(self, source_points, target_points):
        """Compute optimal transformation using SVD"""
        
        if len(source_points) != len(target_points) or len(source_points) < 3:
            return None
        
        # Calculate centroids
        centroid_source = np.mean(source_points, axis=0)
        centroid_target = np.mean(target_points, axis=0)
        
        # Center the points
        source_centered = source_points - centroid_source
        target_centered = target_points - centroid_target
        
        # Cross-covariance matrix
        H = source_centered.T @ target_centered
        
        # SVD
        try:
            U, _, Vt = np.linalg.svd(H)
        except np.linalg.LinAlgError:
            return None
        
        # Rotation matrix
        R = Vt.T @ U.T
        
        # Ensure proper rotation (det(R) = 1)
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Extract rotation angle
        dtheta = math.atan2(R[1, 0], R[0, 0])
        
        # Translation
        t = centroid_target - R @ centroid_source
        dx, dy = t
        
        return np.array([dx, dy, dtheta])
    
    def update_pose_with_icp(self, transformation, current_time):
        """Update robot pose using ICP transformation"""
        
        dx, dy, dtheta = transformation
        
        # Apply transformation to current pose
        cos_theta = math.cos(self.current_pose[2])
        sin_theta = math.sin(self.current_pose[2])
        
        # Transform incremental motion to global frame
        global_dx = dx * cos_theta - dy * sin_theta
        global_dy = dx * sin_theta + dy * cos_theta
        
        # Update pose
        self.current_pose[0] += global_dx
        self.current_pose[1] += global_dy
        self.current_pose[2] += dtheta
        
        # Normalize angle
        self.current_pose[2] = math.atan2(math.sin(self.current_pose[2]), math.cos(self.current_pose[2]))
        
        # Update velocities based on time difference
        if self.previous_time is not None:
            dt = (current_time - self.previous_time).nanoseconds / 1e9
            if dt > 0:
                self.current_velocity[0] = global_dx / dt
                self.current_velocity[1] = global_dy / dt
                self.current_velocity[2] = dtheta / dt
    
    def publish_icp_odometry(self, timestamp):
        """Publish ICP-refined odometry"""
        
        x, y, theta = self.current_pose
        vx, vy, omega = self.current_velocity
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position = Point(x=x, y=y, z=0.0)
        
        # Orientation
        quat = tf_transformations.quaternion_from_euler(0, 0, theta)
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        # Velocities
        odom.twist.twist.linear = Vector3(x=vx, y=vy, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=omega)
        
        # Covariance (lower uncertainty than wheel/EKF due to scan matching)
        pose_cov = np.zeros(36)
        twist_cov = np.zeros(36)
        
        # Lower covariance values (higher confidence)
        pose_cov[0] = 0.01   # x variance
        pose_cov[7] = 0.01   # y variance
        pose_cov[35] = 0.05  # yaw variance
        
        twist_cov[0] = 0.01   # vx variance
        twist_cov[7] = 0.01   # vy variance  
        twist_cov[35] = 0.05  # omega variance
        
        # High uncertainty for unused DOFs
        pose_cov[14] = 1e6  # z
        pose_cov[21] = 1e6  # roll
        pose_cov[28] = 1e6  # pitch
        
        twist_cov[14] = 1e6  # vz
        twist_cov[21] = 1e6  # angular x
        twist_cov[28] = 1e6  # angular y
        
        odom.pose.covariance = pose_cov.tolist()
        odom.twist.covariance = twist_cov.tolist()
        
        # Publish
        self.icp_odom_pub.publish(odom)
        
        # Broadcast transform
        self.broadcast_transform(timestamp, x, y, theta)
    
    def broadcast_transform(self, timestamp, x, y, theta):
        """Broadcast ICP odom -> base_link transform"""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom_icp'
        t.child_frame_id = 'base_link_icp'
        
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        
        quat = tf_transformations.quaternion_from_euler(0, 0, theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    icp_odom = ICPOdometry()
    
    try:
        rclpy.spin(icp_odom)
    except KeyboardInterrupt:
        pass
    finally:
        icp_odom.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()