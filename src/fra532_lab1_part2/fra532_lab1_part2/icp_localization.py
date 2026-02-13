#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Twist, TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import numpy as np
import math
from rclpy.time import Time
import tf2_geometry_msgs

class ICPLocalizationNode(Node):
    def __init__(self):
        super().__init__('icp_localization_node')
        
        # Robot parameters
        self.robot_pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
        self.pose_covariance = np.eye(3) * 0.1
        
        # ICP parameters
        self.max_iterations = 50
        self.convergence_threshold = 0.001
        self.max_correspondence_distance = 1.0
        
        # Scan-to-scan matching storage
        self.previous_scan = None
        self.previous_points = None
        self.first_scan = True
        
        # EKF odometry for initial guess
        self.ekf_pose = np.array([0.0, 0.0, 0.0])
        self.previous_ekf_pose = np.array([0.0, 0.0, 0.0])
        self.ekf_available = False
        self.ekf_timeout = 5.0  # seconds to wait for EKF data
        
        # Publishers and subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        # Subscribe to EKF odometry for initial guess
        self.ekf_odom_sub = self.create_subscription(
            Odometry,
            '/ekf_odom',
            self.ekf_odom_callback,
            10
        )
        
        # Publishers
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/icp_pose', 
            10
        )
        
        self.refined_odom_pub = self.create_publisher(
            Odometry,
            '/icp_refined_odom',
            10
        )
        
        # TF
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Current odometry
        self.current_odom = None
        
        self.get_logger().info('ICP Odometry Refinement Node started')
        self.get_logger().info('Waiting for /ekf_odom data for initial guess (or will use zero motion)')
        
        # Timer to check EKF status
        self.status_timer = self.create_timer(2.0, self.log_status)
        
    def laser_callback(self, msg):
        """Process laser scan for ICP odometry refinement."""
        
        try:
            # Convert laser scan to points
            current_points = self.laser_scan_to_points(msg)
            
            if self.first_scan:
                # Store first scan
                self.previous_scan = msg
                self.previous_points = current_points
                self.first_scan = False
                self.get_logger().info('First scan stored for consecutive matching')
                return
            
            if self.previous_points is None or len(current_points) == 0:
                return
            
            # Use EKF odometry as initial guess if available
            initial_guess = np.array([0.0, 0.0, 0.0])
            if self.ekf_available:
                # Calculate relative motion from EKF as initial guess
                initial_guess = self.calculate_initial_guess()
                self.get_logger().debug('Using EKF initial guess: [{:.3f}, {:.3f}, {:.3f}]'.format(*initial_guess))
            else:
                self.get_logger().debug('No EKF data available, using zero initial guess')
            
            # Perform ICP between consecutive scans
            relative_transform = self.run_icp(
                self.previous_points, 
                current_points, 
                initial_guess
            )
            
            # Integrate relative transformation
            self.integrate_transformation(relative_transform)
            
            # Publish refined pose and odometry
            self.publish_icp_pose(msg.header.stamp)
            
            # Update for next iteration
            self.previous_scan = msg
            self.previous_points = current_points
            
        except Exception as e:
            self.get_logger().error(f'Error in laser callback: {str(e)}')
    
    def run_icp(self, previous_points, current_points, initial_guess=None):
        """
        TODO: Implement your ICP algorithm here for consecutive scan matching.
        
        Args:
            previous_points: numpy array of shape (N, 2) - previous scan points
            current_points: numpy array of shape (M, 2) - current scan points
            initial_guess: numpy array [dx, dy, dtheta] - initial transformation guess from EKF
            
        Returns:
            relative_transform: numpy array [dx, dy, dtheta] - relative transformation
        """
        
        # Initialize transformation with EKF initial guess if available
        if initial_guess is not None:
            transformation = initial_guess.copy()
        else:
            transformation = np.array([0.0, 0.0, 0.0])  # [dx, dy, dtheta]
        
        # Apply initial guess to current points
        aligned_points = current_points.copy()
        if initial_guess is not None:
            aligned_points = self.apply_transformation(aligned_points, initial_guess)
        
        for iteration in range(self.max_iterations):
            # TODO: Step 1 - Find correspondences (nearest neighbors)
            correspondences = self.find_correspondences(previous_points, aligned_points)
            
            if len(correspondences) < 3:
                self.get_logger().warn('Too few correspondences found')
                break
            
            # TODO: Step 2 - Estimate transformation (least squares)
            delta_transform = self.estimate_transformation(
                correspondences, previous_points, aligned_points
            )
            
            # TODO: Step 3 - Apply transformation
            transformation += delta_transform
            aligned_points = self.apply_transformation(aligned_points, delta_transform)
            
            # TODO: Step 4 - Check convergence
            if np.linalg.norm(delta_transform) < self.convergence_threshold:
                self.get_logger().debug(f'ICP converged after {iteration + 1} iterations')
                break
        
        return transformation
    
    def find_correspondences(self, reference_points, current_points):
        """
        TODO: Implement correspondence finding.
        Find the nearest neighbor in reference_points for each point in current_points.
        
        Args:
            reference_points: numpy array of shape (N, 2)
            current_points: numpy array of shape (M, 2)
            
        Returns:
            correspondences: list of tuples (current_point_idx, reference_point_idx)
        """
        
        # PLACEHOLDER: You need to implement this
        correspondences = []
        
        # Example structure:
        for i, current_point in enumerate(current_points):
            # Find nearest neighbor in reference_points
            distances = np.linalg.norm(reference_points - current_point, axis=1)
            nearest_idx = np.argmin(distances)
            
            if distances[nearest_idx] < self.max_correspondence_distance:
                correspondences.append((i, nearest_idx))
        
        return correspondences
    
    def estimate_transformation(self, correspondences, previous_points, current_points):
        """
        TODO: Implement transformation estimation.
        Estimate the relative transformation (dx, dy, dtheta) from correspondences.
        
        Args:
            correspondences: list of tuples (current_point_idx, previous_point_idx)
            previous_points: numpy array of previous scan points
            current_points: numpy array of current scan points
            
        Returns:
            transformation: numpy array [dx, dy, dtheta]
        """
        
        # PLACEHOLDER: You need to implement this
        transformation = np.array([0.0, 0.0, 0.0])
        
        # TODO: Implement least squares solution for 2D transformation
        # Example structure:
        if len(correspondences) < 3:
            return transformation
            
        # Extract corresponding points
        curr_pts = np.array([current_points[i] for i, j in correspondences])
        prev_pts = np.array([previous_points[j] for i, j in correspondences])
        
        # Calculate centroids
        curr_centroid = np.mean(curr_pts, axis=0)
        prev_centroid = np.mean(prev_pts, axis=0)
        
        # Center points
        curr_centered = curr_pts - curr_centroid
        prev_centered = prev_pts - prev_centroid
        
        # Estimate rotation using SVD
        H = curr_centered.T @ prev_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Extract angle
        dtheta = math.atan2(R[1,0], R[0,0])
        
        # Estimate translation
        dx = prev_centroid[0] - curr_centroid[0]
        dy = prev_centroid[1] - curr_centroid[1]
        
        transformation = np.array([dx, dy, dtheta])
        
        return transformation
    
    def apply_transformation(self, points, transformation):
        """
        Apply 2D transformation to points.
        
        Args:
            points: numpy array of shape (N, 2)
            transformation: numpy array [dx, dy, dtheta]
            
        Returns:
            transformed_points: numpy array of shape (N, 2)
        """
        
        dx, dy, dtheta = transformation
        
        # Create rotation matrix
        cos_theta = math.cos(dtheta)
        sin_theta = math.sin(dtheta)
        
        # Apply rotation and translation
        transformed_points = np.zeros_like(points)
        transformed_points[:, 0] = (cos_theta * points[:, 0] - 
                                  sin_theta * points[:, 1] + dx)
        transformed_points[:, 1] = (sin_theta * points[:, 0] + 
                                  cos_theta * points[:, 1] + dy)
        
        return transformed_points
    
    def laser_scan_to_points(self, laser_scan):
        """Convert LaserScan message to 2D points in robot frame."""
        
        points = []
        angle = laser_scan.angle_min
        
        for i, range_val in enumerate(laser_scan.ranges):
            if (laser_scan.range_min <= range_val <= laser_scan.range_max):
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                points.append([x, y])
            
            angle += laser_scan.angle_increment
        
        return np.array(points)
    
    def odom_callback(self, msg):
        """Store current odometry for reference."""
        self.current_odom = msg
    
    def ekf_odom_callback(self, msg):
        """Store EKF odometry for initial guess."""
        # Extract pose from EKF odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        # Convert quaternion to euler angle
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        theta = 2 * math.atan2(qz, qw)
        
        # Store previous pose for relative motion calculation
        if self.ekf_available:
            self.previous_ekf_pose = self.ekf_pose.copy()
        
        self.ekf_pose = np.array([x, y, theta])
        
        if not self.ekf_available:
            self.get_logger().info('EKF odometry data received - initial guess now available')
            self.ekf_available = True
    
    def calculate_initial_guess(self):
        """Calculate initial guess for ICP from EKF motion."""
        if not hasattr(self, 'previous_ekf_pose'):
            return np.array([0.0, 0.0, 0.0])
        
        # Calculate relative motion from EKF
        dx = self.ekf_pose[0] - self.previous_ekf_pose[0]
        dy = self.ekf_pose[1] - self.previous_ekf_pose[1]
        dtheta = self.ekf_pose[2] - self.previous_ekf_pose[2]
        
        # Normalize angle
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
        
        return np.array([dx, dy, dtheta])
    
    def log_status(self):
        """Log current status of the ICP refinement node."""
        if self.ekf_available:
            self.get_logger().debug('Status: EKF available, ICP refinement active')
        else:
            self.get_logger().warn('Status: No EKF data - make sure Part 1 is running or launch with launch_ekf:=true')
    
    def integrate_transformation(self, relative_transform):
        """Integrate relative transformation into global pose."""
        dx, dy, dtheta = relative_transform
        
        # Apply relative transformation to global pose
        cos_theta = math.cos(self.robot_pose[2])
        sin_theta = math.sin(self.robot_pose[2])
        
        # Transform relative motion to global coordinates
        global_dx = cos_theta * dx - sin_theta * dy
        global_dy = sin_theta * dx + cos_theta * dy
        
        # Update global pose
        self.robot_pose[0] += global_dx
        self.robot_pose[1] += global_dy
        self.robot_pose[2] += dtheta
        
        # Normalize theta
        self.robot_pose[2] = math.atan2(
            math.sin(self.robot_pose[2]), 
            math.cos(self.robot_pose[2])
        )
    
    def publish_icp_pose(self, timestamp):
        """Publish ICP-corrected pose."""
        
        # Pose with covariance
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = timestamp
        pose_msg.header.frame_id = 'map'
        
        pose_msg.pose.pose.position.x = float(self.robot_pose[0])
        pose_msg.pose.pose.position.y = float(self.robot_pose[1])
        pose_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion
        theta = float(self.robot_pose[2])
        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Set covariance (simplified)
        pose_msg.pose.covariance[0] = float(self.pose_covariance[0, 0])   # x
        pose_msg.pose.covariance[7] = float(self.pose_covariance[1, 1])   # y  
        pose_msg.pose.covariance[35] = float(self.pose_covariance[2, 2])  # theta
        
        self.pose_pub.publish(pose_msg)
        
        # Publish TF transform
        self.publish_transform(timestamp, theta)
        
        # Publish refined odometry
        self.publish_refined_odom(timestamp)
    
    def publish_transform(self, timestamp, theta):
        """Publish TF transform from map to base_footprint."""
        
        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'base_footprint'
        
        tf_msg.transform.translation.x = float(self.robot_pose[0])
        tf_msg.transform.translation.y = float(self.robot_pose[1])
        tf_msg.transform.translation.z = 0.0
        
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(theta / 2.0)
        tf_msg.transform.rotation.w = math.cos(theta / 2.0)
        
        self.tf_broadcaster.sendTransform(tf_msg)
    
    def publish_refined_odom(self, timestamp):
        """Publish ICP-refined odometry."""
        
        refined_odom = Odometry()
        refined_odom.header.stamp = timestamp
        refined_odom.header.frame_id = 'map'
        refined_odom.child_frame_id = 'base_footprint'
        
        # Use ICP-refined pose
        refined_odom.pose.pose.position.x = float(self.robot_pose[0])
        refined_odom.pose.pose.position.y = float(self.robot_pose[1])
        refined_odom.pose.pose.position.z = 0.0
        
        theta = float(self.robot_pose[2])
        refined_odom.pose.pose.orientation.x = 0.0
        refined_odom.pose.pose.orientation.y = 0.0
        refined_odom.pose.pose.orientation.z = math.sin(theta / 2.0)
        refined_odom.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Set covariance (refined estimate should have lower uncertainty)
        refined_odom.pose.covariance[0] = 0.01   # x variance
        refined_odom.pose.covariance[7] = 0.01   # y variance
        refined_odom.pose.covariance[35] = 0.05  # theta variance
        
        # Copy velocity from original odometry if available
        if self.current_odom:
            refined_odom.twist = self.current_odom.twist
        
        self.refined_odom_pub.publish(refined_odom)


def main(args=None):
    rclpy.init(args=args)
    
    node = ICPLocalizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()