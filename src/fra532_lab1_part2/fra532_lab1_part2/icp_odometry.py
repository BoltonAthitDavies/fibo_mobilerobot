#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class ICPOdometry(Node):
    def __init__(self):
        super().__init__('icp_odometry')
        
        # ICP parameters
        self.max_iterations = 50
        self.convergence_threshold = 1e-6
        self.max_correspondence_distance = 0.3
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous scan and odometry
        self.prev_scan = None
        self.prev_scan_time = None
        self.prev_odom = None
        
        # Current EKF odometry for initial guess
        self.current_ekf_odom = None
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        
        self.ekf_odom_sub = self.create_subscription(
            Odometry,
            '/ekf_odom',
            self.ekf_odom_callback,
            10)
        
        # Publisher
        self.icp_odom_pub = self.create_publisher(
            Odometry,
            '/icp_odom',
            10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('ICP Odometry Node Started')

    def ekf_odom_callback(self, msg):
        """Store current EKF odometry for initial guess"""
        self.current_ekf_odom = msg

    def scan_to_points(self, scan_msg):
        """Convert LaserScan to 2D points"""
        points = []
        angle = scan_msg.angle_min
        
        for i, range_val in enumerate(scan_msg.ranges):
            if scan_msg.range_min <= range_val <= scan_msg.range_max:
                # Convert polar to cartesian
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                points.append([x, y])
            
            angle += scan_msg.angle_increment
        
        return np.array(points)

    def transform_points(self, points, dx, dy, dtheta):
        """Apply 2D transformation to points"""
        cos_theta = math.cos(dtheta)
        sin_theta = math.sin(dtheta)
        
        # Rotation matrix
        R = np.array([
            [cos_theta, -sin_theta],
            [sin_theta, cos_theta]
        ])
        
        # Translation
        t = np.array([dx, dy])
        
        # Transform points: R * p + t
        transformed_points = (R @ points.T).T + t
        
        return transformed_points

    def find_correspondences(self, source_points, target_points, max_distance):
        """Find nearest neighbor correspondences between point clouds"""
        correspondences = []
        
        for i, source_point in enumerate(source_points):
            min_distance = float('inf')
            best_match = -1
            
            for j, target_point in enumerate(target_points):
                distance = np.linalg.norm(source_point - target_point)
                if distance < min_distance and distance < max_distance:
                    min_distance = distance
                    best_match = j
            
            if best_match != -1:
                correspondences.append((i, best_match, min_distance))
        
        return correspondences

    def compute_transformation(self, source_points, target_points, correspondences):
        """Compute optimal transformation using least squares"""
        if len(correspondences) < 3:
            return 0.0, 0.0, 0.0, False
        
        # Extract corresponding points
        source_corr = []
        target_corr = []
        
        for src_idx, tgt_idx, _ in correspondences:
            source_corr.append(source_points[src_idx])
            target_corr.append(target_points[tgt_idx])
        
        source_corr = np.array(source_corr)
        target_corr = np.array(target_corr)
        
        # Compute centroids
        source_centroid = np.mean(source_corr, axis=0)
        target_centroid = np.mean(target_corr, axis=0)
        
        # Center the points
        source_centered = source_corr - source_centroid
        target_centered = target_corr - target_centroid
        
        # Compute rotation using SVD
        H = source_centered.T @ target_centered
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        
        # Ensure proper rotation matrix
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        
        # Extract angle
        dtheta = math.atan2(R[1, 0], R[0, 0])
        
        # Compute translation
        dt = target_centroid - R @ source_centroid
        dx, dy = dt[0], dt[1]
        
        return dx, dy, dtheta, True

    def icp_algorithm(self, source_points, target_points, initial_guess):
        """Main ICP algorithm"""
        dx, dy, dtheta = initial_guess
        
        for iteration in range(self.max_iterations):
            # Transform source points with current estimate
            transformed_source = self.transform_points(source_points, dx, dy, dtheta)
            
            # Find correspondences
            correspondences = self.find_correspondences(
                transformed_source, target_points, self.max_correspondence_distance)
            
            if len(correspondences) < 3:
                self.get_logger().warn(f'ICP: Too few correspondences ({len(correspondences)})')
                return dx, dy, dtheta, False
            
            # Compute transformation correction
            delta_dx, delta_dy, delta_dtheta, success = self.compute_transformation(
                transformed_source, target_points, correspondences)
            
            if not success:
                return dx, dy, dtheta, False
            
            # Update transformation
            prev_dx, prev_dy, prev_dtheta = dx, dy, dtheta
            dx += delta_dx
            dy += delta_dy
            dtheta += delta_dtheta
            
            # Check convergence
            change = math.sqrt(delta_dx**2 + delta_dy**2 + delta_dtheta**2)
            if change < self.convergence_threshold:
                self.get_logger().debug(f'ICP converged in {iteration+1} iterations')
                break
        
        return dx, dy, dtheta, True

    def get_initial_guess(self):
        """Get initial transformation guess from EKF odometry"""
        if self.current_ekf_odom is None or self.prev_odom is None:
            return 0.0, 0.0, 0.0
        
        # Extract poses
        curr_x = self.current_ekf_odom.pose.pose.position.x
        curr_y = self.current_ekf_odom.pose.pose.position.y
        curr_quat = self.current_ekf_odom.pose.pose.orientation
        _, _, curr_yaw = tf_transformations.euler_from_quaternion([
            curr_quat.x, curr_quat.y, curr_quat.z, curr_quat.w])
        
        prev_x = self.prev_odom.pose.pose.position.x
        prev_y = self.prev_odom.pose.pose.position.y
        prev_quat = self.prev_odom.pose.pose.orientation
        _, _, prev_yaw = tf_transformations.euler_from_quaternion([
            prev_quat.x, prev_quat.y, prev_quat.z, prev_quat.w])
        
        # Compute relative transformation
        dx = curr_x - prev_x
        dy = curr_y - prev_y
        dtheta = curr_yaw - prev_yaw
        
        # Normalize angle
        dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
        
        return dx, dy, dtheta

    def scan_callback(self, scan_msg):
        """Process new scan with ICP"""
        current_time = self.get_clock().now()
        
        # Convert scan to points
        current_points = self.scan_to_points(scan_msg)
        
        if len(current_points) < 10:
            self.get_logger().warn('Too few valid scan points')
            return
        
        if self.prev_scan is not None:
            # Get initial transformation guess
            initial_guess = self.get_initial_guess()
            
            # Run ICP
            dx, dy, dtheta, success = self.icp_algorithm(
                current_points, self.prev_scan, initial_guess)
            
            if success:
                # Update pose
                cos_theta = math.cos(self.theta)
                sin_theta = math.sin(self.theta)
                
                # Transform relative motion to global frame
                global_dx = cos_theta * dx - sin_theta * dy
                global_dy = sin_theta * dx + cos_theta * dy
                
                self.x += global_dx
                self.y += global_dy
                self.theta += dtheta
                
                # Normalize angle
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
                
                # Calculate velocities
                dt = (current_time - self.prev_scan_time).nanoseconds / 1e9
                if dt > 0:
                    linear_vel = math.sqrt(global_dx**2 + global_dy**2) / dt
                    angular_vel = dtheta / dt
                else:
                    linear_vel = 0.0
                    angular_vel = 0.0
                
                # Publish results
                self.publish_icp_odometry(current_time, linear_vel, angular_vel)
                self.publish_tf_transform(current_time)
            
            else:
                self.get_logger().warn('ICP failed to converge')
        
        # Store current scan for next iteration
        self.prev_scan = current_points
        self.prev_scan_time = current_time
        self.prev_odom = self.current_ekf_odom

    def publish_icp_odometry(self, timestamp, linear_vel, angular_vel):
        """Publish ICP odometry result"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        
        # Orientation
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        
        # Velocity
        odom_msg.twist.twist.linear = Vector3(x=linear_vel, y=0.0, z=0.0)
        odom_msg.twist.twist.angular = Vector3(x=0.0, y=0.0, z=angular_vel)
        
        # Covariance (estimated from ICP uncertainty)
        odom_msg.pose.covariance[0] = 0.05   # x
        odom_msg.pose.covariance[7] = 0.05   # y
        odom_msg.pose.covariance[35] = 0.02  # yaw
        
        odom_msg.twist.covariance[0] = 0.05   # vx
        odom_msg.twist.covariance[35] = 0.02  # vyaw
        
        self.icp_odom_pub.publish(odom_msg)

    def publish_tf_transform(self, timestamp):
        """Publish TF transform"""
        transform = TransformStamped()
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link_icp'
        
        # Translation
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        
        # Rotation
        quaternion = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        transform.transform.rotation = Quaternion(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3]
        )
        
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    
    icp_node = ICPOdometry()
    
    try:
        rclpy.spin(icp_node)
    except KeyboardInterrupt:
        pass
    
    icp_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()