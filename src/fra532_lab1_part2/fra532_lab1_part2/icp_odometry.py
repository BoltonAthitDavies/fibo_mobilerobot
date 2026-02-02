#!/usr/bin/env python3
"""
ICP Odometry Node
Refines odometry using LiDAR scan matching with ICP algorithm
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import numpy as np
import math


class ICPOdometry(Node):
    def __init__(self):
        super().__init__('icp_odometry_node')
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous scan
        self.prev_scan_points = None
        self.prev_time = None
        
        # ICP parameters
        self.max_iterations = 50
        self.convergence_threshold = 1e-5
        self.max_correspondence_distance = 0.5
        
        # Subscriber
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        # Subscriber for initial odometry (EKF)
        self.odom_sub = self.create_subscription(
            Odometry,
            '/ekf_odom',
            self.odom_callback,
            10
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/icp_odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Store EKF odometry for initial guess
        self.ekf_odom = None
        
        self.get_logger().info('ICP Odometry node initialized')
    
    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q
    
    def scan_to_points(self, scan):
        """Convert LaserScan to 2D points"""
        points = []
        angle = scan.angle_min
        
        for r in scan.ranges:
            if scan.range_min < r < scan.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                points.append([x, y])
            angle += scan.angle_increment
        
        return np.array(points) if points else np.empty((0, 2))
    
    def find_correspondences(self, source, target):
        """Find nearest neighbors between source and target point clouds"""
        correspondences = []
        
        for i, src_point in enumerate(source):
            min_dist = float('inf')
            min_idx = -1
            
            for j, tgt_point in enumerate(target):
                dist = np.linalg.norm(src_point - tgt_point)
                if dist < min_dist:
                    min_dist = dist
                    min_idx = j
            
            if min_dist < self.max_correspondence_distance:
                correspondences.append((i, min_idx, min_dist))
        
        return correspondences
    
    def transform_points(self, points, tx, ty, theta):
        """Apply 2D transformation to points"""
        cos_t = math.cos(theta)
        sin_t = math.sin(theta)
        
        rotation = np.array([[cos_t, -sin_t],
                           [sin_t, cos_t]])
        translation = np.array([tx, ty])
        
        return (points @ rotation.T) + translation
    
    def icp(self, source, target, initial_guess=(0.0, 0.0, 0.0)):
        """Iterative Closest Point algorithm"""
        tx, ty, theta = initial_guess
        
        for iteration in range(self.max_iterations):
            # Transform source points
            transformed_source = self.transform_points(source, tx, ty, theta)
            
            # Find correspondences
            correspondences = self.find_correspondences(transformed_source, target)
            
            if len(correspondences) < 3:
                break
            
            # Extract corresponding points
            src_pts = np.array([transformed_source[i] for i, _, _ in correspondences])
            tgt_pts = np.array([target[j] for _, j, _ in correspondences])
            
            # Compute centroids
            src_centroid = np.mean(src_pts, axis=0)
            tgt_centroid = np.mean(tgt_pts, axis=0)
            
            # Center the points
            src_centered = src_pts - src_centroid
            tgt_centered = tgt_pts - tgt_centroid
            
            # Compute covariance matrix
            H = src_centered.T @ tgt_centered
            
            # SVD
            U, _, Vt = np.linalg.svd(H)
            R = Vt.T @ U.T
            
            # Ensure proper rotation (det(R) should be 1)
            if np.linalg.det(R) < 0:
                Vt[-1, :] *= -1
                R = Vt.T @ U.T
            
            # Compute incremental transformation
            delta_theta = math.atan2(R[1, 0], R[0, 0])
            delta_t = tgt_centroid - (R @ src_centroid)
            
            # Update transformation
            old_tx, old_ty, old_theta = tx, ty, theta
            
            cos_t = math.cos(theta)
            sin_t = math.sin(theta)
            tx += delta_t[0] * cos_t - delta_t[1] * sin_t
            ty += delta_t[0] * sin_t + delta_t[1] * cos_t
            theta += delta_theta
            
            # Check convergence
            change = abs(tx - old_tx) + abs(ty - old_ty) + abs(theta - old_theta)
            if change < self.convergence_threshold:
                break
        
        return tx, ty, theta
    
    def odom_callback(self, msg):
        """Store EKF odometry for initial guess"""
        self.ekf_odom = msg
    
    def scan_callback(self, msg):
        """Process laser scan for ICP-based odometry"""
        current_time = self.get_clock().now()
        
        # Convert scan to points
        current_points = self.scan_to_points(msg)
        
        if len(current_points) < 10:
            self.get_logger().warn('Not enough scan points')
            return
        
        if self.prev_scan_points is None:
            self.prev_scan_points = current_points
            self.prev_time = current_time
            return
        
        # Use EKF odometry as initial guess if available
        initial_guess = (0.0, 0.0, 0.0)
        if self.ekf_odom is not None:
            # This is simplified - in practice, you'd compute the relative transform
            pass
        
        # Run ICP
        dx, dy, dtheta = self.icp(current_points, self.prev_scan_points, initial_guess)
        
        # Update global pose
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)
        self.x += dx * cos_t - dy * sin_t
        self.y += dx * sin_t + dy * cos_t
        self.theta += dtheta
        
        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry
        self.publish_odometry(current_time)
        
        # Update previous scan
        self.prev_scan_points = current_points
        self.prev_time = current_time
    
    def publish_odometry(self, current_time):
        """Publish odometry message and TF transform"""
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation
        odom.pose.pose.orientation = self.quaternion_from_euler(0, 0, self.theta)
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = self.quaternion_from_euler(0, 0, self.theta)
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = ICPOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
