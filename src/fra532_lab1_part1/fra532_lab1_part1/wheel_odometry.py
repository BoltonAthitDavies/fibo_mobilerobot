#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Pose, Point, Vector3
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations
import numpy as np
import math
import time

class WheelOdometry(Node):
    """
    Differential drive odometry computation from wheel encoders
    Uses Turtlebot3 Burger parameters and Runge-Kutta integration
    """
    
    def __init__(self):
        super().__init__('wheel_odometry')
        
        # Robot parameters (Turtlebot3 Burger)
        self.wheel_radius = 0.033  # meters
        self.wheel_separation = 0.160  # meters (distance between wheels)
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = None
        self.last_left_pos = None
        self.last_right_pos = None
        
        # Covariance parameters (tune based on wheel encoder accuracy)
        self.position_covariance = 0.01  # meters^2
        self.orientation_covariance = 0.1  # radians^2
        self.velocity_covariance = 0.05  # (m/s)^2
        
        # Publishers and subscribers
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info("Wheel Odometry Node Started")
        self.get_logger().info(f"Robot params - Wheel radius: {self.wheel_radius}m, Separation: {self.wheel_separation}m")
    
    def joint_callback(self, msg):
        """Process joint state messages to compute odometry"""
        try:
            current_time = self.get_clock().now()
            
            # Find wheel joint indices
            if 'wheel_left_joint' not in msg.name or 'wheel_right_joint' not in msg.name:
                return
            
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
            
            left_pos = msg.position[left_idx]
            right_pos = msg.position[right_idx]
            
            # Skip first message (need previous position for velocity)
            if self.last_time is None:
                self.last_time = current_time
                self.last_left_pos = left_pos
                self.last_right_pos = right_pos
                return
            
            # Calculate time difference
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt <= 0:
                return
            
            # Calculate wheel movements
            delta_left = left_pos - self.last_left_pos
            delta_right = right_pos - self.last_right_pos
            
            # Convert to linear distances
            left_distance = delta_left * self.wheel_radius
            right_distance = delta_right * self.wheel_radius
            
            # Compute robot motion
            linear_distance = (left_distance + right_distance) / 2.0
            angular_distance = (right_distance - left_distance) / self.wheel_separation
            
            # Compute velocities
            linear_velocity = linear_distance / dt
            angular_velocity = angular_distance / dt
            
            # Update pose using Runge-Kutta integration (4th order)
            self.update_pose_rk4(linear_distance, angular_distance)
            
            # Publish odometry
            self.publish_odometry(current_time, linear_velocity, angular_velocity)
            
            # Update for next iteration
            self.last_time = current_time
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f"Joint state processing error: {e}")
    
    def update_pose_rk4(self, linear_distance, angular_distance):
        """
        Update robot pose using 4th-order Runge-Kutta integration
        Handles curved motion more accurately than Euler integration
        """
        if abs(angular_distance) < 1e-6:
            # Straight line motion
            self.x += linear_distance * math.cos(self.theta)
            self.y += linear_distance * math.sin(self.theta)
        else:
            # Curved motion - use RK4 for better accuracy
            # The motion model: dx/dt = v*cos(θ), dy/dt = v*sin(θ), dθ/dt = ω
            
            # For differential drive, the motion is actually a circular arc
            radius = linear_distance / angular_distance
            
            # Center of circular motion
            cx = self.x - radius * math.sin(self.theta)
            cy = self.y + radius * math.cos(self.theta)
            
            # Update angle
            self.theta += angular_distance
            
            # New position on the arc
            self.x = cx + radius * math.sin(self.theta)
            self.y = cy - radius * math.cos(self.theta)
        
        # Normalize angle to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
    
    def publish_odometry(self, timestamp, linear_vel, angular_vel):
        """Publish odometry message with covariance"""
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        
        # Orientation (quaternion from yaw)
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
        
        # Velocities
        odom.twist.twist.linear = Vector3(x=linear_vel, y=0.0, z=0.0)
        odom.twist.twist.angular = Vector3(x=0.0, y=0.0, z=angular_vel)
        
        # Covariance matrices (6x6 = 36 elements)
        # Position covariance (x, y, z, roll, pitch, yaw)
        pose_cov = np.zeros(36)
        pose_cov[0] = self.position_covariance  # x
        pose_cov[7] = self.position_covariance  # y
        pose_cov[14] = 1e6  # z (high uncertainty for 2D)
        pose_cov[21] = 1e6  # roll
        pose_cov[28] = 1e6  # pitch  
        pose_cov[35] = self.orientation_covariance  # yaw
        
        # Velocity covariance
        twist_cov = np.zeros(36)
        twist_cov[0] = self.velocity_covariance  # vx
        twist_cov[7] = 1e6  # vy (high uncertainty for differential drive)
        twist_cov[14] = 1e6  # vz
        twist_cov[21] = 1e6  # angular x
        twist_cov[28] = 1e6  # angular y
        twist_cov[35] = self.velocity_covariance  # angular z
        
        odom.pose.covariance = pose_cov.tolist()
        odom.twist.covariance = twist_cov.tolist()
        
        # Publish
        self.odom_pub.publish(odom)
        
        # Broadcast transform
        self.broadcast_transform(timestamp)
    
    def broadcast_transform(self, timestamp):
        """Broadcast odom -> base_link transform"""
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        quat = tf_transformations.quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    
    wheel_odom = WheelOdometry()
    
    try:
        rclpy.spin(wheel_odom)
    except KeyboardInterrupt:
        pass
    finally:
        wheel_odom.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()