#!/usr/bin/env python3
"""
Wheel Odometry Node
Computes odometry from /joint_states using differential drive kinematics
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import numpy as np
import math


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        
        # Declare and get parameters
        self.declare_parameter('wheel_radius', 0.033)
        self.declare_parameter('wheel_separation', 0.160)
        
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        
        self.get_logger().info(f'Wheel radius: {self.wheel_radius} m')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation} m')
        
        # Initialize state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.last_left_pos = None
        self.last_right_pos = None
        self.last_time = None
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        self.get_logger().info('Wheel Odometry Node initialized')
    
    def joint_states_callback(self, msg):
        """Process joint states and compute wheel odometry"""
        try:
            # Extract wheel positions (assuming standard TurtleBot3 joint names)
            # Joint names are typically: ['wheel_left_joint', 'wheel_right_joint']
            left_idx = msg.name.index('wheel_left_joint')
            right_idx = msg.name.index('wheel_right_joint')
            
            left_pos = msg.position[left_idx]
            right_pos = msg.position[right_idx]
            
            current_time = self.get_clock().now()
            
            # Initialize on first message
            if self.last_left_pos is None:
                self.last_left_pos = left_pos
                self.last_right_pos = right_pos
                self.last_time = current_time
                return
            
            # Compute wheel displacements
            delta_left = left_pos - self.last_left_pos
            delta_right = right_pos - self.last_right_pos
            
            # Differential drive kinematics
            delta_s_left = delta_left * self.wheel_radius
            delta_s_right = delta_right * self.wheel_radius
            
            # Linear and angular displacement
            delta_s = (delta_s_left + delta_s_right) / 2.0
            delta_theta = (delta_s_right - delta_s_left) / self.wheel_separation
            
            # Update pose (using midpoint integration)
            self.x += delta_s * math.cos(self.theta + delta_theta / 2.0)
            self.y += delta_s * math.sin(self.theta + delta_theta / 2.0)
            self.theta += delta_theta
            
            # Normalize theta to [-pi, pi]
            self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
            
            # Compute velocities
            dt = (current_time - self.last_time).nanoseconds / 1e9
            if dt > 0:
                v = delta_s / dt
                omega = delta_theta / dt
            else:
                v = 0.0
                omega = 0.0
            
            # Publish odometry
            self.publish_odometry(current_time, v, omega)
            
            # Update last values
            self.last_left_pos = left_pos
            self.last_right_pos = right_pos
            self.last_time = current_time
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Error processing joint states: {e}')
    
    def publish_odometry(self, timestamp, v, omega):
        """Publish odometry message and TF transform"""
        # Create Odometry message
        odom = Odometry()
        odom.header.stamp = timestamp.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Set orientation (quaternion from yaw)
        quat = self.quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation = quat
        
        # Set velocity
        odom.twist.twist.linear.x = v
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = omega
        
        self.odom_pub.publish(odom)
        
        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = timestamp.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = quat
        
        self.tf_broadcaster.sendTransform(t)
    
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


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
