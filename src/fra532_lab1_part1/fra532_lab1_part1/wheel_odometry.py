#!/usr/bin/env python3
"""
Wheel Odometry Node
Computes odometry from wheel encoder data only
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
import tf2_ros
import math


class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        
        # Robot parameters (Turtlebot3 Burger)
        self.wheel_radius = 0.033  # meters
        self.wheel_separation = 0.160  # meters
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous joint states
        self.prev_joint_positions = None
        self.prev_time = None
        
        # Subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.get_logger().info('Wheel Odometry node initialized')
    
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
    
    def joint_callback(self, msg):
        """Process joint states to compute wheel odometry"""
        current_time = self.get_clock().now()
        
        if self.prev_joint_positions is None:
            self.prev_joint_positions = msg.position
            self.prev_time = current_time
            return
        
        # Calculate dt
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        
        if dt <= 0:
            return
        
        # Get wheel velocities
        # Assuming wheel_left_joint is index 0 and wheel_right_joint is index 1
        left_vel = msg.velocity[0] if len(msg.velocity) > 0 else 0.0
        right_vel = msg.velocity[1] if len(msg.velocity) > 1 else 0.0
        
        # Compute linear and angular velocities
        v = self.wheel_radius * (right_vel + left_vel) / 2.0
        omega = self.wheel_radius * (right_vel - left_vel) / self.wheel_separation
        
        # Update pose
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += omega * dt
        
        # Normalize angle
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Publish odometry
        self.publish_odometry(current_time, v, omega)
        
        # Update previous values
        self.prev_joint_positions = msg.position
        self.prev_time = current_time
    
    def publish_odometry(self, current_time, v, omega):
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
        
        # Set velocities
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        
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
    node = WheelOdometry()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
