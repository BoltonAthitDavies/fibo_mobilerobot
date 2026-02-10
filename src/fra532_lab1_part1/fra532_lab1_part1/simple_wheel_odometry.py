#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math
from rclpy.time import Time

class SimpleWheelOdometryNode(Node):
    """
    Simple wheel odometry node for Part 1 comparison.
    This provides basic wheel odometry without sensor fusion.
    """
    
    def __init__(self):
        super().__init__('simple_wheel_odometry_node')
        
        # Robot parameters (TurtleBot3 Burger)
        self.wheel_radius = 0.033  # m  
        self.wheel_separation = 0.160  # m
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous measurements
        self.prev_left_wheel_pos = 0.0
        self.prev_right_wheel_pos = 0.0
        self.prev_time = None
        self.first_measurement = True
        
        # Publishers and subscribers
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        self.odom_pub = self.create_publisher(Odometry, '/wheel_odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Simple Wheel Odometry Node started')
        
    def joint_states_callback(self, msg):
        """Process joint states to compute wheel odometry."""
        
        # Find wheel joints
        left_wheel_idx = None
        right_wheel_idx = None
        
        # Try different naming conventions
        for i, name in enumerate(msg.name):
            if 'left' in name.lower() and 'wheel' in name.lower():
                left_wheel_idx = i
            elif 'right' in name.lower() and 'wheel' in name.lower():
                right_wheel_idx = i
                
        if left_wheel_idx is None or right_wheel_idx is None:
            # Try generic indices
            if len(msg.position) >= 2:
                left_wheel_idx = 0
                right_wheel_idx = 1
            else:
                self.get_logger().warn('Could not find wheel joints')
                return
                
        # Get current wheel positions
        left_wheel_pos = msg.position[left_wheel_idx]
        right_wheel_pos = msg.position[right_wheel_idx]
        
        current_time = Time.from_msg(msg.header.stamp)
        
        if self.first_measurement:
            self.prev_left_wheel_pos = left_wheel_pos
            self.prev_right_wheel_pos = right_wheel_pos  
            self.prev_time = current_time
            self.first_measurement = False
            return
            
        # Calculate wheel displacement
        dt = (current_time - self.prev_time).nanoseconds * 1e-9
        if dt <= 0:
            return
            
        delta_left = left_wheel_pos - self.prev_left_wheel_pos
        delta_right = right_wheel_pos - self.prev_right_wheel_pos
        
        # Convert to linear distances
        left_distance = delta_left * self.wheel_radius
        right_distance = delta_right * self.wheel_radius
        
        # Calculate robot motion
        distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation
        
        # Update robot pose
        if abs(delta_theta) > 1e-6:
            # Curved motion
            radius = distance / delta_theta
            dx = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            dy = radius * (-math.cos(self.theta + delta_theta) + math.cos(self.theta))
        else:
            # Straight motion  
            dx = distance * math.cos(self.theta)
            dy = distance * math.sin(self.theta)
            
        self.x += dx
        self.y += dy
        self.theta += delta_theta
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        linear_vel = distance / dt
        angular_vel = delta_theta / dt
        
        # Publish odometry
        self.publish_odometry(msg.header.stamp, linear_vel, angular_vel)
        
        # Update previous values
        self.prev_left_wheel_pos = left_wheel_pos
        self.prev_right_wheel_pos = right_wheel_pos
        self.prev_time = current_time
        
    def publish_odometry(self, timestamp, linear_vel, angular_vel):
        """Publish odometry message and transform."""
        
        # Create odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_footprint'
        
        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        
        # Orientation  
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Velocity
        odom_msg.twist.twist.linear.x = linear_vel
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.angular.z = angular_vel
        
        # Simple covariance
        odom_msg.pose.covariance[0] = 0.1   # x
        odom_msg.pose.covariance[7] = 0.1   # y
        odom_msg.pose.covariance[35] = 0.2  # theta
        
        self.odom_pub.publish(odom_msg)
        
        # Publish transform  
        tf_msg = TransformStamped()
        tf_msg.header.stamp = timestamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        
        tf_msg.transform.translation.x = self.x
        tf_msg.transform.translation.y = self.y
        tf_msg.transform.translation.z = 0.0
        
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = math.sin(self.theta / 2.0)
        tf_msg.transform.rotation.w = math.cos(self.theta / 2.0)
        
        self.tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleWheelOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()