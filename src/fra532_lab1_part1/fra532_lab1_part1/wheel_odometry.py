#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
import math
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist, Vector3, Point
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf_transformations

class WheelOdometry(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        
        # Robot parameters (Turtlebot3 Burger)
        self.wheel_radius = 0.033  # meters
        self.wheel_separation = 0.160  # meters
        
        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Previous wheel positions
        self.prev_left_pos = None
        self.prev_right_pos = None
        self.prev_time = None
        
        # Create subscriber and publisher
        self.joint_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/wheel_odom',
            10)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.get_logger().info('Wheel Odometry Node Started')

    def joint_state_callback(self, msg):
        """
        Compute wheel odometry from joint states
        Uses differential drive kinematics
        """
        try:
            # Find wheel joint indices
            left_wheel_idx = msg.name.index('wheel_left_joint')
            right_wheel_idx = msg.name.index('wheel_right_joint')
            
            # Get wheel positions (radians)
            left_pos = msg.position[left_wheel_idx]
            right_pos = msg.position[right_wheel_idx]
            
            current_time = self.get_clock().now()
            
            if self.prev_left_pos is not None and self.prev_right_pos is not None:
                # Calculate wheel displacement
                delta_left = left_pos - self.prev_left_pos
                delta_right = right_pos - self.prev_right_pos
                
                # Convert to linear distances
                delta_left_dist = delta_left * self.wheel_radius
                delta_right_dist = delta_right * self.wheel_radius
                
                # Calculate robot motion using differential drive kinematics
                delta_s = (delta_left_dist + delta_right_dist) / 2.0  # forward distance
                delta_theta = (delta_right_dist - delta_left_dist) / self.wheel_separation  # rotation
                
                # Update pose using Runge-Kutta integration
                if abs(delta_theta) < 1e-6:
                    # Straight line motion
                    delta_x = delta_s * math.cos(self.theta)
                    delta_y = delta_s * math.sin(self.theta)
                else:
                    # Curved motion
                    radius = delta_s / delta_theta
                    delta_x = radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
                    delta_y = radius * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
                
                # Update pose
                self.x += delta_x
                self.y += delta_y
                self.theta += delta_theta
                
                # Normalize angle to [-pi, pi]
                self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
                
                # Calculate velocities
                dt = (current_time - self.prev_time).nanoseconds / 1e9
                if dt > 0:
                    linear_vel = delta_s / dt
                    angular_vel = delta_theta / dt
                else:
                    linear_vel = 0.0
                    angular_vel = 0.0
                
                # Publish odometry
                self.publish_odometry(current_time, linear_vel, angular_vel)
                
                # Publish TF transform
                self.publish_tf_transform(current_time)
            
            # Store current values for next iteration
            self.prev_left_pos = left_pos
            self.prev_right_pos = right_pos
            self.prev_time = current_time
            
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'Could not find wheel joints: {e}')

    def publish_odometry(self, timestamp, linear_vel, angular_vel):
        """Publish wheel odometry message"""
        odom_msg = Odometry()
        odom_msg.header.stamp = timestamp.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Position
        odom_msg.pose.pose.position = Point(x=self.x, y=self.y, z=0.0)
        
        # Orientation (quaternion from yaw)
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
        
        # Covariance (simple diagonal matrix)
        # Position covariance
        odom_msg.pose.covariance[0] = 0.1   # x
        odom_msg.pose.covariance[7] = 0.1   # y  
        odom_msg.pose.covariance[35] = 0.05 # yaw
        
        # Velocity covariance
        odom_msg.twist.covariance[0] = 0.1   # vx
        odom_msg.twist.covariance[35] = 0.05 # vyaw
        
        self.odom_publisher.publish(odom_msg)

    def publish_tf_transform(self, timestamp):
        """Publish TF transform from odom to base_link"""
        transform = TransformStamped()
        transform.header.stamp = timestamp.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        
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
    
    wheel_odometry_node = WheelOdometry()
    
    try:
        rclpy.spin(wheel_odometry_node)
    except KeyboardInterrupt:
        pass
    
    wheel_odometry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()