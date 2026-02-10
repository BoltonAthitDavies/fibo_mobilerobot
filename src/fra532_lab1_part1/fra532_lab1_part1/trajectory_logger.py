#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

class TrajectoryLoggerNode(Node):
    """
    Template node for logging trajectory data from Part 1 odometry methods.
    """
    
    def __init__(self):
        super().__init__('trajectory_logger')
        
        # Create subscribers for different odometry sources
        self.wheel_odom_sub = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.wheel_odom_callback,
            10
        )
        
        self.ekf_odom_sub = self.create_subscription(
            Odometry,
            '/ekf_odom', 
            self.ekf_odom_callback,
            10
        )
        
        # Data storage
        self.wheel_trajectory = []
        self.ekf_trajectory = []
        
        # Output directory
        self.output_dir = 'part1_results'
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info('Trajectory Logger Node started')
        
    def wheel_odom_callback(self, msg):
        """Log wheel odometry data."""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.wheel_trajectory.append([timestamp, x, y])
        
    def ekf_odom_callback(self, msg):
        """Log EKF odometry data."""
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        
        self.ekf_trajectory.append([timestamp, x, y])
        
    def save_trajectories(self):
        """Save trajectory data to files."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        if self.wheel_trajectory:
            wheel_data = np.array(self.wheel_trajectory)
            wheel_file = os.path.join(self.output_dir, f'wheel_trajectory_{timestamp}.txt')
            np.savetxt(wheel_file, wheel_data, header='timestamp x y', comments='# ')
            self.get_logger().info(f'Wheel trajectory saved: {wheel_file}')
            
        if self.ekf_trajectory:
            ekf_data = np.array(self.ekf_trajectory)
            ekf_file = os.path.join(self.output_dir, f'ekf_trajectory_{timestamp}.txt')
            np.savetxt(ekf_file, ekf_data, header='timestamp x y', comments='# ')  
            self.get_logger().info(f'EKF trajectory saved: {ekf_file}')


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryLoggerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_trajectories()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()