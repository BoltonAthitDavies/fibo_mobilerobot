#!/usr/bin/env python3
"""
Trajectory Plotter
Subscribes to wheel and EKF odometry and saves trajectory data for plotting
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime


class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Storage for trajectories
        self.wheel_traj = {'x': [], 'y': [], 'time': []}
        self.ekf_traj = {'x': [], 'y': [], 'time': []}
        
        self.start_time = None
        
        # Subscribers
        self.wheel_sub = self.create_subscription(
            Odometry,
            '/wheel_odom',
            self.wheel_callback,
            10
        )
        
        self.ekf_sub = self.create_subscription(
            Odometry,
            '/ekf_odom',
            self.ekf_callback,
            10
        )
        
        self.get_logger().info('Trajectory Plotter Node initialized')
        self.get_logger().info('Recording trajectories... Press Ctrl+C to save and plot')
    
    def wheel_callback(self, msg):
        """Record wheel odometry trajectory"""
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.wheel_traj['x'].append(msg.pose.pose.position.x)
        self.wheel_traj['y'].append(msg.pose.pose.position.y)
        self.wheel_traj['time'].append(current_time)
    
    def ekf_callback(self, msg):
        """Record EKF odometry trajectory"""
        if self.start_time is None:
            self.start_time = self.get_clock().now()
        
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        self.ekf_traj['x'].append(msg.pose.pose.position.x)
        self.ekf_traj['y'].append(msg.pose.pose.position.y)
        self.ekf_traj['time'].append(current_time)
    
    def save_and_plot(self):
        """Save trajectory data and generate plots"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save data to CSV
        if len(self.wheel_traj['x']) > 0:
            wheel_data = np.column_stack([
                self.wheel_traj['time'],
                self.wheel_traj['x'],
                self.wheel_traj['y']
            ])
            np.savetxt(f'wheel_trajectory_{timestamp}.csv', wheel_data,
                      delimiter=',', header='time,x,y', comments='')
            self.get_logger().info(f'Saved wheel trajectory to wheel_trajectory_{timestamp}.csv')
        
        if len(self.ekf_traj['x']) > 0:
            ekf_data = np.column_stack([
                self.ekf_traj['time'],
                self.ekf_traj['x'],
                self.ekf_traj['y']
            ])
            np.savetxt(f'ekf_trajectory_{timestamp}.csv', ekf_data,
                      delimiter=',', header='time,x,y', comments='')
            self.get_logger().info(f'Saved EKF trajectory to ekf_trajectory_{timestamp}.csv')
        
        # Generate plots
        if len(self.wheel_traj['x']) > 0 or len(self.ekf_traj['x']) > 0:
            fig, axes = plt.subplots(1, 2, figsize=(14, 6))
            
            # XY trajectory plot
            if len(self.wheel_traj['x']) > 0:
                axes[0].plot(self.wheel_traj['x'], self.wheel_traj['y'], 
                           'b-', label='Wheel Odometry', linewidth=1.5)
            if len(self.ekf_traj['x']) > 0:
                axes[0].plot(self.ekf_traj['x'], self.ekf_traj['y'], 
                           'r-', label='EKF Odometry', linewidth=1.5, alpha=0.8)
            
            axes[0].set_xlabel('X (m)')
            axes[0].set_ylabel('Y (m)')
            axes[0].set_title('Robot Trajectory')
            axes[0].legend()
            axes[0].grid(True)
            axes[0].axis('equal')
            
            # Time series plot
            if len(self.wheel_traj['x']) > 0:
                wheel_dist = np.sqrt(np.diff(self.wheel_traj['x'])**2 + 
                                    np.diff(self.wheel_traj['y'])**2)
                wheel_cumulative = np.concatenate([[0], np.cumsum(wheel_dist)])
                axes[1].plot(self.wheel_traj['time'], wheel_cumulative, 
                           'b-', label='Wheel Odometry')
            
            if len(self.ekf_traj['x']) > 0:
                ekf_dist = np.sqrt(np.diff(self.ekf_traj['x'])**2 + 
                                  np.diff(self.ekf_traj['y'])**2)
                ekf_cumulative = np.concatenate([[0], np.cumsum(ekf_dist)])
                axes[1].plot(self.ekf_traj['time'], ekf_cumulative, 
                           'r-', label='EKF Odometry')
            
            axes[1].set_xlabel('Time (s)')
            axes[1].set_ylabel('Cumulative Distance (m)')
            axes[1].set_title('Distance Traveled Over Time')
            axes[1].legend()
            axes[1].grid(True)
            
            plt.tight_layout()
            plt.savefig(f'trajectory_comparison_{timestamp}.png', dpi=300)
            self.get_logger().info(f'Saved plot to trajectory_comparison_{timestamp}.png')
            plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down, saving trajectories...')
        node.save_and_plot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
