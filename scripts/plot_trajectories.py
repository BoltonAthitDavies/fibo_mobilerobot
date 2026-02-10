#!/usr/bin/env python3
"""
Trajectory Plotting Script for FRA532 Lab1
Reads ROS2 bag files and plots trajectory comparisons between different odometry methods
"""

import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
import sqlite3
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
from rosidl_runtime_py.utilities import get_message
import tf_transformations
from nav_msgs.msg import Odometry

class TrajectoryPlotter:
    """Plots and analyzes trajectory data from multiple odometry sources"""
    
    def __init__(self):
        self.trajectories = {}
        self.colors = {
            '/wheel_odom': 'red',
            '/ekf_odom': 'blue', 
            '/icp_odom': 'green',
            '/slam_toolbox/pose': 'purple'
        }
        self.labels = {
            '/wheel_odom': 'Wheel Odometry',
            '/ekf_odom': 'EKF Odometry',
            '/icp_odom': 'ICP Odometry',
            '/slam_toolbox/pose': 'SLAM Pose'
        }
        
    def read_bag_file(self, bag_path):
        """Read trajectory data from ROS2 bag file"""
        print(f"Reading bag file: {bag_path}")
        
        # Initialize bag reader
        storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr')
        
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)
        
        # Get topic metadata
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        print("Available topics:")
        for topic_name in type_map:
            print(f"  {topic_name}: {type_map[topic_name]}")
        
        # Read messages
        while reader.has_next():
            (topic, data, timestamp) = reader.read_next()
            
            # Only process odometry topics
            if topic not in ['/wheel_odom', '/ekf_odom', '/icp_odom'] or topic not in type_map:
                continue
            
            # Deserialize message
            msg_type = get_message(type_map[topic])
            msg = deserialize_message(data, msg_type)
            
            # Extract pose data
            if hasattr(msg, 'pose') and hasattr(msg.pose, 'pose'):
                pose_data = self.extract_pose_from_odometry(msg, timestamp)
                
                # Store trajectory data
                if topic not in self.trajectories:
                    self.trajectories[topic] = []
                self.trajectories[topic].append(pose_data)
        
        reader.close()
        
        # Convert lists to numpy arrays and sort by time
        for topic in self.trajectories:
            data = np.array(self.trajectories[topic])
            # Sort by timestamp
            sorted_indices = np.argsort(data[:, 0])
            self.trajectories[topic] = data[sorted_indices]
            
            print(f"{topic}: {len(self.trajectories[topic])} messages")
    
    def extract_pose_from_odometry(self, odom_msg, timestamp):
        """Extract [timestamp, x, y, theta, vx, vy, omega] from odometry message"""
        
        # Position
        x = odom_msg.pose.pose.position.x
        y = odom_msg.pose.pose.position.y
        
        # Orientation (quaternion to yaw)
        quat = odom_msg.pose.pose.orientation
        _, _, theta = tf_transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        
        # Velocities
        vx = odom_msg.twist.twist.linear.x if hasattr(odom_msg.twist.twist.linear, 'x') else 0.0
        vy = odom_msg.twist.twist.linear.y if hasattr(odom_msg.twist.twist.linear, 'y') else 0.0
        omega = odom_msg.twist.twist.angular.z if hasattr(odom_msg.twist.twist.angular, 'z') else 0.0
        
        return [timestamp, x, y, theta, vx, vy, omega]
    
    def plot_trajectories_2d(self, save_path=None):
        """Plot 2D trajectory comparison"""
        
        plt.figure(figsize=(12, 10))
        
        # Plot trajectories
        for topic, data in self.trajectories.items():
            if len(data) == 0:
                continue
                
            x_positions = data[:, 1]  # x coordinates
            y_positions = data[:, 2]  # y coordinates
            
            plt.plot(x_positions, y_positions, 
                    color=self.colors.get(topic, 'black'),
                    label=self.labels.get(topic, topic),
                    linewidth=2, alpha=0.8)
            
            # Mark start and end points
            plt.plot(x_positions[0], y_positions[0], 'o', 
                    color=self.colors.get(topic, 'black'), 
                    markersize=8, label=f'{self.labels.get(topic, topic)} Start')
            plt.plot(x_positions[-1], y_positions[-1], 's', 
                    color=self.colors.get(topic, 'black'), 
                    markersize=8, label=f'{self.labels.get(topic, topic)} End')
        
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Trajectory Comparison - FRA532 Lab1')
        plt.legend()
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"2D trajectory plot saved to: {save_path}")
        
        plt.tight_layout()
        plt.show()
    
    def plot_position_vs_time(self, save_path=None):
        """Plot position components vs time"""
        
        fig, axes = plt.subplots(3, 1, figsize=(14, 10))
        
        for topic, data in self.trajectories.items():
            if len(data) == 0:
                continue
                
            timestamps = (data[:, 0] - data[0, 0]) / 1e9  # Convert to seconds from start
            x_positions = data[:, 1]
            y_positions = data[:, 2]
            theta_positions = data[:, 3]
            
            color = self.colors.get(topic, 'black')
            label = self.labels.get(topic, topic)
            
            axes[0].plot(timestamps, x_positions, color=color, label=label, linewidth=2, alpha=0.8)
            axes[1].plot(timestamps, y_positions, color=color, label=label, linewidth=2, alpha=0.8)
            axes[2].plot(timestamps, theta_positions, color=color, label=label, linewidth=2, alpha=0.8)
        
        axes[0].set_ylabel('X Position (m)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        
        axes[1].set_ylabel('Y Position (m)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        axes[2].set_xlabel('Time (s)')
        axes[2].set_ylabel('Heading (rad)')
        axes[2].legend()
        axes[2].grid(True, alpha=0.3)
        
        plt.suptitle('Position vs Time - FRA532 Lab1')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Position vs time plot saved to: {save_path}")
        
        plt.tight_layout()
        plt.show()
    
    def plot_velocity_profiles(self, save_path=None):
        """Plot velocity profiles"""
        
        fig, axes = plt.subplots(2, 1, figsize=(14, 8))
        
        for topic, data in self.trajectories.items():
            if len(data) == 0:
                continue
                
            timestamps = (data[:, 0] - data[0, 0]) / 1e9  # Convert to seconds
            vx = data[:, 4]
            omega = data[:, 6]
            
            color = self.colors.get(topic, 'black')
            label = self.labels.get(topic, topic)
            
            axes[0].plot(timestamps, vx, color=color, label=label, linewidth=2, alpha=0.8)
            axes[1].plot(timestamps, omega, color=color, label=label, linewidth=2, alpha=0.8)
        
        axes[0].set_ylabel('Linear Velocity (m/s)')
        axes[0].legend()
        axes[0].grid(True, alpha=0.3)
        
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Angular Velocity (rad/s)')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        
        plt.suptitle('Velocity Profiles - FRA532 Lab1')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
            print(f"Velocity profiles plot saved to: {save_path}")
        
        plt.tight_layout()
        plt.show()
    
    def compute_trajectory_statistics(self):
        """Compute and print trajectory statistics"""
        
        print("\n=== Trajectory Statistics ===")
        
        for topic, data in self.trajectories.items():
            if len(data) == 0:
                continue
                
            x_pos = data[:, 1]
            y_pos = data[:, 2]
            
            # Basic statistics
            total_distance = np.sum(np.sqrt(np.diff(x_pos)**2 + np.diff(y_pos)**2))
            start_pos = np.array([x_pos[0], y_pos[0]])
            end_pos = np.array([x_pos[-1], y_pos[-1]])
            end_to_end_distance = np.linalg.norm(end_pos - start_pos)
            
            print(f"\n{self.labels.get(topic, topic)}:")
            print(f"  Total distance traveled: {total_distance:.3f} m")
            print(f"  End-to-end distance: {end_to_end_distance:.3f} m")
            print(f"  Final position: ({end_pos[0]:.3f}, {end_pos[1]:.3f}) m")
            
            # Trajectory bounds
            x_range = np.max(x_pos) - np.min(x_pos)
            y_range = np.max(y_pos) - np.min(y_pos)
            print(f"  X range: {x_range:.3f} m")
            print(f"  Y range: {y_range:.3f} m")
    
    def generate_all_plots(self, output_dir='results'):
        """Generate all plots and save to output directory"""
        
        os.makedirs(output_dir, exist_ok=True)
        
        print(f"Generating plots in: {output_dir}")
        
        # Generate plots
        self.plot_trajectories_2d(os.path.join(output_dir, 'trajectories_2d.png'))
        self.plot_position_vs_time(os.path.join(output_dir, 'position_vs_time.png'))
        self.plot_velocity_profiles(os.path.join(output_dir, 'velocity_profiles.png'))
        
        # Print statistics
        self.compute_trajectory_statistics()
        
        print(f"\nAll plots saved to: {output_dir}")


def main():
    parser = argparse.ArgumentParser(description='Plot trajectory comparisons from ROS2 bag file')
    parser.add_argument('bag_path', help='Path to ROS2 bag file (.db3 or directory)')
    parser.add_argument('-o', '--output', default='results', help='Output directory for plots')
    parser.add_argument('--show-only', action='store_true', help='Only show plots, dont save')
    
    args = parser.parse_args()
    
    # Check if bag file exists
    if not os.path.exists(args.bag_path):
        print(f"Error: Bag file not found: {args.bag_path}")
        return 1
    
    # Initialize plotter and read data
    plotter = TrajectoryPlotter()
    
    try:
        plotter.read_bag_file(args.bag_path)
        
        if not plotter.trajectories:
            print("No trajectory data found in bag file!")
            return 1
        
        if args.show_only:
            plotter.plot_trajectories_2d()
            plotter.plot_position_vs_time()  
            plotter.plot_velocity_profiles()
            plotter.compute_trajectory_statistics()
        else:
            plotter.generate_all_plots(args.output)
        
    except Exception as e:
        print(f"Error processing bag file: {e}")
        return 1
    
    return 0


if __name__ == '__main__':
    exit(main())
                    ORDER BY m.timestamp
                """, (topic_name,))
                
                messages = cursor.fetchall()
                
                for msg_data, timestamp in messages:
                    try:
                        msg = deserialize_message(msg_data, msg_type)
                        
                        x = msg.pose.pose.position.x
                        y = msg.pose.pose.position.y
                        time = timestamp / 1e9  # Convert to seconds
                        
                        self.trajectories[odom_key]['x'].append(x)
                        self.trajectories[odom_key]['y'].append(y)
                        self.trajectories[odom_key]['time'].append(time)
                        
                    except Exception as e:
                        print(f"Error processing message: {e}")
                        continue
            
            conn.close()
            
        except Exception as e:
            print(f"Error reading bag file: {e}")
    
    def plot_trajectories(self):
        """Plot all trajectories"""
        plt.figure(figsize=(15, 10))
        
        # Trajectory comparison plot
        plt.subplot(2, 2, 1)
        
        colors = {'wheel_odom': 'red', 'ekf_odom': 'blue', 'icp_odom': 'green'}
        labels = {'wheel_odom': 'Wheel Odometry', 'ekf_odom': 'EKF Odometry', 'icp_odom': 'ICP Odometry'}
        
        for key in self.trajectories:
            if len(self.trajectories[key]['x']) > 0:
                plt.plot(self.trajectories[key]['x'], self.trajectories[key]['y'], 
                        color=colors[key], label=labels[key], linewidth=2)
                
                # Mark start and end points
                plt.plot(self.trajectories[key]['x'][0], self.trajectories[key]['y'][0], 
                        color=colors[key], marker='o', markersize=8)
                plt.plot(self.trajectories[key]['x'][-1], self.trajectories[key]['y'][-1], 
                        color=colors[key], marker='s', markersize=8)
        
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title('Trajectory Comparison')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        # X position over time
        plt.subplot(2, 2, 2)
        for key in self.trajectories:
            if len(self.trajectories[key]['x']) > 0:
                times = np.array(self.trajectories[key]['time'])
                times = times - times[0]  # Start from 0
                plt.plot(times, self.trajectories[key]['x'], 
                        color=colors[key], label=labels[key])
        
        plt.xlabel('Time (s)')
        plt.ylabel('X Position (m)')
        plt.title('X Position vs Time')
        plt.legend()
        plt.grid(True)
        
        # Y position over time
        plt.subplot(2, 2, 3)
        for key in self.trajectories:
            if len(self.trajectories[key]['y']) > 0:
                times = np.array(self.trajectories[key]['time'])
                times = times - times[0]  # Start from 0
                plt.plot(times, self.trajectories[key]['y'], 
                        color=colors[key], label=labels[key])
        
        plt.xlabel('Time (s)')
        plt.ylabel('Y Position (m)')
        plt.title('Y Position vs Time')
        plt.legend()
        plt.grid(True)
        
        # Error analysis (if multiple trajectories available)
        plt.subplot(2, 2, 4)
        
        if len(self.trajectories['icp_odom']['x']) > 0 and len(self.trajectories['wheel_odom']['x']) > 0:
            # Compare ICP vs Wheel odometry error
            icp_x = np.array(self.trajectories['icp_odom']['x'])
            icp_y = np.array(self.trajectories['icp_odom']['y'])
            wheel_x = np.array(self.trajectories['wheel_odom']['x'])
            wheel_y = np.array(self.trajectories['wheel_odom']['y'])
            
            # Interpolate to same length if needed
            min_len = min(len(icp_x), len(wheel_x))
            if min_len > 0:
                error_x = icp_x[:min_len] - wheel_x[:min_len]
                error_y = icp_y[:min_len] - wheel_y[:min_len]
                error_dist = np.sqrt(error_x**2 + error_y**2)
                
                times = np.array(self.trajectories['icp_odom']['time'][:min_len])
                times = times - times[0]
                
                plt.plot(times, error_dist, 'purple', label='ICP vs Wheel Error')
                plt.xlabel('Time (s)')
                plt.ylabel('Position Error (m)')
                plt.title('Odometry Comparison Error')
                plt.legend()
                plt.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def print_statistics(self):
        """Print trajectory statistics"""
        print("\n=== Trajectory Statistics ===")
        
        for key in self.trajectories:
            if len(self.trajectories[key]['x']) > 0:
                x_data = np.array(self.trajectories[key]['x'])
                y_data = np.array(self.trajectories[key]['y'])
                
                total_distance = 0
                for i in range(1, len(x_data)):
                    dx = x_data[i] - x_data[i-1]
                    dy = y_data[i] - y_data[i-1]
                    total_distance += np.sqrt(dx**2 + dy**2)
                
                print(f"\n{key.upper()}:")
                print(f"  Total points: {len(x_data)}")
                print(f"  Start: ({x_data[0]:.3f}, {y_data[0]:.3f})")
                print(f"  End: ({x_data[-1]:.3f}, {y_data[-1]:.3f})")
                print(f"  Total distance: {total_distance:.3f} m")
                print(f"  Final displacement: {np.sqrt((x_data[-1]-x_data[0])**2 + (y_data[-1]-y_data[0])**2):.3f} m")


def main():
    parser = argparse.ArgumentParser(description='Plot odometry trajectories from ROS2 bag')
    parser.add_argument('bag_file', help='Path to ROS2 bag file (.db3)')
    
    args = parser.parse_args()
    
    if not args.bag_file.endswith('.db3'):
        print("Error: Please provide a .db3 bag file")
        return
    
    plotter = TrajectoryPlotter(args.bag_file)
    
    print("Reading bag data...")
    plotter.read_bag_data()
    
    print("Generating plots...")
    plotter.plot_trajectories()
    
    plotter.print_statistics()

if __name__ == '__main__':
    main()