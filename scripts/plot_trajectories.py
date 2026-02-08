#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
import numpy as np
import sqlite3
import argparse
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

class TrajectoryPlotter:
    def __init__(self, bag_file):
        self.bag_file = bag_file
        self.trajectories = {
            'wheel_odom': {'x': [], 'y': [], 'time': []},
            'ekf_odom': {'x': [], 'y': [], 'time': []},
            'icp_odom': {'x': [], 'y': [], 'time': []}
        }
        
    def read_bag_data(self):
        """Read odometry data from ROS2 bag file"""
        try:
            conn = sqlite3.connect(self.bag_file)
            cursor = conn.cursor()
            
            # Get message type info
            cursor.execute("SELECT name, type FROM topics")
            topics = cursor.fetchall()
            
            topic_types = {}
            for name, msg_type in topics:
                topic_types[name] = msg_type
            
            # Read messages
            for topic_name, odom_key in [('/wheel_odom', 'wheel_odom'),
                                       ('/ekf_odom', 'ekf_odom'),
                                       ('/icp_odom', 'icp_odom')]:
                
                if topic_name not in topic_types:
                    print(f"Warning: Topic {topic_name} not found in bag")
                    continue
                
                msg_type = get_message(topic_types[topic_name])
                
                cursor.execute("""
                    SELECT m.data, m.timestamp 
                    FROM messages m
                    JOIN topics t ON m.topic_id = t.id
                    WHERE t.name = ?
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