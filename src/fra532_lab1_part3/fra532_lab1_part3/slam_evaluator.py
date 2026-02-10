#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt
import numpy as np
from datetime import datetime
import os

class SLAMEvaluatorNode(Node):
    def __init__(self):
        super().__init__('slam_evaluator')
        
        # Create subscribers
        self.path_subscriber = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )
        
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map', 
            self.map_callback,
            10
        )
        
        # Initialize data storage
        self.trajectory_data = []
        self.latest_map = None
        
        # Create output directory
        self.output_dir = os.path.join(os.getcwd(), 'slam_results')
        os.makedirs(self.output_dir, exist_ok=True)
        
        self.get_logger().info('SLAM Evaluator Node started.')
        
    def path_callback(self, msg):
        """Callback for receiving path/trajectory data from SLAM."""
        if len(msg.poses) > 0:
            # Extract pose data
            for pose_stamped in msg.poses:
                x = pose_stamped.pose.position.x
                y = pose_stamped.pose.position.y
                timestamp = pose_stamped.header.stamp.sec + pose_stamped.header.stamp.nanosec * 1e-9
                
                self.trajectory_data.append([timestamp, x, y])
            
            self.get_logger().info(f'Received path with {len(msg.poses)} poses')
            
    def map_callback(self, msg):
        """Callback for receiving map data from SLAM."""
        self.latest_map = msg
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height} at {msg.info.resolution}m resolution')
        
    def save_trajectory(self):
        """Save trajectory data to file."""
        if len(self.trajectory_data) == 0:
            self.get_logger().warning('No trajectory data to save') 
            return
            
        trajectory_array = np.array(self.trajectory_data)
        
        # Save raw data
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        trajectory_file = os.path.join(self.output_dir, f'slam_trajectory_{timestamp}.txt')
        
        np.savetxt(trajectory_file, trajectory_array, 
                   header='timestamp x y', comments='# ')
        
        # Generate plot
        plt.figure(figsize=(12, 8))
        plt.plot(trajectory_array[:, 1], trajectory_array[:, 2], 'b-', linewidth=2, label='SLAM Trajectory')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.title('SLAM Trajectory')
        plt.grid(True, alpha=0.3)
        plt.axis('equal')
        plt.legend()
        
        plot_file = os.path.join(self.output_dir, f'slam_trajectory_plot_{timestamp}.png')
        plt.savefig(plot_file, dpi=300, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f'Trajectory saved to {trajectory_file}')
        self.get_logger().info(f'Trajectory plot saved to {plot_file}')
        
    def save_map(self):
        """Save map data to file."""
        if self.latest_map is None:
            self.get_logger().warning('No map data to save')
            return
            
        # Convert map data
        map_data = np.array(self.latest_map.data).reshape(
            self.latest_map.info.height, self.latest_map.info.width)
        
        # Save map metadata and data  
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        map_file = os.path.join(self.output_dir, f'slam_map_{timestamp}')
        
        # Save as numpy array
        np.save(f'{map_file}.npy', map_data)
        
        # Save map metadata
        with open(f'{map_file}_info.txt', 'w') as f:
            f.write(f"resolution: {self.latest_map.info.resolution}\n")
            f.write(f"width: {self.latest_map.info.width}\n")
            f.write(f"height: {self.latest_map.info.height}\n")
            f.write(f"origin_x: {self.latest_map.info.origin.position.x}\n")
            f.write(f"origin_y: {self.latest_map.info.origin.position.y}\n")
        
        # Generate map visualization
        plt.figure(figsize=(12, 10))
        
        # Create custom colormap: unknown=-1 (gray), free=0 (white), occupied=100 (black)
        map_display = map_data.copy().astype(float)
        map_display[map_data == -1] = 0.5  # Unknown -> gray
        map_display[map_data == 0] = 1.0   # Free -> white  
        map_display[map_data == 100] = 0.0 # Occupied -> black
        
        plt.imshow(map_display, cmap='gray', origin='lower')
        plt.title('SLAM Generated Map')
        plt.xlabel('Grid Cell (X)')
        plt.ylabel('Grid Cell (Y)')
        plt.colorbar(label='Occupancy')
        
        plt.savefig(f'{map_file}_plot.png', dpi=300, bbox_inches='tight')
        plt.close()
        
        self.get_logger().info(f'Map saved to {map_file}.npy')
        self.get_logger().info(f'Map plot saved to {map_file}_plot.png')


def main(args=None):
    rclpy.init(args=args)
    
    node = SLAMEvaluatorNode()
    
    try:
        # Run for a period to collect data
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('Received shutdown signal...')
        
        # Save data before shutdown
        node.save_trajectory()
        node.save_map() 
        
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()