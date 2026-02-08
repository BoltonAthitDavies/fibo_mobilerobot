#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import matplotlib.pyplot as plt
import numpy as np
import argparse

class MapVisualizerNode(Node):
    def __init__(self):
        super().__init__('map_visualizer')
        
        self.map_data = None
        self.map_received = False
        
        # Subscribe to map topic
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        self.get_logger().info('Map Visualizer Node Started - waiting for map...')

    def map_callback(self, msg):
        """Receive and store map data"""
        self.map_data = msg
        self.map_received = True
        self.get_logger().info(f'Received map: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution}')

    def save_map_image(self, filename='slam_map.png'):
        """Save map as image file"""
        if not self.map_received or self.map_data is None:
            self.get_logger().error('No map data received')
            return False
        
        # Extract map parameters
        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        
        # Convert map data to numpy array
        map_array = np.array(self.map_data.data)
        map_2d = map_array.reshape((height, width))
        
        # Create visualization
        plt.figure(figsize=(12, 8))
        
        # Map visualization (flip for correct orientation)
        map_display = np.flipud(map_2d)
        
        # Convert occupancy values to colors
        # -1 (unknown) -> gray, 0 (free) -> white, 100 (occupied) -> black
        colored_map = np.zeros_like(map_display, dtype=float)
        colored_map[map_display == -1] = 0.5  # Unknown -> gray
        colored_map[map_display == 0] = 1.0   # Free -> white  
        colored_map[map_display == 100] = 0.0 # Occupied -> black
        
        plt.imshow(colored_map, cmap='gray', origin='lower')
        
        # Add scale and labels
        extent = [
            self.map_data.info.origin.position.x,
            self.map_data.info.origin.position.x + width * resolution,
            self.map_data.info.origin.position.y,
            self.map_data.info.origin.position.y + height * resolution
        ]
        
        plt.xlabel('X (m)')
        plt.ylabel('Y (m)')
        plt.title(f'SLAM Map - Resolution: {resolution:.3f} m/pixel')
        
        # Add grid
        plt.grid(True, alpha=0.3)
        
        # Add colorbar legend
        from matplotlib.patches import Patch
        legend_elements = [
            Patch(facecolor='black', label='Occupied'),
            Patch(facecolor='white', edgecolor='black', label='Free'),
            Patch(facecolor='gray', label='Unknown')
        ]
        plt.legend(handles=legend_elements, loc='upper right')
        
        # Save the figure
        plt.savefig(filename, dpi=300, bbox_inches='tight')
        plt.show()
        
        self.get_logger().info(f'Map saved as {filename}')
        return True

def main():
    parser = argparse.ArgumentParser(description='Visualize and save SLAM map')
    parser.add_argument('--output', '-o', default='slam_map.png', 
                       help='Output filename for map image')
    parser.add_argument('--timeout', '-t', type=int, default=10,
                       help='Timeout in seconds to wait for map')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    map_viz_node = MapVisualizerNode()
    
    # Wait for map data
    timeout_count = 0
    rate = map_viz_node.create_rate(1)  # 1 Hz
    
    while not map_viz_node.map_received and timeout_count < args.timeout:
        rclpy.spin_once(map_viz_node, timeout_sec=1.0)
        timeout_count += 1
        
        if timeout_count % 2 == 0:
            map_viz_node.get_logger().info(f'Waiting for map... ({timeout_count}/{args.timeout})')
    
    if map_viz_node.map_received:
        # Save map
        success = map_viz_node.save_map_image(args.output)
        if success:
            print(f"Map successfully saved as {args.output}")
        else:
            print("Failed to save map")
    else:
        map_viz_node.get_logger().error(f'No map received within {args.timeout} seconds')
        print("Make sure SLAM is running and publishing to /map topic")
    
    map_viz_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()