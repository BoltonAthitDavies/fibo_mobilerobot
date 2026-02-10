#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import yaml
import sys
import os
from datetime import datetime

class MapSaver(Node):
    def __init__(self, map_name='map'):
        super().__init__('map_saver')
        self.map_name = map_name
        self.map_received = False
        
        # Subscribe to map topic
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        self.get_logger().info(f'Map saver started. Waiting for map on /map topic...')
        
    def map_callback(self, msg):
        if not self.map_received:
            self.save_map(msg)
            self.map_received = True
            self.get_logger().info('Map saved successfully. Shutting down...')
            rclpy.shutdown()
            
    def save_map(self, map_msg):
        """Save the map to files."""
        
        # Create output directory
        output_dir = 'maps'
        os.makedirs(output_dir, exist_ok=True)
        
        # Add timestamp to filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S") 
        map_file = os.path.join(output_dir, f'{self.map_name}_{timestamp}')
        
        # Save map metadata as YAML
        map_yaml = {
            'image': f'{os.path.basename(map_file)}.pgm',
            'resolution': float(map_msg.info.resolution),
            'origin': [
                float(map_msg.info.origin.position.x),
                float(map_msg.info.origin.position.y),
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        
        with open(f'{map_file}.yaml', 'w') as f:
            yaml.dump(map_yaml, f, default_flow_style=False)
            
        # Save map data as PGM image
        width = map_msg.info.width
        height = map_msg.info.height
        
        with open(f'{map_file}.pgm', 'wb') as f:
            # PGM header
            f.write(f'P5\n{width} {height}\n255\n'.encode())
            
            # Convert occupancy grid to grayscale image
            for y in range(height - 1, -1, -1):  # PGM origin is top-left
                for x in range(width):
                    index = y * width + x
                    value = map_msg.data[index]
                    
                    if value == -1:  # Unknown
                        pixel = 205  # Gray
                    elif value == 0:  # Free
                        pixel = 254  # White  
                    else:  # Occupied (value 100)
                        pixel = 0    # Black
                        
                    f.write(bytes([pixel]))
        
        self.get_logger().info(f'Map saved as {map_file}.yaml and {map_file}.pgm')


def main(args=None):
    rclpy.init(args=args)
    
    map_name = 'slam_map'
    if len(sys.argv) > 1:
        map_name = sys.argv[1]
    
    map_saver = MapSaver(map_name)
    
    try:
        rclpy.spin(map_saver)
    except KeyboardInterrupt:
        pass
    finally:
        map_saver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()