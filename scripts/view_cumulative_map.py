#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.colors import ListedColormap
import threading
import time

class MapViewer(Node):
    def __init__(self):
        super().__init__('map_viewer')
        
        # Subscribe to the map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        
        self.latest_map = None
        self.map_received = False
        
        # Custom colormap: unknown=gray, free=white, occupied=black
        self.colors = ['gray', 'white', 'black']  # -1, 0, 100
        
        print("Map Viewer started. Waiting for map data...")
        
    def map_callback(self, msg):
        """Callback function when a new map is received"""
        self.latest_map = msg
        self.map_received = True
        
        # Convert map data to numpy array
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        
        # Convert occupancy grid to 2D numpy array
        map_data = np.array(msg.data).reshape(height, width)
        
        # Print map info
        self.get_logger().info(f"Map updated - Size: {width}x{height}, Resolution: {resolution:.3f}m/pixel")
        num_occupied = np.sum(map_data == 100)
        num_free = np.sum(map_data == 0)
        num_unknown = np.sum(map_data == -1)
        
        self.get_logger().info(f"Cells - Occupied: {num_occupied}, Free: {num_free}, Unknown: {num_unknown}")
        
    def get_latest_map_image(self):
        """Convert the latest map to an image array for plotting"""
        if not self.map_received:
            return None
            
        msg = self.latest_map
        width = msg.info.width
        height = msg.info.height
        
        # Convert to numpy array and flip for correct orientation
        map_data = np.array(msg.data).reshape(height, width)
        
        # Convert values: -1 (unknown) -> 0, 0 (free) -> 1, 100 (occupied) -> 2
        display_map = np.zeros_like(map_data)
        display_map[map_data == -1] = 0  # Unknown = gray
        display_map[map_data == 0] = 1   # Free = white
        display_map[map_data == 100] = 2 # Occupied = black
        
        # Flip vertically to match standard map orientation
        return np.flipud(display_map)

def plot_map(node):
    """Set up matplotlib plot for real-time map visualization"""
    
    plt.ion()  # Turn on interactive mode
    fig, ax = plt.subplots(figsize=(10, 8))
    
    # Create custom colormap
    cmap = ListedColormap(['gray', 'white', 'black'])
    
    # Wait for first map
    while not node.map_received:
        rclpy.spin_once(node, timeout_sec=0.1)
        time.sleep(0.1)
    
    print("First map received! Starting visualization...")
    
    # Initial plot
    map_img = node.get_latest_map_image()
    if map_img is not None:
        im = ax.imshow(map_img, cmap=cmap, vmin=0, vmax=2, origin='lower')
        ax.set_title('Cumulative SLAM Map (Real-time)', fontsize=14)
        ax.set_xlabel('X (pixels)')
        ax.set_ylabel('Y (pixels)')
        
        # Add colorbar
        cbar = plt.colorbar(im, ax=ax, ticks=[0, 1, 2])
        cbar.ax.set_yticklabels(['Unknown', 'Free', 'Occupied'])
        
        plt.tight_layout()
        
        try:
            # Animation update function
            def update_plot(frame):
                if node.map_received:
                    new_map = node.get_latest_map_image()
                    if new_map is not None:
                        im.set_array(new_map)
                        ax.set_title(f'Cumulative SLAM Map - Frame {frame}', fontsize=14)
                return [im]
            
            # Create animation that updates every 1000ms
            ani = animation.FuncAnimation(fig, update_plot, interval=1000, blit=False)
            
            plt.show(block=True)
            
        except KeyboardInterrupt:
            print("Map viewer interrupted by user")
            
    return ani

def main():
    rclpy.init()
    
    node = MapViewer()
    
    # Run ROS2 spinning in a separate thread
    def spin_node():
        rclpy.spin(node)
    
    spin_thread = threading.Thread(target=spin_node, daemon=True)
    spin_thread.start()
    
    try:
        # Start the map plotting
        ani = plot_map(node)
        
    except KeyboardInterrupt:
        print("Shutting down map viewer...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()