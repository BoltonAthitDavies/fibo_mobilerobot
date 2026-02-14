#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
import numpy as np

class MapperDiagnostic(Node):
    def __init__(self):
        super().__init__('mapper_diagnostic')
        
        self.scan_count = 0
        self.odom_count = 0
        self.map_count = 0
        
        # Subscribe to all mapper-related topics
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/icp_odom', self.odom_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/icp_map', self.map_callback, 10)
        
        # Timer to report statistics
        self.timer = self.create_timer(2.0, self.report_stats)
        
        self.get_logger().info('Mapper diagnostic started')
    
    def scan_callback(self, msg):
        self.scan_count += 1
        
    def odom_callback(self, msg):
        self.odom_count += 1
        if self.odom_count == 1:
            self.get_logger().info(f'First ICP odom: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}')
    
    def map_callback(self, msg):
        self.map_count += 1
        if self.map_count == 1:
            # Analyze first map message
            data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
            occupied = np.sum(data == 100)
            free = np.sum(data == 0)  
            unknown = np.sum(data == -1)
            self.get_logger().info(f'First map received: {occupied} occupied, {free} free, {unknown} unknown cells')
    
    def report_stats(self):
        self.get_logger().info(f'Counts: {self.scan_count} scans, {self.odom_count} odom, {self.map_count} maps')

def main():
    rclpy.init()
    node = MapperDiagnostic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()