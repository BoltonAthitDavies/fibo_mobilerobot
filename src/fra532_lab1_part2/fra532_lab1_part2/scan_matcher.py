#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math

class ScanMatcherNode(Node):
    def __init__(self):
        super().__init__('scan_matcher_node')
        
        # Scan matching parameters
        self.previous_scan = None
        self.scan_matching_enabled = True
        
        # Publishers and subscribers
        self.laser_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_corrected', 10)
        
        # Parameters
        self.declare_parameter('max_linear_correction', 0.1)
        self.declare_parameter('max_angular_correction', 0.2)
        
        self.get_logger().info('Scan Matcher Node started')
    
    def laser_callback(self, msg):
        """Process laser scan for scan matching."""
        
        if not self.scan_matching_enabled:
            return
            
        if self.previous_scan is None:
            self.previous_scan = msg
            return
        
        # TODO: Implement scan matching between previous and current scan
        # This is a simplified version - you can enhance it
        
        try:
            # Convert scans to points
            prev_points = self.laser_scan_to_points(self.previous_scan)
            curr_points = self.laser_scan_to_points(msg)
            
            # Simple scan matching
            motion_estimate = self.estimate_motion(prev_points, curr_points)
            
            self.get_logger().debug(
                f'Estimated motion: dx={motion_estimate[0]:.3f}, '
                f'dy={motion_estimate[1]:.3f}, dtheta={motion_estimate[2]:.3f}'
            )
            
            # Store current scan for next iteration
            self.previous_scan = msg
            
        except Exception as e:
            self.get_logger().error(f'Error in scan matching: {str(e)}')
    
    def estimate_motion(self, prev_points, curr_points):
        """
        TODO: Implement your scan matching algorithm here.
        
        Args:
            prev_points: numpy array of previous scan points
            curr_points: numpy array of current scan points
            
        Returns:
            motion: numpy array [dx, dy, dtheta] - estimated motion
        """
        
        # PLACEHOLDER: Simple centroid-based matching
        # You should implement a more sophisticated algorithm
        
        if len(prev_points) == 0 or len(curr_points) == 0:
            return np.array([0.0, 0.0, 0.0])
        
        # Calculate centroids
        prev_centroid = np.mean(prev_points, axis=0)
        curr_centroid = np.mean(curr_points, axis=0)
        
        # Simple translation estimate
        dx = curr_centroid[0] - prev_centroid[0]
        dy = curr_centroid[1] - prev_centroid[1]
        
        # TODO: Estimate rotation (simplified for now)
        dtheta = 0.0
        
        return np.array([dx, dy, dtheta])
    
    def laser_scan_to_points(self, laser_scan):
        """Convert LaserScan message to 2D points."""
        
        points = []
        angle = laser_scan.angle_min
        
        for range_val in laser_scan.ranges:
            if laser_scan.range_min <= range_val <= laser_scan.range_max:
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                points.append([x, y])
            
            angle += laser_scan.angle_increment
        
        return np.array(points)


def main(args=None):
    rclpy.init(args=args)
    
    node = ScanMatcherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()