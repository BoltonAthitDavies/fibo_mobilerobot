#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
import math
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ICPMapperNode(Node):
    def __init__(self):
        super().__init__('icp_mapper_node')
        
        # Map parameters
        self.map_resolution = 0.05  # meters per pixel
        self.map_width = 800   # pixels
        self.map_height = 800  # pixels
        self.map_origin_x = -20.0  # meters
        self.map_origin_y = -20.0  # meters
        
        # Initialize occupancy grid
        self.occupancy_grid = np.ones((self.map_height, self.map_width)) * -1  # Unknown = -1
        self.hit_count = np.zeros((self.map_height, self.map_width))
        self.miss_count = np.zeros((self.map_height, self.map_width))
        
        # Robot state (store with timestamp)
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        self.pose_available = False
        self.last_pose_time = None
        
        # Laser parameters
        self.laser_max_range = 3.5
        self.laser_min_range = 0.12
        
        # QoS profiles
        laser_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.laser_sub = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.laser_callback, 
            laser_qos
        )
        
        self.icp_odom_sub = self.create_subscription(
            Odometry, 
            '/icp_odom', 
            self.icp_odom_callback, 
            10
        )
        
        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/icp_map', 10)
        
        # Timer to publish map periodically
        self.map_timer = self.create_timer(1.0, self.publish_map)
        
        # Debug counters
        self.scan_count = 0
        self.map_updates = 0
        
        self.get_logger().info('ICP Mapper Node started - waiting for ICP odometry and laser data')
        
    def icp_odom_callback(self, msg):
        """Update robot pose from ICP odometry."""
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        
        # Convert quaternion to euler angle
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.robot_theta = 2 * math.atan2(qz, qw)
        
        # Store timestamp for synchronization
        self.last_pose_time = msg.header.stamp
        
        if not self.pose_available:
            self.get_logger().info(f'ICP pose available for mapping: x={self.robot_x:.3f}, y={self.robot_y:.3f}, θ={self.robot_theta:.3f}')
            self.pose_available = True
        
        # Debug: log every 50 pose updates
        if hasattr(self, '_pose_count'):
            self._pose_count += 1
        else:
            self._pose_count = 1
            
        if self._pose_count % 50 == 0:
            self.get_logger().info(f'Received {self._pose_count} pose updates. Current: [{self.robot_x:.2f}, {self.robot_y:.2f}, {self.robot_theta:.2f}]')
        
    def laser_callback(self, msg):
        """Process laser scan to update occupancy grid."""
        self.scan_count += 1
        
        if not self.pose_available:
            if self.scan_count % 20 == 0:
                self.get_logger().warn(f'Received {self.scan_count} scans but no ICP pose data yet')
            return
        
        # Temporarily disable strict timing check for debugging
        # Check if pose is recent enough (within 0.5 seconds - more relaxed)
        if self.last_pose_time is not None:
            scan_time = msg.header.stamp
            pose_time = self.last_pose_time
            
            time_diff = abs((scan_time.sec + scan_time.nanosec * 1e-9) - 
                          (pose_time.sec + pose_time.nanosec * 1e-9))
            
            if time_diff > 0.5:  # Much more relaxed timing
                if self.scan_count % 20 == 0:
                    self.get_logger().warn(f'Pose too old for scan: {time_diff:.3f}s, scan_count: {self.scan_count}')
                # Continue processing anyway for debugging
                # return
        
        # Convert laser scan to rays and update map
        self.update_occupancy_grid(msg)
        
        # Debug: log every 10 scans
        if self.scan_count % 10 == 0:
            self.get_logger().info(f'Processed {self.scan_count} scans, {self.map_updates} map updates')
        
    def update_occupancy_grid(self, laser_scan):
        """Update occupancy grid using laser scan data."""
        angle = laser_scan.angle_min
        
        # Store current robot pose for this scan to avoid changes during processing
        robot_x = self.robot_x
        robot_y = self.robot_y  
        robot_theta = self.robot_theta
        
        cos_theta = math.cos(robot_theta)
        sin_theta = math.sin(robot_theta)
        
        valid_points = 0
        
        for range_val in laser_scan.ranges:
            # Skip invalid ranges
            if not (self.laser_min_range <= range_val <= self.laser_max_range):
                angle += laser_scan.angle_increment
                continue
                
            # Calculate end point of laser ray in robot frame
            laser_x_robot = range_val * math.cos(angle)
            laser_y_robot = range_val * math.sin(angle)
            
            # Transform to global frame using 2D rotation matrix
            laser_x_global = (cos_theta * laser_x_robot - sin_theta * laser_y_robot) + robot_x
            laser_y_global = (sin_theta * laser_x_robot + cos_theta * laser_y_robot) + robot_y
            
            # Ray tracing from robot position to laser hit point
            self.trace_ray(robot_x, robot_y, laser_x_global, laser_y_global)
            
            valid_points += 1
            angle += laser_scan.angle_increment
            
        if valid_points > 0:
            self.map_updates += 1
            self.get_logger().debug(f'Processed {valid_points} laser points at pose [{robot_x:.3f}, {robot_y:.3f}, {robot_theta:.3f}]')
    
    def trace_ray(self, x0, y0, x1, y1):
        """Trace ray from (x0,y0) to (x1,y1) and update occupancy grid."""
        # Convert world coordinates to grid coordinates
        start_col = int((x0 - self.map_origin_x) / self.map_resolution)
        start_row = int((y0 - self.map_origin_y) / self.map_resolution)
        end_col = int((x1 - self.map_origin_x) / self.map_resolution)
        end_row = int((y1 - self.map_origin_y) / self.map_resolution)
        
        # Bresenham's line algorithm for ray tracing
        points = self.bresenham_line(start_col, start_row, end_col, end_row)
        
        # Mark all points along ray as free space (except the last point)
        for i, (col, row) in enumerate(points):
            if self.is_valid_grid_point(col, row):
                if i == len(points) - 1:
                    # Last point is an obstacle (hit)
                    self.hit_count[row, col] += 1
                else:
                    # Points along ray are free space (miss)
                    self.miss_count[row, col] += 1
                
                # Update occupancy probability
                self.update_cell_probability(row, col)
    
    def bresenham_line(self, x0, y0, x1, y1):
        """Bresenham's line algorithm to get all points between two points."""
        points = []
        
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy
        
        x, y = x0, y0
        
        while True:
            points.append((x, y))
            
            if x == x1 and y == y1:
                break
                
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
                
        return points
    
    def is_valid_grid_point(self, col, row):
        """Check if grid point is within bounds."""
        return 0 <= col < self.map_width and 0 <= row < self.map_height
    
    def update_cell_probability(self, row, col):
        """Update cell occupancy probability using hit/miss counts."""
        hits = self.hit_count[row, col]
        misses = self.miss_count[row, col]
        total = hits + misses
        
        if total == 0:
            self.occupancy_grid[row, col] = -1  # Unknown
        else:
            # Calculate probability (0-100)
            prob = (hits / total) * 100
            
            # Apply more conservative thresholds for cleaner mapping
            if prob > 80:  # More confident about obstacles
                self.occupancy_grid[row, col] = 100  # Occupied
            elif prob < 20:  # More confident about free space
                self.occupancy_grid[row, col] = 0    # Free
            else:
                self.occupancy_grid[row, col] = -1   # Unknown
    
    def publish_map(self):
        """Publish the current occupancy grid."""
        if not self.pose_available:
            return
            
        # Count different cell types for debugging
        occupied_cells = np.sum(self.occupancy_grid == 100)
        free_cells = np.sum(self.occupancy_grid == 0)
        unknown_cells = np.sum(self.occupancy_grid == -1)
        
        map_msg = OccupancyGrid()
        
        # Header
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'odom'
        
        # Map metadata
        map_msg.info = MapMetaData()
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        
        # Map origin
        map_msg.info.origin = Pose()
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
        
        # Map data (flatten and convert to int8)
        map_data = self.occupancy_grid.astype(np.int8).flatten().tolist()
        map_msg.data = map_data
        
        self.map_pub.publish(map_msg)
        
        # Debug info every 10 publishes
        if hasattr(self, '_pub_count'):
            self._pub_count += 1
        else:
            self._pub_count = 1
            
        if self._pub_count % 10 == 0:
            self.get_logger().info(f'Published map #{self._pub_count}: {occupied_cells} occupied, {free_cells} free, {unknown_cells} unknown cells')


def main(args=None):
    rclpy.init(args=args)
    
    node = ICPMapperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()