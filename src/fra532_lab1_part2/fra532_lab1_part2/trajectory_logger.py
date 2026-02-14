#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class TrajectoryLogger(Node):
    def __init__(self):
        super().__init__('trajectory_logger')
        
        # Path storage
        self.path = Path()
        self.path.header.frame_id = 'map'
        
        # Subscriber to ICP odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/icp_odom',
            self.odom_callback,
            10
        )
        
        # Publisher for path
        self.path_pub = self.create_publisher(Path, '/icp_trajectory', 10)
        
        # Timer to publish path
        self.timer = self.create_timer(0.1, self.publish_path)  # 10 Hz
        
        self.get_logger().info('Trajectory Logger started')
    
    def odom_callback(self, msg):
        """Store odometry pose in path."""
        pose_stamped = PoseStamped()
        pose_stamped.header = msg.header
        pose_stamped.pose = msg.pose.pose
        
        self.path.poses.append(pose_stamped)
        
        # Limit path length to avoid memory issues
        if len(self.path.poses) > 1000:
            self.path.poses = self.path.poses[-800:]  # Keep last 800 poses
    
    def publish_path(self):
        """Publish the trajectory path."""
        if len(self.path.poses) > 0:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.path)


def main(args=None):
    rclpy.init(args=args)
    
    node = TrajectoryLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()