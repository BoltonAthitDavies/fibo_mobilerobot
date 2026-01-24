#!/usr/bin/env python3
"""
Launch file for Part 1: EKF Odometry Fusion
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('fra532_lab1_part1')
    config_file = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
    
    return LaunchDescription([
        # Wheel Odometry Node
        Node(
            package='fra532_lab1_part1',
            executable='wheel_odometry_node',
            name='wheel_odometry_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # EKF Odometry Node
        Node(
            package='fra532_lab1_part1',
            executable='ekf_odometry_node',
            name='ekf_odometry_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # Trajectory Plotter Node
        Node(
            package='fra532_lab1_part1',
            executable='trajectory_plotter',
            name='trajectory_plotter',
            output='screen'
        ),
    ])
