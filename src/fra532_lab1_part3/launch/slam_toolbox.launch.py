#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('fra532_lab1_part3')
    
    # Path to slam_toolbox config file
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    
    return LaunchDescription([
        # SLAM Toolbox Node
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_params_file],
            arguments=['--ros-args', '--log-level', 'info']
        ),
    ])