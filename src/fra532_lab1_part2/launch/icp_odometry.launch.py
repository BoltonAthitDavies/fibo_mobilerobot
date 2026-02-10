#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    return LaunchDescription([
        # ICP Odometry Node
        Node(
            package='fra532_lab1_part2',
            executable='icp_odometry.py',
            name='icp_odometry',
            output='screen',
            parameters=[
                {'use_sim_time': True}  # For bag file playback
            ]
        ),
    ])