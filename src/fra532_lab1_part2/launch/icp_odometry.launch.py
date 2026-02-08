#!/usr/bin/env python3

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
            parameters=[]
        ),
    ])