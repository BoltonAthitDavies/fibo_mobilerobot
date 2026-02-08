#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Wheel Odometry Node
        Node(
            package='fra532_lab1_part1',
            executable='wheel_odometry.py',
            name='wheel_odometry',
            output='screen',
            parameters=[]
        ),
        
        # EKF Odometry Node  
        Node(
            package='fra532_lab1_part1',
            executable='ekf_odometry.py',
            name='ekf_odometry',
            output='screen',
            parameters=[]
        ),
    ])