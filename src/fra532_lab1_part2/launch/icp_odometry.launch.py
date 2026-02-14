#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # ICP Odometry Node
    icp_odometry_node = Node(
        package='fra532_lab1_part2',
        executable='icp_odometry',
        name='icp_odometry_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Static transforms
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.032', '0', '0.172', '0', '0', '0', 'base_link', 'base_scan']
    )

    base_footprint_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_tf',
        arguments=['0', '0', '0.010', '0', '0', '0', 'base_footprint', 'base_link']
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    
    # Add nodes
    ld.add_action(icp_odometry_node)
    ld.add_action(base_footprint_to_base_link_tf)
    ld.add_action(base_to_laser_tf)
    
    return ld