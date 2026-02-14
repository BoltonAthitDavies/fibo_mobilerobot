#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get workspace directory for RViz config
    workspace_dir = '/home/ambushee/fibo_mobilerobot'
    rviz_config_file = os.path.join(workspace_dir, 'config', 'icp_odom_viz.rviz')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    launch_ekf_arg = DeclareLaunchArgument(
        'launch_ekf',
        default_value='false',
        description='Launch EKF node from Part 1 (set to true if Part 1 not running)'
    )
    
    # Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_ekf = LaunchConfiguration('launch_ekf')
    
    # EKF Odometry Node from Part 1 (conditional)
    ekf_odometry_node = Node(
        package='fra532_lab1_part1',
        executable='ekf_odometry',
        name='ekf_odometry_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # ICP Odometry Node
    icp_odometry_node = Node(
        package='fra532_lab1_part2',
        executable='icp_odometry',
        name='icp_odometry_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Trajectory Logger for ICP path
    trajectory_logger_node = Node(
        package='fra532_lab1_part2',
        executable='trajectory_logger',
        name='icp_trajectory_logger',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Static transform from map to odom
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    # Static transforms for robot
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
    
    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(launch_ekf_arg)
    
    # Add nodes
    ld.add_action(map_to_odom_tf)
    # Note: EKF node should be launched separately or with launch_ekf:=true
    ld.add_action(icp_odometry_node)
    ld.add_action(trajectory_logger_node)
    ld.add_action(base_footprint_to_base_link_tf)
    ld.add_action(base_to_laser_tf)
    # ld.add_action(rviz_node)
    
    return ld