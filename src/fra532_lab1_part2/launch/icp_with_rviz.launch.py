#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get workspace directory for RViz config
    workspace_dir = '/home/ambushee/fibo_mobilerobot'
    rviz_config_file = os.path.join(workspace_dir, 'config', 'lab1_slam.rviz')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    launch_ekf_arg = DeclareLaunchArgument(
        'launch_ekf',
        default_value='false', 
        description='Launch EKF odometry node from Part 1 (set to false if Part 1 is already running)'
    )
    
    # Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    launch_ekf = LaunchConfiguration('launch_ekf')
    
    # EKF Odometry Node from Part 1 (optional - only launch if Part 1 not running)
    ekf_odometry_node = Node(
        package='fra532_lab1_part1',
        executable='ekf_odometry',
        name='ekf_odometry_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=LaunchConfigurationEquals('launch_ekf', 'true')
    )
    
    # ICP Odometry Refinement Node
    icp_refinement_node = Node(
        package='fra532_lab1_part2',
        executable='icp_localization',
        name='icp_refinement_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Static transform from map to odom (for global reference)
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
    
    # RViz Node for visualization
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
    ld.add_action(map_to_odom_tf)  # Important: map->odom transform for global view
    ld.add_action(ekf_odometry_node)  # Conditional: EKF only if launch_ekf=true
    ld.add_action(icp_refinement_node)  # ICP refinement (always launched)
    ld.add_action(base_footprint_to_base_link_tf)
    ld.add_action(base_to_laser_tf)
    ld.add_action(rviz_node)
    
    return ld