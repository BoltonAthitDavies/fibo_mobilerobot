#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get the launch directory
    package_dir = get_package_share_directory('fra532_lab1_part3')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    autostart_arg = DeclareLaunchArgument(
        'autostart', 
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(package_dir, 'config', 'slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )
    
    # Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    slam_params_file = LaunchConfiguration('slam_params_file')
    
    # Nodes launching
    
    # Slam Toolbox Node
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen'
    )

    # Static transform from base_link to laser
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.032', '0', '0.172', '0', '0', '0', 'base_link', 'laser']
    )

    # Static transform from base_footprint to base_link  
    base_footprint_to_base_link_tf = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        name='base_footprint_to_base_link_tf',
        arguments=['0', '0', '0.010', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # Robot State Publisher (if URDF is needed)
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time}],
    #     arguments=[urdf]
    # )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(autostart_arg)
    ld.add_action(slam_params_file_arg)
    
    # Add nodes
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(base_to_laser_tf)
    ld.add_action(base_footprint_to_base_link_tf)
    
    return ld