#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get the package directory and current working directory
    try:
        package_dir = get_package_share_directory('fra532_lab1_part3')
        slam_params_file_default = os.path.join(package_dir, 'config', 'slam_params.yaml')
    except:
        # Fallback if package not found (running from source)
        current_dir = os.getcwd()
        slam_params_file_default = os.path.join(current_dir, 'src', 'fra532_lab1_part3', 'config', 'slam_params.yaml')
        package_dir = os.path.join(current_dir, 'src', 'fra532_lab1_part3')
    
    # RViz config file
    current_dir = os.getcwd()  
    rviz_config_file = os.path.join(current_dir, 'config', 'lab1_slam.rviz')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=slam_params_file_default,
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_file,
        description='Full path to the RViz config file'
    )
    
    # Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Nodes
    
    # SLAM Toolbox Node
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

    # Wheel Odometry Node  
    wheel_odometry_node = Node(
        package='fra532_lab1_part3',
        executable='wheel_odometry',
        name='wheel_odometry',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Static transform from base_link to base_scan (laser)
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0.032', '0', '0.172', '0', '0', '0', 'base_link', 'base_scan']
    )

    # Static transform from base_footprint to base_link  
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
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(rviz_config_arg)
    
    # Add nodes
    ld.add_action(wheel_odometry_node)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(base_to_laser_tf)
    ld.add_action(base_footprint_to_base_link_tf)
    ld.add_action(rviz_node)
    
    return ld