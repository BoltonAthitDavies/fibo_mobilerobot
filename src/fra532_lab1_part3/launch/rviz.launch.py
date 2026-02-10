#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get the current directory (since we're not in a package structure for config)
    current_dir = os.getcwd()
    rviz_config_file = os.path.join(current_dir, 'config', 'lab1_slam.rviz')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=rviz_config_file,
        description='Full path to the RViz config file'
    )
    
    # Variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config = LaunchConfiguration('rviz_config')
    
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
    ld.add_action(rviz_config_arg)
    
    # Add nodes
    ld.add_action(rviz_node)
    
    return ld