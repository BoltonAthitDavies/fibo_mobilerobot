#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get workspace directory for RViz config
    workspace_dir = '/home/ambushee/fibo_mobilerobot'
    rviz_config_file = os.path.join(workspace_dir, 'config', 'icp_mapping_viz.rviz')
    
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
    
    # ICP Mapper Node (NEW)
    icp_mapper_node = Node(
        package='fra532_lab1_part2',
        executable='icp_mapper',
        name='icp_mapper_node',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Trajectory Logger for ICP path
    icp_trajectory_logger_node = Node(
        package='fra532_lab1_part2',
        executable='trajectory_logger',
        name='icp_trajectory_logger_node',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'input_odom_topic': '/icp_odom'},
            {'output_path_topic': '/icp_path'},
            {'log_rate': 5.0}
        ],
        output='screen'
    )
    
    # Static transform publishers for frame relationships
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # RViz2 Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_icp_mapping',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        launch_ekf_arg,
        
        # Conditional EKF node
        ekf_odometry_node,
        
        # Core ICP nodes for Part 2
        icp_odometry_node,
        icp_mapper_node,
        icp_trajectory_logger_node,
        
        # TF publishers
        base_to_laser_tf,
        
        # Visualization
        rviz_node,
    ])