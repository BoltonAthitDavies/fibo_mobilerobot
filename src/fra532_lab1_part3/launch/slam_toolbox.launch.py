from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get the config directory
    config_dir = os.path.join(
        get_package_share_directory('fra532_lab1_part3'),
        'config'
    )
    
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'slam_toolbox_params.yaml'),
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        slam_toolbox_node
    ])
