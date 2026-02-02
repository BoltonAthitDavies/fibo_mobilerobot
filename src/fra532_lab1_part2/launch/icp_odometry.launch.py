from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # EKF Odometry Node (from Part 1)
        Node(
            package='fra532_lab1_part1',
            executable='ekf_odometry',
            name='ekf_odometry',
            output='screen'
        ),
        
        # ICP Odometry Node
        Node(
            package='fra532_lab1_part2',
            executable='icp_odometry',
            name='icp_odometry',
            output='screen'
        ),
    ])
