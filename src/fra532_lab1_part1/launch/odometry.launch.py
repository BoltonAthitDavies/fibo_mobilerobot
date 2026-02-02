from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Wheel Odometry Node
        Node(
            package='fra532_lab1_part1',
            executable='wheel_odometry',
            name='wheel_odometry',
            output='screen'
        ),
        
        # EKF Odometry Node
        Node(
            package='fra532_lab1_part1',
            executable='ekf_odometry',
            name='ekf_odometry',
            output='screen'
        ),
    ])
