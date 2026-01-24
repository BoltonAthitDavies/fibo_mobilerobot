from setuptools import find_packages, setup

package_name = 'fra532_lab1_part1'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/part1_launch.py']),
        ('share/' + package_name + '/config', ['config/robot_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='FRA532 Lab1 Part1: EKF Odometry Fusion',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wheel_odometry_node = fra532_lab1_part1.wheel_odometry_node:main',
            'ekf_odometry_node = fra532_lab1_part1.ekf_odometry_node:main',
            'trajectory_plotter = fra532_lab1_part1.trajectory_plotter:main',
        ],
    },
)
