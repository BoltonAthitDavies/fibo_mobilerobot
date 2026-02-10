from setuptools import find_packages, setup

package_name = 'fra532_lab1_part1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/ekf_odometry.launch.py',
            'launch/simple_wheel_odometry.launch.py',
            'launch/wheel_odom_with_rviz.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/ekf_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ambushee',
    maintainer_email='athit.jake@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_odometry = fra532_lab1_part1.ekf_odometry:main',
            'simple_wheel_odometry = fra532_lab1_part1.simple_wheel_odometry:main',
            'trajectory_logger = fra532_lab1_part1.trajectory_logger:main',
        ],
    },
)
