from setuptools import find_packages, setup

package_name = 'fra532_lab1_part2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/icp_odometry.launch.py',
            'launch/icp_with_rviz.launch.py',
            'launch/icp_only.launch.py',
            'launch/icp_with_mapping.launch.py'
        ]),
        ('share/' + package_name + '/config', ['config/icp_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ambushee',
    maintainer_email='athit.jake@gmail.com',
    description='ICP Odometry Refinement - Lab 1 Part 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'icp_odometry = fra532_lab1_part2.icp_odometry:main',
            'trajectory_logger = fra532_lab1_part2.trajectory_logger:main',
            'icp_mapper = fra532_lab1_part2.icp_mapper:main',
        ],
    },
)