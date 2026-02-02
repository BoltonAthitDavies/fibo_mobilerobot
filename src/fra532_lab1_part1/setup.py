from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fra532_lab1_part1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ambushee',
    maintainer_email='athit.jake@gmail.com',
    description='EKF-based sensor fusion for wheel odometry and IMU',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ekf_odometry = fra532_lab1_part1.ekf_odometry:main',
            'wheel_odometry = fra532_lab1_part1.wheel_odometry:main',
        ],
    },
)
