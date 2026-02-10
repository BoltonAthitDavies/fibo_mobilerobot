from setuptools import find_packages, setup

package_name = 'fra532_lab1_part3'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/slam.launch.py']),
        ('share/' + package_name + '/config', ['config/slam_params.yaml']),
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
            'slam_evaluator = fra532_lab1_part3.slam_evaluator:main',
            'map_saver = fra532_lab1_part3.map_saver:main',
        ],
    },
)
