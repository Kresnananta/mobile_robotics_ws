from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'turtlebot3_subsumption'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))
            + glob(os.path.join('config', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Anak Agung Ngurah Agung Kresna Ananta',
    maintainer_email='kresna@its.ac.id',
    description='Modular ROS 2 Subsumption Architecture for TurtleBot3 Gazebo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inhibitor_node = turtlebot3_subsumption.inhibitor_node:main',
            'suppressor_node = turtlebot3_subsumption.suppressor_node:main',
            'avoid_node = turtlebot3_subsumption.avoid_node:main',
            'wander_node = turtlebot3_subsumption.wander_node:main',
            'explore_node = turtlebot3_subsumption.explore_node:main',
            'trajectory_logger_node = turtlebot3_subsumption.trajectory_logger_node:main',
        ],
    },
)
