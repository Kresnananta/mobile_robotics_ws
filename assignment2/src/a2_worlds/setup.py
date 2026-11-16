import os
from glob import glob
from setuptools import setup

package_name = 'a2_worlds'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Muhtadin',
    maintainer_email='muhtadin.s@gmail.com',
    description='Assignment 2 (Bug 2): three Gazebo worlds, a launch file and one editable node.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bug2 = a2_worlds.bug2:main',
        ],
    },
)
