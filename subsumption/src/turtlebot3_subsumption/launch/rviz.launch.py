"""Launch RViz2 pre-configured for TurtleBot3 Subsumption visualization.

Displays:
  - Grid (odom frame)
  - RobotModel (/robot_description)
  - LaserScan (/scan, Best Effort QoS)
  - Odometry Trail (/odom, live trajectory arrows)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('turtlebot3_subsumption')
    rviz_config = os.path.join(pkg_share, 'config', 'subsumption.rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation /clock from Gazebo'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
