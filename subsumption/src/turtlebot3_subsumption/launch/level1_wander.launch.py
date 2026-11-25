"""Incremental Launch File — Level 1 (Layer 0 Avoid + Layer 1 Wander).

Adds `wander_node` and inserts `suppressor_safety` between `wander_node`
(lower wire) and `avoid_node` (upper safety override wire) before `/cmd_vel`.

Expected behavior in Gazebo:
  - Robot wanders aimlessly around the arena.
  - Whenever approaching an obstacle (< danger_distance), `avoid_node` overrides
    `wander_node` through `suppressor_safety` to steer clear without collision.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory('turtlebot3_subsumption')
    default_params = os.path.join(pkg_share, 'config', 'params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    record_trajectory = LaunchConfiguration('record_trajectory')
    output_csv = LaunchConfiguration('output_csv')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation /clock topic from Gazebo'
        ),
        DeclareLaunchArgument(
            'record_trajectory',
            default_value='false',
            description='Optionally launch trajectory_logger_node to record CSV telemetry'
        ),
        DeclareLaunchArgument(
            'output_csv',
            default_value='/mobile_robotics_ws/subsumption_level1.csv',
            description='CSV output file path when record_trajectory:=true'
        ),

        Node(
            package='turtlebot3_subsumption',
            executable='avoid_node',
            name='avoid_node',
            output='screen',
            parameters=[
                default_params,
                {
                    'use_sim_time': use_sim_time,
                    'output_topic': '/avoid/cmd_vel',
                },
            ],
        ),

        Node(
            package='turtlebot3_subsumption',
            executable='wander_node',
            name='wander_node',
            output='screen',
            parameters=[
                default_params,
                {
                    'use_sim_time': use_sim_time,
                    'output_topic': '/wander/cmd_vel',
                },
            ],
        ),

        Node(
            package='turtlebot3_subsumption',
            executable='suppressor_node',
            name='suppressor_safety',
            output='screen',
            parameters=[
                default_params,
                {
                    'use_sim_time': use_sim_time,
                    'lower_topic': '/wander/cmd_vel',
                    'upper_topic': '/avoid/cmd_vel',
                    'output_topic': '/cmd_vel',
                },
            ],
        ),

        Node(
            package='turtlebot3_subsumption',
            executable='trajectory_logger_node',
            name='trajectory_logger_node',
            output='screen',
            condition=IfCondition(record_trajectory),
            parameters=[{
                'use_sim_time': use_sim_time,
                'level': 1,
                'output_csv': output_csv,
            }],
        ),
    ])
