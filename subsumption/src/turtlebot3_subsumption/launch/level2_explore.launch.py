"""Incremental Launch File — Level 2 (Full 3-Layer Subsumption Architecture).

Stacks all three competence layers using reusable Inhibitor and Suppressor nodes:
  - Layer 0: `avoid_node` (survival obstacle avoidance)
  - Layer 1: `wander_node` (stochastic wandering)
  - Layer 2: `explore_node` (LiDAR open-space / corridor exploration)
  - Coordinators:
      * `inhibitor_wander`: blocks `/wander/cmd_vel` when `/explore/inhibit` fires
      * `suppressor_l1_l2`: replaces inhibited wander with `/explore/cmd_vel`
      * `suppressor_safety`: ensures `/avoid/cmd_vel` always overrides `/cmd_vel`
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
            default_value='/mobile_robotics_ws/subsumption_level2.csv',
            description='CSV output file path when record_trajectory:=true'
        ),

        # --- Layer 0: Avoid Contact ---
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

        # --- Layer 1: Wander ---
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

        # --- Layer 2: Explore ---
        Node(
            package='turtlebot3_subsumption',
            executable='explore_node',
            name='explore_node',
            output='screen',
            parameters=[
                default_params,
                {
                    'use_sim_time': use_sim_time,
                    'output_topic': '/explore/cmd_vel',
                    'inhibit_topic': '/explore/inhibit',
                },
            ],
        ),

        # --- Coordinator 1: Inhibitor on Wander output ---
        Node(
            package='turtlebot3_subsumption',
            executable='inhibitor_node',
            name='inhibitor_wander',
            output='screen',
            parameters=[
                default_params,
                {
                    'use_sim_time': use_sim_time,
                    'lower_topic': '/wander/cmd_vel',
                    'upper_topic': '/explore/inhibit',
                    'output_topic': '/wander/inhibited_cmd_vel',
                    'upper_msg_type': 'bool',
                },
            ],
        ),

        # --- Coordinator 2: Suppressor between Layer 1 (Wander) and Layer 2 (Explore) ---
        Node(
            package='turtlebot3_subsumption',
            executable='suppressor_node',
            name='suppressor_l1_l2',
            output='screen',
            parameters=[
                default_params,
                {
                    'use_sim_time': use_sim_time,
                    'lower_topic': '/wander/inhibited_cmd_vel',
                    'upper_topic': '/explore/cmd_vel',
                    'output_topic': '/subsumption/nav_cmd_vel',
                },
            ],
        ),

        # --- Coordinator 3: Safety Suppressor guaranteeing Layer 0 survival priority ---
        Node(
            package='turtlebot3_subsumption',
            executable='suppressor_node',
            name='suppressor_safety',
            output='screen',
            parameters=[
                default_params,
                {
                    'use_sim_time': use_sim_time,
                    'lower_topic': '/subsumption/nav_cmd_vel',
                    'upper_topic': '/avoid/cmd_vel',
                    'output_topic': '/cmd_vel',
                },
            ],
        ),

        # --- Optional Trajectory Logger (disabled by default to keep rqt_graph pure) ---
        Node(
            package='turtlebot3_subsumption',
            executable='trajectory_logger_node',
            name='trajectory_logger_node',
            output='screen',
            condition=IfCondition(record_trajectory),
            parameters=[{
                'use_sim_time': use_sim_time,
                'level': 2,
                'output_csv': output_csv,
            }],
        ),
    ])
