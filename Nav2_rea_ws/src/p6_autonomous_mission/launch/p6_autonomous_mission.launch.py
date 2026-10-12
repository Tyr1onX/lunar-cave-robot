"""Launch stock Nav2 against the isolated cave base model and P6 coordinator."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory('p6_autonomous_mission')
    nav2_share = get_package_share_directory('nav2_bringup')
    params = os.path.join(package_share, 'config', 'nav2_offline.yaml')
    nav2_launch = os.path.join(nav2_share, 'launch', 'navigation_launch.py')
    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='97'),
        DeclareLaunchArgument('scenario', default_value='normal'),
        DeclareLaunchArgument('arm_backend', default_value='synthetic'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        DeclareLaunchArgument('p4e_status_required', default_value='false'),
        DeclareLaunchArgument('chassis_command_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('navigation_timeout_sec', default_value='90.0'),
        DeclareLaunchArgument('test_cancel_delay_sec', default_value='0.5'),
        DeclareLaunchArgument('event_log_path', default_value='/tmp/p6_autonomous_mission_events.jsonl'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        Node(
            package='tf2_ros', executable='static_transform_publisher',
            name='p6_map_to_odom', output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0', '--roll', '0',
                       '--pitch', '0', '--yaw', '0', '--frame-id', 'map',
                       '--child-frame-id', 'odom'],
        ),
        Node(
            package='p6_autonomous_mission', executable='cave_simulator',
            name='p6_cave_simulator', output='screen',
            parameters=[{'command_topic': LaunchConfiguration('chassis_command_topic')}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch),
            launch_arguments={
                'params_file': params,
                'use_sim_time': 'False',
                'autostart': 'True',
            }.items(),
        ),
        Node(
            package='p6_autonomous_mission', executable='mission_coordinator',
            name='p6_mission_coordinator', output='screen',
            parameters=[{
                'arm_backend': LaunchConfiguration('arm_backend'),
                'scenario': LaunchConfiguration('scenario'),
                'test_cancel_delay_sec': LaunchConfiguration('test_cancel_delay_sec'),
                'auto_start': ParameterValue(LaunchConfiguration('auto_start'), value_type=bool),
                'p4e_status_required': ParameterValue(LaunchConfiguration('p4e_status_required'), value_type=bool),
                'navigation_timeout_sec': ParameterValue(
                    LaunchConfiguration('navigation_timeout_sec'), value_type=float),
                'event_log_path': LaunchConfiguration('event_log_path'),
            }],
        ),
    ])
