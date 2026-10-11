#!/usr/bin/env python3
"""One-command, localhost-only offline arm validation profile derived from P1."""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_TEMP_PARAMS = set()


def _cleanup_temp_params(_context):
    for path in list(_TEMP_PARAMS):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        _TEMP_PARAMS.discard(path)
    return []


def _make_offline_nodes(context):
    planner_share = get_package_share_directory('lite_motion_planner')
    description_share = get_package_share_directory('tzb_description')
    config_path = os.path.join(planner_share, 'config', 'offline_p2_planner.yaml')
    with open(config_path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    params = data.setdefault('/**', {}).setdefault('ros__parameters', {})

    # Resolve package files from the active overlay, never from /home/tzb or a
    # checkout-specific absolute path.
    params['urdf_path'] = os.path.join(
        description_share, 'urdf', 'tzb_description.urdf')
    params['srdf_path'] = os.path.join(
        planner_share, 'config', 'tzb_description.srdf')
    params['mesh_package_dirs'] = [description_share, planner_share]

    # Fail closed if an output topic drifts back to a hardware-facing default.
    safe_topics = {
        'joint_trajectory_topic': '/p2_offline/trajectory_preview',
        'visualization_joint_states_topic': '/p2_offline/joint_states',
        'tzb.arm_state_topic': '/p2_offline/arm_joint_state',
        'tzb.arm_command_topic': '/p2_offline/arm_joint_command',
        'tzb.gripper_command_topic': '/p2_offline/gripper_command',
    }
    for name, value in safe_topics.items():
        params[name] = value
    if params.get('tzb.arm_feedback_enabled') is not True:
        raise RuntimeError('offline profile requires its isolated synthetic JointState feedback')
    if any(
        params.get(name) != value or not params[name].startswith('/p2_offline/')
        for name, value in safe_topics.items()
    ):
        raise RuntimeError('offline output topic isolation check failed')

    handle = tempfile.NamedTemporaryFile(
        mode='w', prefix='p2_offline_planner_', suffix='.yaml', delete=False)
    try:
        yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=True)
        params_file = handle.name
    finally:
        handle.close()
    _TEMP_PARAMS.add(params_file)

    catch_launch = os.path.join(planner_share, 'launch', 'tzb_catch.launch.py')
    feedback = Node(
        package='lite_motion_planner',
        executable='p2_offline_feedback.py',
        name='p2_offline_arm_feedback',
        output='screen',
        parameters=[{
            'state_topic': '/p2_offline/arm_joint_state',
            'command_topic': '/p2_offline/arm_joint_command',
        }],
    )
    catch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(catch_launch),
        launch_arguments={
            'params_file': params_file,
            'robot_model': 'tzb_description',
            'autostart': 'true',
            'use_serial_driver': 'false',
            'use_robot_state_publisher': 'true',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )
    return [
        LogInfo(msg=(
            'P2 offline profile: domain=187, localhost-only; synthetic feedback '
            'and all arm/gripper/trajectory topics are isolated under /p2_offline. '
            'No camera, serial, base, or robot-control driver is launched.')),
        feedback,
        catch,
        RegisterEventHandler(OnShutdown(
            on_shutdown=[OpaqueFunction(function=_cleanup_temp_params)])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('offline_domain_id', default_value='187'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('offline_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        OpaqueFunction(function=_make_offline_nodes),
    ])
