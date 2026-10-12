#!/usr/bin/env python3
"""Offline P3b profile: default URDF/RSP plus synthetic Orbbec optical TF."""

import os
import tempfile
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_TEMP_PARAMS = set()


def _cleanup(_context):
    for path in list(_TEMP_PARAMS):
        try:
            os.unlink(path)
        except FileNotFoundError:
            pass
        _TEMP_PARAMS.discard(path)
    return []


def _make_nodes(context):
    planner_share = get_package_share_directory('lite_motion_planner')
    description_share = get_package_share_directory('tzb_description')
    config_path = os.path.join(planner_share, 'config', 'offline_p2_planner.yaml')
    with open(config_path, 'r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    params = data.setdefault('/**', {}).setdefault('ros__parameters', {})

    params['urdf_path'] = os.path.join(description_share, 'urdf', 'tzb_description.urdf')
    params['srdf_path'] = os.path.join(planner_share, 'config', 'tzb_description.srdf')
    params['mesh_package_dirs'] = [description_share, planner_share]
    params['joint_trajectory_topic'] = '/p2_offline/trajectory_preview'
    params['visualization_joint_states_topic'] = '/p2_offline/joint_states'
    params['tzb.arm_state_topic'] = '/p2_offline/arm_joint_state'
    params['tzb.arm_command_topic'] = '/p2_offline/arm_joint_command'
    params['tzb.gripper_command_topic'] = '/p2_offline/gripper_command'
    params['tzb.pose_estimator_target_topic'] = '/p2_offline/pose_estimator/object_pose'
    params['tzb.use_yaml_e3_target'] = False
    auto = LaunchConfiguration('autostart').perform(context).lower() == 'true'
    p6_staged = LaunchConfiguration('p6_staged_tasks').perform(context).lower() == 'true'
    params['tzb.autostart'] = auto
    params['tzb.p6_staged_tasks'] = p6_staged
    params['tzb.p6_task_request_topic'] = '/p6/arm_task/request'
    params['tzb.p6_task_cancel_topic'] = '/p6/arm_task/cancel'
    params['tzb.p6_task_result_topic'] = '/p6/arm_task/result'
    params['tzb.listen_navigation_goal_status'] = not auto and not p6_staged

    safe_topics = (
        'joint_trajectory_topic', 'visualization_joint_states_topic',
        'tzb.arm_state_topic', 'tzb.arm_command_topic', 'tzb.gripper_command_topic',
        'tzb.pose_estimator_target_topic',
    )
    if any(not params[name].startswith('/p2_offline/') for name in safe_topics):
        raise RuntimeError('P3b isolation check failed: a topic escaped /p2_offline')
    if params.get('tzb.arm_feedback_enabled') is not True:
        raise RuntimeError('P3b requires synthetic arm feedback')

    handle = tempfile.NamedTemporaryFile(
        mode='w', prefix='p3b_offline_planner_', suffix='.yaml', delete=False)
    try:
        yaml.safe_dump(data, handle, sort_keys=False, allow_unicode=True)
        params_file = handle.name
    finally:
        handle.close()
    _TEMP_PARAMS.add(params_file)

    # Exact Orbbec optical-axis convention from ob_camera_node.cpp.
    # Zero translation is a simulation fixture, not a measured camera extrinsic.
    optical_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='p3b_simulated_orbbec_optical_tf', output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '-1.5707963267948966', '--pitch', '0', '--yaw', '-1.5707963267948966',
            '--frame-id', 'camera_link', '--child-frame-id', 'camera_color_optical_frame',
        ],
    )
    feedback = Node(
        package='lite_motion_planner', executable='p2_offline_feedback.py',
        name='p2_offline_arm_feedback', output='screen',
        parameters=[{
            'state_topic': '/p2_offline/arm_joint_state',
            'command_topic': '/p2_offline/arm_joint_command',
        }],
        condition=IfCondition(LaunchConfiguration('synthetic_arm_feedback')),
    )
    joint_state_relay = Node(
        package='lite_motion_planner', executable='p3b_joint_state_relay.py',
        name='p3b_offline_joint_state_relay', output='screen',
        parameters=[{
            'state_topic': '/p2_offline/arm_joint_state',
            'joint_states_topic': '/p2_offline/joint_states',
        }],
        condition=IfCondition(LaunchConfiguration('publish_synthetic_target')),
    )
    vision = Node(
        package='lite_motion_planner', executable='p3b_synthetic_vision.py',
        name='p3b_synthetic_vision', output='screen',
        parameters=[{
            'target_topic': '/p2_offline/pose_estimator/object_pose',
            'target_xyz_base': params['tzb.e3_target_xyz'],
            'target_rpy_base': params['tzb.e3_target_rpy'],
        }],
        condition=IfCondition(LaunchConfiguration('publish_synthetic_target')),
    )
    catch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(planner_share, 'launch', 'tzb_catch.launch.py')),
        launch_arguments={
            'params_file': params_file,
            'robot_model': 'tzb_description',
            'autostart': LaunchConfiguration('autostart'),
            'listen_navigation_goal_status': 'false' if auto or p6_staged else 'true',
            'use_serial_driver': 'false',
            'use_robot_state_publisher': 'true',
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )
    return [
        LogInfo(msg=(
            'P3b simulation only: isolated localhost ROS domain; the default tzb_description '
            'URDF drives robot_state_publisher from synthetic /p2_offline/joint_states. '
            'The optical frame fixture uses the Orbbec source RPY convention; zero translation '
            'is not a measured camera extrinsic. No serial, camera, base, lidar, or hardware '
            'control node is launched.')),
        optical_tf,
        feedback,
        joint_state_relay,
        catch,
        vision,
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=_cleanup)])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('offline_domain_id', default_value='97'),
        DeclareLaunchArgument('autostart', default_value='false'),
        DeclareLaunchArgument('p6_staged_tasks', default_value='false'),
        DeclareLaunchArgument('publish_synthetic_target', default_value='false'),
        DeclareLaunchArgument('synthetic_arm_feedback', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('offline_domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        OpaqueFunction(function=_make_nodes),
    ])
