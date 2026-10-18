#!/usr/bin/env python3
"""Offline RGB-D to visual grasp demo using the repository's normal nodes."""

import math
import os
import re
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


CAMERA_XYZ = (0.0, 0.0, 0.60)
CAMERA_PITCH_DEG = 52.13
CAMERA_YAW_DEG = 26.57


def _deep_merge(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def _load_yaml(path):
    with open(path, 'r', encoding='utf-8') as handle:
        return yaml.safe_load(handle) or {}


def _write_merged_params(base_path, override_path, dynamic_override, label):
    params = _deep_merge(_load_yaml(base_path), _load_yaml(override_path))
    _deep_merge(params, dynamic_override)
    with tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', prefix=f'tzb_sim_{label}_',
            suffix='.yaml', delete=False) as handle:
        yaml.safe_dump(params, handle, sort_keys=False, allow_unicode=True)
        return handle.name


def _write_sim_urdf(description_share, camera_frame):
    if not re.fullmatch(r'[A-Za-z][A-Za-z0-9_]*', camera_frame):
        raise RuntimeError('camera_frame must be a simple ROS frame name')
    import numpy as np
    from scipy.spatial.transform import Rotation

    pitch = math.radians(CAMERA_PITCH_DEG)
    yaw = math.radians(CAMERA_YAW_DEG)
    x_axis = np.array([0.0, -1.0, 0.0])
    z_axis = np.array([math.cos(pitch), 0.0, -math.sin(pitch)])
    y_axis = np.cross(z_axis, x_axis)
    optical = np.column_stack((x_axis, y_axis, z_axis))
    rot_z = np.array([[math.cos(yaw), -math.sin(yaw), 0.0],
                      [math.sin(yaw), math.cos(yaw), 0.0],
                      [0.0, 0.0, 1.0]])
    rotation = rot_z @ optical
    roll, pitch_euler, yaw_euler = Rotation.from_matrix(rotation).as_euler('xyz')

    source = os.path.join(description_share, 'urdf', 'tzb_description.urdf')
    with open(source, 'r', encoding='utf-8') as handle:
        urdf = handle.read()
    addition = (
        f'  <link name="{camera_frame}"/>\n'
        f'  <joint name="tzb_sim_camera_joint" type="fixed">\n'
        f'    <origin xyz="{CAMERA_XYZ[0]} {CAMERA_XYZ[1]} {CAMERA_XYZ[2]}"'
        f' rpy="{roll:.8f} {pitch_euler:.8f} {yaw_euler:.8f}"/>\n'
        f'    <parent link="base_link"/>\n'
        f'    <child link="{camera_frame}"/>\n'
        f'  </joint>\n</robot>\n')
    if urdf.count('</robot>') != 1:
        raise RuntimeError('tzb_description URDF must contain one closing robot tag')
    urdf = urdf.replace('</robot>', addition)
    with tempfile.NamedTemporaryFile(
            mode='w', encoding='utf-8', prefix='tzb_sim_robot_',
            suffix='.urdf', delete=False) as handle:
        handle.write(urdf)
        return handle.name


def _default_model_path(yolo_share):
    override = os.environ.get('TZB_OPENVINO_MODEL')
    if override:
        return override
    share = Path(yolo_share)
    for workspace_root in (share, *share.parents):
        candidate = workspace_root / 'src' / 'best_openvino_model' / 'blue_block'
        if candidate.is_dir():
            return str(candidate)
    return str(share / 'models' / 'blue_block')


def _default_python(env_name, package_share):
    override = os.environ.get(env_name)
    if override:
        return override
    for workspace_root in (Path(package_share), *Path(package_share).parents):
        candidate = workspace_root / '.venv' / 'bin' / 'python'
        if candidate.is_file():
            return str(candidate)
    return '/usr/bin/python3'


def _build_demo_nodes(context, sim_share, description_share, pose_share,
                      planner_share, yolo_share):
    camera_frame = LaunchConfiguration('camera_frame').perform(context)
    urdf_path = _write_sim_urdf(description_share, camera_frame)
    overrides = os.path.join(sim_share, 'config', 'sim_overrides.yaml')
    pose_params = _write_merged_params(
        os.path.join(pose_share, 'config', 'params.yaml'),
        overrides, {}, 'pose')
    planner_dynamic = {
        '/**': {'ros__parameters': {
            'urdf_path': urdf_path,
            'srdf_path': os.path.join(planner_share, 'config', 'tzb_description.srdf'),
            'mesh_package_dirs': [description_share],
            'visualization_joint_states_topic': '/serial/arm_joint_state',
            'publish_joint_states_for_visualization': False,
            'enable_joint_state_playback': False,
            'tzb.pose_estimator_reference_frame': camera_frame,
            'tzb.arm_state_topic': '/serial/arm_joint_state',
            'tzb.arm_feedback_enabled': True,
            'tzb.require_arm_feedback_for_completion': True,
            'tzb.arm_command_enabled': True,
            'tzb.e3_mesh_resource': 'package://tzb_sim/config/blue_block.stl',
            'tzb.mesh_scale': [0.001, 0.001, 0.001],
            'tzb.publish_e3_marker': True,
            'tzb.replay_after_complete': False,
            'tzb.replay_loop': False,
            'tzb.sequence_loop': False,
        }}}
    planner_params = _write_merged_params(
        os.path.join(planner_share, 'config', 'tzb_planner.yaml'),
        overrides, planner_dynamic, 'planner')

    scene = Node(
        package='tzb_sim', executable='blue_block_scene',
        name='blue_block_scene', output='screen',
        parameters=[{
            'publish_rate': LaunchConfiguration('publish_rate'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fx': LaunchConfiguration('fx'), 'fy': LaunchConfiguration('fy'),
            'cx': LaunchConfiguration('cx'), 'cy': LaunchConfiguration('cy'),
            'frame_id': camera_frame,
            'publish_camera_tf': False,
            'publish_gt_detections': False,
            'object_x': LaunchConfiguration('object_x'),
            'object_y': LaunchConfiguration('object_y'),
            'object_z': LaunchConfiguration('object_z'),
        }],
    )
    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(yolo_share, 'launch', 'yolo_node.launch.py')),
        launch_arguments={
            'image_topic': '/camera/color/image_raw',
            'detection_topic': '/yolo/detections',
            'annotated_image_topic': '/yolo/annotated_image',
            'model_path': LaunchConfiguration('model_path'),
            'yolo_python': LaunchConfiguration('yolo_python'),
            'target_class': 'blue_block',
            'device': 'cpu',
        }.items(),
    )
    pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pose_share, 'launch', 'pose_estimator.launch.py')),
        launch_arguments={
            'launch_camera': 'false',
            'params_file': pose_params,
            'pose_estimator_python': LaunchConfiguration('pose_estimator_python'),
        }.items(),
    )
    bridge = Node(
        package='tzb_sim', executable='serial_bridge_sim',
        name='serial_bridge_sim', output='screen',
        parameters=[{
            'initial_positions': LaunchConfiguration('initial_positions'),
        }],
    )
    rviz_config = os.path.join(sim_share, 'config', 'visual_grasp.rviz')
    catch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(planner_share, 'launch', 'tzb_catch.launch.py')),
        launch_arguments={
            'robot_model': 'tzb_description',
            'params_file': planner_params,
            'autostart': LaunchConfiguration('autostart'),
            'listen_navigation_goal_status': 'false',
            'use_serial_driver': 'false',
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_robot_state_publisher': 'true',
            'rviz_config': rviz_config,
        }.items(),
    )
    return [
        LogInfo(msg=(
            'tzb_sim isolated offline visual grasp: synthetic RGB-D -> '
            'OpenVINO YOLO -> pose_estimator -> tzb_catch -> virtual arm; '
            'use_serial_driver=false, launch_camera=false')),
        scene,
        bridge,
        TimerAction(period=2.0, actions=[yolo]),
        TimerAction(period=3.0, actions=[pose]),
        TimerAction(period=5.0, actions=[catch]),
    ]


def generate_launch_description():
    sim_share = get_package_share_directory('tzb_sim')
    description_share = get_package_share_directory('tzb_description')
    pose_share = get_package_share_directory('pose_estimator')
    planner_share = get_package_share_directory('lite_motion_planner')
    yolo_share = get_package_share_directory('yolo_node')
    return LaunchDescription([
        DeclareLaunchArgument('camera_frame', default_value='sim_camera_link'),
        DeclareLaunchArgument('publish_rate', default_value='5.0'),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fx', default_value='800.0'),
        DeclareLaunchArgument('fy', default_value='800.0'),
        DeclareLaunchArgument('cx', default_value='320.0'),
        DeclareLaunchArgument('cy', default_value='240.0'),
        DeclareLaunchArgument('object_x', default_value='0.40'),
        DeclareLaunchArgument('object_y', default_value='0.20'),
        DeclareLaunchArgument('object_z', default_value='0.025'),
        DeclareLaunchArgument(
            'initial_positions',
            default_value='[0.0, 1.0, -2.61, 0.0, 0.0, 0.0]'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'model_path', default_value=_default_model_path(yolo_share)),
        DeclareLaunchArgument(
            'yolo_python', default_value=_default_python('YOLO_NODE_PYTHON', yolo_share)),
        DeclareLaunchArgument(
            'pose_estimator_python',
            default_value=_default_python('POSE_ESTIMATOR_PYTHON', pose_share)),
        OpaqueFunction(function=lambda context: _build_demo_nodes(
            context, sim_share, description_share, pose_share,
            planner_share, yolo_share)),
    ])
