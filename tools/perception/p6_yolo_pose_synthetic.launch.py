#!/usr/bin/env python3
"""Localhost-only YOLO → pose-estimator → P6 software integration fixture.

This launch starts the software perception nodes, synthetic RGB-D inputs,
Nav2's P6 cave simulator, the P6 coordinator, and a simulated task-feedback
peer. It does not start a camera, robot driver, serial bridge, or actuator.
"""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    p6_share = Path(get_package_share_directory('p6_autonomous_mission'))
    pose_share = Path(get_package_share_directory('pose_estimator'))
    yolo_python = os.environ.get('YOLO_NODE_PYTHON', sys.executable)
    pose_python = os.environ.get('POSE_ESTIMATOR_PYTHON', sys.executable)
    fixture_python = os.environ.get('P6_SYNTHETIC_PYTHON', sys.executable)
    fixture_dir = Path(__file__).resolve().parent

    return LaunchDescription([
        DeclareLaunchArgument('domain_id', default_value='151'),
        DeclareLaunchArgument('mission_scenario', default_value='normal'),
        DeclareLaunchArgument('input_scenario', default_value='normal'),
        DeclareLaunchArgument('model_path', default_value=os.environ.get('TZB_OPENVINO_MODEL', '')),
        DeclareLaunchArgument('model_pcd', default_value=str(
            pose_share / 'config' / 'half_block.pcd')),
        DeclareLaunchArgument('event_log_path', default_value='/tmp/p6-yolo-pose-mission-events.jsonl'),
        DeclareLaunchArgument('yolo_conf_threshold', default_value='0.1'),
        DeclareLaunchArgument('yolo_publish_conf_threshold', default_value='0.1'),
        DeclareLaunchArgument('target_timeout_sec', default_value='15.0'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('domain_id')),
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        Node(
            package='yolo_node', executable='yolo_detector.py', name='yolo_node',
            output='screen', prefix=[yolo_python],
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'conf_threshold': ParameterValue(
                    LaunchConfiguration('yolo_conf_threshold'), value_type=float),
                'publish_conf_threshold': ParameterValue(
                    LaunchConfiguration('yolo_publish_conf_threshold'), value_type=float),
                'publish_hold_seconds': 0.0,
                'device': 'cpu',
                'image_topic': '/camera/color/image_raw',
                'detection_topic': '/yolo/detections',
                'annotated_image_topic': '/yolo/annotated_image',
                'target_class': 'blue_block',
                'timing_log_enable': True,
                'timing_log_throttle_sec': 0.0,
                'image_queue_depth': 1,
            }],
        ),
        Node(
            package='pose_estimator', executable='pose_estimation_node',
            name='pose_estimation_node', output='screen', prefix=[pose_python],
            parameters=[
                str(pose_share / 'config' / 'params.yaml'),
                {
                    'pointcloud_topic': '/camera/depth/color/points',
                    'camera_info_topic': '/camera/color/camera_info',
                    'detection_topic': '/yolo/detections',
                    'object_pose_topic': '/pose_estimator/object_pose',
                    'target_class': 'blue_block',
                    # Keep the synthetic fixture permissive; this test-only override
                    # does not change the shipped pose-estimator configuration.
                    'det_min_score': 0.0001,
                },
            ],
        ),
        ExecuteProcess(
            cmd=[
                fixture_python, str(fixture_dir / 'p6_yolo_pose_synthetic_inputs.py'),
                '--scenario', LaunchConfiguration('input_scenario'),
                '--model-pcd', LaunchConfiguration('model_pcd'),
            ],
            output='screen',
        ),
        ExecuteProcess(
            cmd=[
                fixture_python, str(fixture_dir / 'p6_synthetic_task_feedback.py'),
                '--scenario', LaunchConfiguration('mission_scenario'),
            ],
            output='screen',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(p6_share / 'launch' / 'p6_autonomous_mission.launch.py')),
            launch_arguments={
                'domain_id': LaunchConfiguration('domain_id'),
                'scenario': LaunchConfiguration('mission_scenario'),
                'arm_backend': 'synthetic',
                'auto_start': 'true',
                'p4e_status_required': 'false',
                'chassis_link_status_required': 'false',
                'target_topic': '/pose_estimator/object_pose',
                'target_timeout_sec': LaunchConfiguration('target_timeout_sec'),
                'event_log_path': LaunchConfiguration('event_log_path'),
            }.items(),
        ),
    ])
