import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    LogInfo, ExecuteProcess, TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def _default_pose_estimator_python(pkg_dir):
    override = os.environ.get('POSE_ESTIMATOR_PYTHON')
    if override:
        return override
    for workspace_root in (Path(pkg_dir), *Path(pkg_dir).parents):
        candidate = workspace_root / '.venv' / 'bin' / 'python'
        if candidate.is_file():
            return str(candidate)
    return '/usr/bin/python3'


def generate_launch_description():
    pkg_dir = get_package_share_directory('pose_estimator')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')
    camera_config_file = os.path.join(pkg_dir, 'config', 'camera_switch.yaml')
    estimator_python = _default_pose_estimator_python(pkg_dir)

    return LaunchDescription([
        DeclareLaunchArgument(
            'pose_estimator_python', default_value=estimator_python,
            description='Python executable for Open3D and ROS dependencies'),

        # =================== 启动参数 ===================
        DeclareLaunchArgument('params_file', default_value=params_file),
        DeclareLaunchArgument('camera_config_file', default_value=camera_config_file),
        DeclareLaunchArgument('launch_camera', default_value='false',
                              description='是否按 camera_switch.yaml 启动相机'),

        # =================== YAML-selected camera ===================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_dir,
                '/launch/camera_switch.launch.py'
            ]),
            launch_arguments={
                'camera_config_file': LaunchConfiguration('camera_config_file'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('launch_camera')),
        ),

        # =================== 延迟诊断: 3秒后打印话题列表 ===================
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=['bash', '-c',
                         'echo "\\n===== 当前点云相关话题 =====" && '
                         'ros2 topic list 2>/dev/null | grep -E "points|camera_info|depth" && '
                         'echo "=========================\\n"'],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('launch_camera')),
                ),
            ],
        ),

        # =================== 位姿估计节点 ===================
        Node(
            package='pose_estimator',
            executable='pose_estimation_node',
            name='pose_estimation_node',
            output='screen',
            prefix=[LaunchConfiguration('pose_estimator_python')],
            parameters=[LaunchConfiguration('params_file')],
        ),

        LogInfo(msg=[
            '\n============================================\n',
            '  pose_estimator 已启动\n',
            '  相机选择配置: ', LaunchConfiguration('camera_config_file'), '\n',
            '============================================\n',
        ]),
    ])
