#!/usr/bin/env python3
"""Isolated Fortress physics + existing YOLO, pose, and tzb_catch algorithms."""
from __future__ import annotations

import os
import tempfile
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
    LogInfo, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable,
    Shutdown, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from tzb_sim.gazebo_model import build_gazebo_urdf


def _merge(base, overlay):
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value
    return base


def _read_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def _write_yaml(output_dir, name, data):
    path = output_dir / name
    path.write_text(yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
                    encoding="utf-8")
    return str(path)


def _write_robot_description(description_share, controller_config, output_dir):
    source = Path(description_share) / "urdf" / "tzb_description.urdf"
    # Gazebo consumes the absolute-path copy from disk. robot_state_publisher
    # keeps package:// URIs so RViz's resource_retriever can resolve the meshes.
    gazebo_description = build_gazebo_urdf(str(source), controller_config)
    rviz_description = build_gazebo_urdf(
        str(source), controller_config, preserve_package_meshes=True)
    path = output_dir / "tzb_gazebo_robot.urdf"
    path.write_text(gazebo_description, encoding="utf-8")
    return rviz_description, str(path)


def _build(context, sim_share, description_share, pose_share,
           planner_share, yolo_share, ros_gz_sim_share):
    output_dir = Path(os.environ.get(
        "TZB_GZ_LOG_DIR",
        str(Path.home() / "tzb-study-output/issue48-p2-gazebo/logs"),
    )) / "generated"
    output_dir.mkdir(parents=True, exist_ok=True)

    controller_config = str(Path(sim_share) / "config/gazebo_controllers.yaml")
    robot_description, urdf_path = _write_robot_description(
        description_share, controller_config, output_dir)

    overrides = _read_yaml(str(Path(sim_share) / "config/sim_overrides.yaml"))
    pose_params = _merge(
        _read_yaml(str(Path(pose_share) / "config/params.yaml")), overrides)
    pose_params = _merge(pose_params, {
        "pose_estimation_node": {"ros__parameters": {
            "use_sim_time": True,
            "pointcloud_topic": "/p2_gazebo/camera/depth/optical/points",
            "detection_topic": "/p2_gazebo/yolo/detections",
            "camera_info_topic": "/p2_gazebo/camera/optical/camera_info",
            "object_pose_topic": "/p2_gazebo/pose_estimator/object_pose",
            "camera_frame": "sim_camera_optical_frame",
            "target_class": "blue_block",
            "roi_timeout_sec": 2.0,
        }},
    })
    pose_params_path = _write_yaml(output_dir, "pose_estimator_gazebo.yaml", pose_params)

    planner_dynamic = {
        "/**": {"ros__parameters": {
            "use_sim_time": True,
            "urdf_path": urdf_path,
            "srdf_path": str(Path(planner_share) / "config/tzb_description.srdf"),
            "mesh_package_dirs": [description_share],
            "visualization_joint_states_topic": "/p2_gazebo/arm_joint_state",
            "publish_joint_states_for_visualization": False,
            "enable_joint_state_playback": False,
            "launch_robot_state_publisher": False,
            "tzb.pose_estimator_reference_frame": "sim_camera_optical_frame",
            "tzb.pose_estimator_target_topic": "/p2_gazebo/pose_estimator/object_pose",
            "tzb.arm_state_topic": "/p2_gazebo/arm_joint_state",
            "tzb.arm_feedback_enabled": True,
            "tzb.require_arm_feedback_for_completion": True,
            "tzb.arm_command_enabled": True,
            "tzb.arm_command_topic": "/p2_gazebo/arm_joint_command",
            "tzb.gripper_command_topic": "/p2_gazebo/gripper_command",
            "tzb.e3_mesh_resource": "package://tzb_sim/config/blue_block.stl",
            "tzb.mesh_scale": [0.001, 0.001, 0.001],
            "tzb.publish_e3_marker": False,
            "tzb.replay_after_complete": False,
            "tzb.replay_loop": False,
            "tzb.sequence_loop": False,
            "tzb.autostart": LaunchConfiguration("autostart").perform(context).lower() == "true",
        }},
    }
    planner_params = _merge(
        _read_yaml(str(Path(planner_share) / "config/tzb_planner.yaml")), overrides)
    planner_params = _merge(planner_params, planner_dynamic)
    planner_params = _merge(planner_params, {
        "tzb_catch": {"ros__parameters": {
            "tzb.arm_state_topic": "/p2_gazebo/arm_joint_state",
            "tzb.arm_feedback_enabled": True,
            "tzb.require_arm_feedback_for_completion": True,
            "tzb.arm_command_enabled": True,
            "tzb.arm_command_topic": "/p2_gazebo/arm_joint_command",
            "tzb.gripper_command_topic": "/p2_gazebo/gripper_command",
            "tzb.pose_estimator_target_topic": "/p2_gazebo/pose_estimator/object_pose",
            "tzb.pose_estimator_reference_frame": "sim_camera_optical_frame",
            "tzb.pose_estimator_target_max_age_sec": 2.0,
            "tzb.publish_e3_marker": False,
            # Keep theoretical planner state off the physical Gazebo feedback topic.
            "visualization_joint_states_topic": "/p2_gazebo/planner_joint_states",
        }},
    })
    planner_params_path = _write_yaml(
        output_dir, "planner_gazebo.yaml", planner_params)

    world = str(Path(sim_share) / "config/gazebo_world.sdf")
    bridge_config = str(Path(sim_share) / "config/gazebo_bridge.yaml")
    pose_bridge_config = str(Path(sim_share) / "config/gazebo_pose_bridge.yaml")
    rviz_config = str(Path(sim_share) / "config/physical_grasp.rviz")
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(ros_gz_sim_share) / "launch/gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -v 3 {world}"}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True,
            "publish_frequency": 50.0,
        }],
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="p2_gazebo_bridge",
        output="screen",
        parameters=[{
            "config_file": bridge_config,
            "override_frame_id": "sim_camera_link",
            "override_timestamps_with_wall_time": False,
        }],
    )

    pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="p2_gazebo_pose_bridge",
        output="screen",
        parameters=[{
            "config_file": pose_bridge_config,
            "override_frame_id": "world",
            "override_timestamps_with_wall_time": False,
        }],
    )
    pose_tf_adapter = Node(
        package="tzb_sim",
        executable="gazebo_pose_tf_adapter",
        name="gazebo_pose_tf_adapter",
        output="screen",
        parameters=[{"use_sim_time": True, "parent_frame": "world"}],
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_tzb_robot",
        output="screen",
        arguments=[
            "-world", "tzb_p2",
            "-file", urdf_path,
            "-name", "tzb_robot",
            "-x", "0", "-y", "0", "-z", "0.219",
        ],
    )
    controllers = ExecuteProcess(
        cmd=["bash", str(Path(sim_share) / "tools/spawn_gazebo_controllers.sh")],
        output="screen",
    )

    effort_controller = Node(
        package="tzb_sim",
        executable="gazebo_joint_command_adapter",
        name="gazebo_joint_command_adapter",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "joint_states_topic": "/joint_states",
            "arm_state_topic": "/p2_gazebo/arm_joint_state",
            "arm_command_topic": "/p2_gazebo/arm_joint_command",
            "gripper_command_topic": "/p2_gazebo/gripper_command",
        }],
    )
    rgbd_adapter = Node(
        package="tzb_sim",
        executable="gazebo_rgbd_optical_adapter",
        name="gazebo_rgbd_optical_adapter",
        output="screen",
        parameters=[{"use_sim_time": True,
                     "optical_frame": "sim_camera_optical_frame"}],
    )
    visualizer = Node(
        package="tzb_sim",
        executable="gazebo_state_visualizer",
        name="gazebo_state_visualizer",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    model_path = LaunchConfiguration("model_path")
    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(yolo_share) / "launch/yolo_node.launch.py")),
        launch_arguments={
            "image_topic": "/p2_gazebo/camera/color/image_raw",
            "detection_topic": "/p2_gazebo/yolo/detections",
            "annotated_image_topic": "/p2_gazebo/yolo/annotated_image",
            "model_path": model_path,
            "yolo_python": LaunchConfiguration("yolo_python"),
            "target_class": "blue_block",
            "device": "cpu",
        }.items(),
    )
    pose = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(pose_share) / "launch/pose_estimator.launch.py")),
        launch_arguments={
            "launch_camera": "false",
            "params_file": pose_params_path,
            "pose_estimator_python": LaunchConfiguration("pose_estimator_python"),
        }.items(),
    )
    catch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(planner_share) / "launch/tzb_catch.launch.py")),
        launch_arguments={
            "robot_model": "tzb_description",
            "params_file": planner_params_path,
            "autostart": LaunchConfiguration("autostart"),
            "listen_navigation_goal_status": LaunchConfiguration("listen_navigation_goal_status"),
            "use_serial_driver": "false",
            "use_robot_state_publisher": "false",
            "use_rviz": LaunchConfiguration("use_rviz"),
            "rviz_config": rviz_config,
        }.items(),
    )

    def after_controllers(event, _context):
        if event.returncode != 0:
            return [
                LogInfo(msg=f"ERROR: Gazebo ros2_control spawner failed: {event.returncode}"),
                Shutdown(reason="Gazebo controllers did not become active"),
            ]
        return [
            LogInfo(msg=(
                "P2 physical control active: arm position targets and effort-limited "
                "gripper -> Gazebo physics -> measured joint_state_broadcaster feedback; "
                "no serial driver")),
            pose_bridge,
            pose_tf_adapter,
            rgbd_adapter,
            visualizer,
            yolo,
            pose,
            TimerAction(
                period=5.0, actions=[catch],
                condition=IfCondition(LaunchConfiguration("enable_grasp_pipeline"))),
        ]

    return [
        LogInfo(msg=(
            "Issue #48 P2 isolated Fortress simulation: Gazebo RGB-D -> OpenVINO "
            "YOLO -> pose_estimator -> tzb_catch/lite_motion_planner -> "
            "physics-controlled motion and force-limited gripper; no hardware drivers")),
        gz_launch,
        robot_state_publisher,
        bridge,
        RegisterEventHandler(
            OnProcessExit(target_action=spawn_robot, on_exit=[controllers, effort_controller])),
        RegisterEventHandler(
            OnProcessExit(target_action=controllers, on_exit=after_controllers)),
        spawn_robot,
    ]


def generate_launch_description():
    sim_share = get_package_share_directory("tzb_sim")
    description_share = get_package_share_directory("tzb_description")
    pose_share = get_package_share_directory("pose_estimator")
    planner_share = get_package_share_directory("lite_motion_planner")
    yolo_share = get_package_share_directory("yolo_node")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    model_default = os.environ.get(
        "TZB_OPENVINO_MODEL",
        str(Path.home() / "tzb-study-output/issue48-p0-p1-2026-10-18/"
            "ska_1_reference/best_openvino_model/blue_block"),
    )
    yolo_python_default = os.environ.get(
        "YOLO_NODE_PYTHON", str(Path.home() / "robot_p0/.venv/bin/python"))
    pose_python_default = os.environ.get(
        "POSE_ESTIMATOR_PYTHON", str(Path.home() / "robot_p0/.venv/bin/python"))
    return LaunchDescription([
        SetEnvironmentVariable("ROS_DOMAIN_ID", os.environ.get(
            "ROS_DOMAIN_ID", "216")),
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        SetEnvironmentVariable("IGN_IP", "127.0.0.1"),
        SetEnvironmentVariable("IGN_PARTITION", os.environ.get(
            "IGN_PARTITION", "tzb-issue48-p2-216")),
        DeclareLaunchArgument("autostart", default_value="true"),
        DeclareLaunchArgument("listen_navigation_goal_status", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("enable_grasp_pipeline", default_value="true"),
        DeclareLaunchArgument("model_path", default_value=model_default),
        DeclareLaunchArgument("yolo_python", default_value=yolo_python_default),
        DeclareLaunchArgument("pose_estimator_python", default_value=pose_python_default),
        OpaqueFunction(function=lambda context: _build(
            context, sim_share, description_share, pose_share, planner_share,
            yolo_share, ros_gz_sim_share)),
    ])
