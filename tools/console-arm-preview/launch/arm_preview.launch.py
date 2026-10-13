"""Isolated model-only ROS graph for the Foxglove URDF pose preview."""

from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

ROOT = Path(__file__).resolve().parents[3]
ARM_TOOLS = ROOT / "tools/console-arm-preview"
URDF = ROOT / "tzb_ws/src/tzb_description/urdf/tzb_description.urdf"


def generate_launch_description():
    description = URDF.read_text(encoding="utf-8")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": description,
            "publish_frequency": 30.0,
            "use_sim_time": False,
        }],
        remappings=[("/joint_states", "/arm_preview/joint_states")],
    )
    joint_state_preview = ExecuteProcess(
        cmd=["python3", str(ARM_TOOLS / "arm_preview_server.py")],
        output="screen",
    )
    foxglove_bridge = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        parameters=[{
            "address": "127.0.0.1",
            "port": 8768,
            "tls": False,
            "sysinfo": False,
            "topic_whitelist": [
                "^/arm_preview/joint_states$",
                "^/tf$",
                "^/tf_static$",
            ],
            "service_whitelist": ["(?!)"],
            "param_whitelist": [".*robot_description.*"],
            "client_topic_whitelist": ["(?!)"],
            "asset_uri_allowlist": ["^package://tzb_description/meshes/.*$"],
            "capabilities": ["parameters", "parametersSubscribe", "connectionGraph", "assets"],
        }],
    )
    return LaunchDescription([robot_state_publisher, joint_state_preview, foxglove_bridge])
