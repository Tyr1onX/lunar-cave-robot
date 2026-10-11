import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = get_package_share_directory("chassis_serial_bridge")
    params_file = os.path.join(package_share, "config", "serial_params.yaml")
    device_name = LaunchConfiguration("device_name")
    command_topic = LaunchConfiguration("command_topic")
    return LaunchDescription([
        DeclareLaunchArgument(
            "device_name",
            description="Explicit serial device path for the STM32 USART1 bridge; use a PTY for offline tests.",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="/cmd_vel",
            description="Bridge input topic. Nav2 bringup uses the velocity_smoother output.",
        ),
        Node(
            package="chassis_serial_bridge",
            executable="chassis_serial_bridge_node",
            name="chassis_serial_bridge",
            output="screen",
            parameters=[
                params_file,
                {"device_name": device_name, "command_topic": command_topic},
            ],
        ),
    ])
