from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            description=(
                "Required MID-360 JSON config. Confirm sensor/host addresses and route "
                "before selecting a device-specific file."
            ),
        ),
        DeclareLaunchArgument(
            "cmdline_input_bd_code",
            description=(
                "Required MID-360 broadcast code read from the device; do not use a placeholder."
            ),
        ),
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_mid360_static",
            output="screen",
            parameters=[{
                "xfer_format": 0,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": "livox_frame",
                "lvx_file_path": "/tmp/livox_unused.lvx",
                "user_config_path": LaunchConfiguration("config_file"),
                "cmdline_input_bd_code": LaunchConfiguration("cmdline_input_bd_code"),
            }],
        ),
    ])
