"""Single-MID360 Point-LIO mapping launch.

This is intentionally separate from mapping.launch.py, which remains the
dual-LiDAR full-pipeline reference launch.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    livox_share = get_package_share_directory("livox_ros_driver2")
    imu_tools_share = get_package_share_directory("imu_tools")
    point_lio_share = get_package_share_directory("point_lio")
    use_rviz = LaunchConfiguration("rviz")

    declare_rviz = DeclareLaunchArgument(
        "rviz",
        default_value="True",
        description="Launch RViz with the Point-LIO mapping display configuration.",
    )

    # Single MID-360 provides /livox/lidar and /livox/imu directly.
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch", "msg_MID360_launch.py")
        )
    )

    imu_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(imu_tools_share, "launch", "imu_transform.launch.py")
        )
    )

    imu_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                imu_tools_share, "launch", "imu_complementary_filter.launch.py"
            )
        )
    )

    point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(point_lio_share, "launch", "point_lio.launch.py")
        )
    )

    # The calibrated installation pose is applied by MID360_config.json.  This
    # identity TF keeps the project's logical base_frame and livox_frame
    # connected without applying that physical extrinsic a second time.
    cmd_tf_base_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="log",
        arguments=[
            "0", "0", "0", "0", "0", "0", "base_frame", "livox_frame"
        ],
        respawn=True,
    )

    rviz = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            os.path.join(bringup_share, "rviz", "pointlio_mapping.rviz"),
        ],
    )

    return LaunchDescription([
        declare_rviz,
        livox_driver,
        cmd_tf_base_to_livox,
        imu_transform,
        imu_filter,
        point_lio,
        rviz,
    ])
