import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    use_rviz = LaunchConfiguration("rviz")
    point_lio_cfg_dir = LaunchConfiguration("point_lio_cfg_dir")
    point_lio_dir = get_package_share_directory("point_lio")

    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="True", description="Flag to launch RViz."
    )

    declare_point_lio_cfg_dir = DeclareLaunchArgument(
        "point_lio_cfg_dir",
        default_value=PathJoinSubstitution([point_lio_dir, "config", "mid360_gravity.yaml"]),
        description="Path to the Point-LIO config file",
    )

    start_point_lio_node = Node(
        package="point_lio",
        executable="pointlio_mapping",
        parameters=[point_lio_cfg_dir],
        remappings=remappings,
        output="screen",
    )

    start_rviz_node = Node(
        condition=IfCondition(use_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz",
        remappings=remappings,
        arguments=[
            "-d",
            PathJoinSubstitution([point_lio_dir, "rviz_cfg", "loam_livox"]),
            ".rviz",
        ],
    )

    ld = LaunchDescription()

    ld.add_action(declare_rviz)
    ld.add_action(declare_point_lio_cfg_dir)
    ld.add_action(start_point_lio_node)
    # ld.add_action(start_rviz_node)

    return ld
