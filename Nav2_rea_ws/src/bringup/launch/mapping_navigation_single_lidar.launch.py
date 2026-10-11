"""Single-MID360 online Point-LIO mapping plus Nav2 navigation.

This follows the architecture used by Nav2_sim_ws: Point-LIO owns local
odometry, terrain analysis filters obstacles, a virtual LaserScan is rastered
by slam_toolbox into a live 2D /map, and Nav2 consumes that map.

``map -> odom_init`` is deliberately a static identity transform in this
mode.  It gives the newly-built map an origin at the Point-LIO start pose;
slam_toolbox is used as a 2D occupancy-grid mapper, not as a second pose
estimator.  Do not add alignment or another map -> odom_init publisher.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    bringup_share = get_package_share_directory("bringup")
    chassis_bridge_share = get_package_share_directory("chassis_serial_bridge")
    livox_share = get_package_share_directory("livox_ros_driver2")
    imu_tools_share = get_package_share_directory("imu_tools")
    point_lio_share = get_package_share_directory("point_lio")
    terrain_share = get_package_share_directory("terrain_analysis")
    nav_share = get_package_share_directory("rm_navigation")
    nav2_bt_share = get_package_share_directory("nav2_bt_navigator")

    use_sim_time = LaunchConfiguration("use_sim_time")
    nav_rviz = LaunchConfiguration("nav_rviz")
    chassis_device = LaunchConfiguration("chassis_device")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="False",
        description="Use a simulation clock (normally False on the real robot).",
    )
    declare_nav_rviz = DeclareLaunchArgument(
        "nav_rviz", default_value="True",
        description="Launch Nav2 RViz, which displays the live /map and Nav2 Goal tool.",
    )
    declare_chassis_device = DeclareLaunchArgument(
        "chassis_device",
        description="Explicit STM32 USART1 serial device path; use a PTY for offline tests.",
    )

    # Single lidar input. Do not start pointcloud_fusion in this launch.
    livox_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_share, "launch", "msg_MID360_launch.py")
        )
    )

    # This project applies the physical installation extrinsic in
    # MID360_config.json. The identity TF is the existing logical frame alias.
    base_to_livox = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_livox_tf",
        output="log",
        arguments=[
            "0", "0", "0", "0", "0", "0", "base_frame", "livox_frame"
        ],
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

    terrain_analysis = Node(
        package="terrain_analysis",
        executable="terrainAnalysis",
        name="terrain_analysis",
        output="screen",
        parameters=[
            os.path.join(terrain_share, "config", "terrain_analysis_plus.yaml")
        ],
    )

    # LaserScan must be expressed in the moving base frame for slam_toolbox.
    pointcloud_to_laserscan = Node(
        package="pointcloud_tools",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        output="screen",
        remappings=[("cloud_in", "/terrain_map"), ("scan", "/scan")],
        parameters=[{
            "target_frame": "base_frame",
            "transform_tolerance": 0.1,
            "queue_size": 20,
            "min_height": -0.25,
            "max_height": 1.6,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.0043,
            # Metadata for the real 10 Hz input.  slam_toolbox throttling is
            # configured separately in slam_toolbox_single_lidar.yaml.
            "scan_time": 0.1,
            "range_min": 0.35,
            "range_max": 10.0,
            "use_inf": True,
            "inf_epsilon": 1.0,
        }],
    )

    # Publishes the live OccupancyGrid /map and map -> odom_init TF.
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            os.path.join(
                bringup_share, "config", "slam_toolbox_single_lidar.yaml"
            ),
            {"use_sim_time": use_sim_time},
        ],
    )

    # Match the simulation design: Point-LIO supplies odom_init -> base_frame,
    # while this makes the online map start at Point-LIO's initial pose.  The
    # slam_toolbox config has transform_publish_period: 0.0, so this is the
    # sole publisher of map -> odom_init.
    map_to_odom_init = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="online_map_to_odom_init_tf",
        output="log",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "map", "--child-frame-id", "odom_init",
        ],
    )

    # This server lets the user save the live /map without starting a
    # map_server (map_server would incorrectly try to publish a prebuilt map).
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "save_map_timeout": 5.0,
            "free_thresh_default": 0.25,
            "occupied_thresh_default": 0.65,
            "map_subscribe_transient_local": True,
        }],
    )

    map_saver_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_online_map_saver",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_saver"],
        }],
    )

    # The copied navigation config contains stale absolute paths to custom BT
    # XML files that are not present in this workspace.  Replace them only for
    # this launch with the portable Humble Nav2 recovery trees; the dual-LiDAR
    # reference configuration remains untouched.
    nav2_params = ReplaceString(
        source_file=os.path.join(bringup_share, "config", "nav2_params.yaml"),
        replacements={
            "/home/eatwhat/nav_ws/src/bringup/behavior_tree/"
            "navigate_to_point_w_recovery.xml": os.path.join(
                nav2_bt_share,
                "behavior_trees",
                "navigate_to_pose_w_replanning_and_recovery.xml",
            ),
            "/home/eatwhat/nav_ws/src/bringup/behavior_tree/"
            "navigate_through_pose_w_recovery.xml": os.path.join(
                nav2_bt_share,
                "behavior_trees",
                "navigate_through_poses_w_replanning_and_recovery.xml",
            ),
        },
    )

    # Do not start map_server or alignment in this launch.  slam_toolbox is
    # the sole publisher of /map and the static node above is the sole
    # publisher of map -> odom_init.
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_share, "launch", "bringup_rm_navigation.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "map": "",
            "params_file": nav2_params,
            "nav_rviz": nav_rviz,
        }.items(),
    )

    chassis_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(chassis_bridge_share, "launch", "serial.launch.py")
        ),
        launch_arguments={"device_name": chassis_device}.items(),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_nav_rviz,
        declare_chassis_device,
        livox_driver,
        base_to_livox,
        imu_transform,
        imu_filter,
        point_lio,
        terrain_analysis,
        pointcloud_to_laserscan,
        slam_toolbox,
        map_to_odom_init,
        map_saver,
        map_saver_lifecycle,
        navigation,
        chassis_bridge,
    ])
