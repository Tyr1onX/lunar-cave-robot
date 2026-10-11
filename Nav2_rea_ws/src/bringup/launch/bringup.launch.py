import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    pck_bringup_path   = get_package_share_directory('bringup')
    pck_livox_path     = get_package_share_directory('livox_ros_driver2')
    pck_nav2_path      = get_package_share_directory('rm_navigation')
    pck_pointlio_path  = get_package_share_directory('point_lio')
    pck_fastlio_path   = get_package_share_directory('fast_lio')
    pck_dlio_path      = get_package_share_directory('dlio')
    pck_pcl_tools_path = get_package_share_directory('pointcloud_tools')
    pck_imu_tools_path = get_package_share_directory('imu_tools')
    pck_alignment_path = get_package_share_directory('alignment')
    pck_terrain_path = get_package_share_directory('terrain_analysis')

    nav2_map_path = os.path.join(pck_bringup_path, 'map', 'nanling.yaml')

    nav2_params_path = os.path.join(pck_bringup_path, 'config', 'nav2_params.yaml')

    terrain_params = os.path.join(pck_bringup_path, 'config', 'terrain_params.yaml')


    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value = 'False',
        description   = 'Use simulation (Gazebo) clock if true'
    )

    ############################## Drivers ##############################
    cmd_livox_ros_driver2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pck_livox_path, 'launch', 'msg_multi_MID360_launch.py')),
    )

    ############################## Preprocess ##############################
    cmd_pointcloud_fusion = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pck_pcl_tools_path, 'launch', 'pointcloud_fusion.launch.py')),
    )

    cmd_imu_transform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pck_imu_tools_path, 'launch', 'imu_transform.launch.py')),
    )


    cmd_imu_complementary_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pck_imu_tools_path, 'launch', 'imu_complementary_filter.launch.py')),
    )

    ############################## Navigation ##############################
    cmd_navigation2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pck_nav2_path, 'launch', 'bringup_rm_navigation.py')),
        launch_arguments = {
            'use_sim_time': use_sim_time,
            'map': nav2_map_path,
            'params_file': nav2_params_path,
            'nav_rviz': 'True'}.items()
    )

    cmd_fake_vel_transform = Node(
        package    = 'fake_vel_transform',
        executable = 'fake_vel_transform_node',
        output     = 'screen',
        parameters = [{
            'use_sim_time': use_sim_time,
            'spin_speed': 0.0 # rad/s
        }],
        respawn = True
    )

    cmd_terrain_analysis = Node(
        package    = "terrain_analysis",
        executable = "terrain_analysis",
        name       = "terrain_analysis",
        output     = "screen",
        parameters = [terrain_params],
        respawn    = True,
        respawn_delay = 2.0,
    )

    cmd_terrain = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pck_terrain_path, 'launch', 'terrain_analysis_plus.launch.py')),
    )

    cmd_map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pck_nav2_path, 'launch', 'map_server_launch.py')),
            launch_arguments = {
                'use_sim_time': use_sim_time,
                'map': nav2_map_path,
                'params_file': nav2_params_path,
                'container_name': 'nav2_container'}.items()
    )

    ############################## Localization ##############################
    cmd_dlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pck_dlio_path, 'launch', 'dlio.launch.py')),
    )

    cmd_fast_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pck_fastlio_path, 'launch', 'point_lio.launch.py')),
    )

    cmd_point_lio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pck_pointlio_path, 'launch', 'point_lio.launch.py')),
    )

    cmd_alignment = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            pck_alignment_path, 'launch', 'alignment.launch.py')),
    )

    ############################## TF2 ##############################
    cmd_tf_base_to_livox = Node(
        package    = "tf2_ros",
        executable = "static_transform_publisher",
        output     = "log" ,
        arguments  = ["0", "0", "0", "0", "0", "0", "base_frame", "livox_frame"],
        respawn = True
    )

    cmd_tf_init = Node(
        package    = "tf2_ros",
        executable = "static_transform_publisher",
        output     = "log" ,
        arguments  = ["0", "0", "0", "0", "0", "0", "map", "odom_init"],    # po and homee
        # arguments  = ["-0.55", "0.45", "0", "0", "0", "0", "map", "odom_init"],    # baolei
        # arguments  = ["-0.5", "0", "0", "0", "0", "0", "map", "odom_init"],    # baolei
        # arguments  = ["-0.58", "-1.25", "0", "0", "0", "0", "map", "odom_init"],
        # arguments  = ["13.01", "0.82", "0", "3.14", "0", "0", "map", "odom_init"],
        respawn = True
    )

    cmd_pointcloud_to_laserscan = Node(
        package    = "pointcloud_tools",
        executable = "pointcloud_to_laserscan_node",
        name       = "pointcloud_to_laserscan",
        output     = "log" ,
        remappings = [('cloud_in',  ['/terrain_map']),
                      ('scan',  ['/scan'])],
        parameters = [{
            'target_frame': 'odom_init',
            'transform_tolerance': 0.01,
            'min_height': -0.25,
            'max_height': 1.6,
            'angle_min': -3.14159,  # -M_PI/2
            'angle_max': 3.14159,   # M_PI/2
            'angle_increment': 0.0043,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.35,
            'range_max': 10.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)

    # ld.add_action(cmd_livox_ros_driver2)
    
    # ld.add_action(cmd_tf_init)
    ld.add_action(cmd_tf_base_to_livox)
    
    # ld.add_action(cmd_pointcloud_fusion)
    ld.add_action(cmd_imu_transform)
    ld.add_action(cmd_imu_complementary_filter)
    
    ld.add_action(cmd_alignment)

    # ld.add_action(cmd_dlio)
    # ld.add_action(cmd_fast_lio)
    ld.add_action(cmd_point_lio)

    ld.add_action(cmd_navigation2)
    # ld.add_action(cmd_terrain_analysis)
    ld.add_action(cmd_terrain)
    ld.add_action(cmd_fake_vel_transform)
    ld.add_action(cmd_map_server)

    # ld.add_action(cmd_pointcloud_to_laserscan)

    return ld
