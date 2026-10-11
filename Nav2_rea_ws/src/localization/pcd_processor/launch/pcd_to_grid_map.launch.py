import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    package_path = get_package_share_directory('pcd_processor')
    params_path = os.path.join(package_path, 'config', 'config.yaml')

    cmd_pcd_to_grid_map = Node(
        package    = "pcd_processor",
        executable = "pcd_to_grid_map",
        name       = "pcd_to_grid_map",
        output     = "screen",
        parameters = [params_path],
    )

    ld = LaunchDescription()
    ld.add_action(cmd_pcd_to_grid_map)

    return ld