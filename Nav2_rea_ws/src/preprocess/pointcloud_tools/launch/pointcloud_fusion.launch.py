from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('pointcloud_tools')
    node_params = os.path.join(pkg_dir, 'config', 'pointcloud_fusion.yaml')

    cmd_pointcloud_fusion = Node(
        package    = 'pointcloud_tools',
        executable = 'pointcloud_fusion_node',
        name       = 'pointcloud_fusion',
        parameters = [node_params],
        output     = 'screen',
    )

    return LaunchDescription([
        cmd_pointcloud_fusion
    ])