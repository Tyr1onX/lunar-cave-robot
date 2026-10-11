from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_dir = get_package_share_directory('imu_tools')
    node_params = os.path.join(pkg_dir, 'config', 'imu_transform.yaml')

    cmd_imu_transform = Node(
        package    = 'imu_tools',
        executable = 'imu_transform_node',
        name       = 'imu_transform',
        parameters = [node_params],
        output     = 'screen',
    )

    return LaunchDescription([
        cmd_imu_transform
    ])