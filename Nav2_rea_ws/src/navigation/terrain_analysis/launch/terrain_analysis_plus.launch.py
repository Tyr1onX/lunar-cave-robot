import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    terrain_pck_path = get_package_share_directory('terrain_analysis')
    terrain_grid_params = os.path.join(terrain_pck_path, 'config', 'terrain_analysis_plus.yaml')

    return LaunchDescription([
        Node(
            package='terrain_analysis',
            executable='terrain_analysis',
            name='terrain_analysis',
            output='screen',
            parameters=[terrain_grid_params]
        )
    ])
