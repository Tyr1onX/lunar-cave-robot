from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    cmd_imu_complementary_filter = Node(
        package    = 'imu_tools',
        executable = 'imu_complementary_filter_node',
        name       = 'imu_complementary_filter',
        output     = 'screen',
        parameters = [
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ],
        remappings = [
            ('/imu/data_raw', '/livox/imu_aft_tf'),
        ]
    )

    return LaunchDescription([
        cmd_imu_complementary_filter
    ])