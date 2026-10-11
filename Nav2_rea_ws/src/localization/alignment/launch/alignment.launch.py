import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    alignment_pck_path = get_package_share_directory('alignment')
    alignment_params = os.path.join(alignment_pck_path, 'config', 'config.yaml')

    # container = ComposableNodeContainer(
    #     name       = 'alignment_container',
    #     namespace  = '',
    #     package    = 'rclcpp_components',
    #     executable = 'component_container_mt',
    #     composable_node_descriptions = [
    #         ComposableNode(
    #             package    = 'alignment',
    #             plugin     = 'tars_alignment::InitPoseAlignment',
    #             name       = 'init_pose_alignment',
    #             parameters = [alignment_params]
    #         ),
    #         ComposableNode(
    #             package    = 'alignment',
    #             plugin     = 'tars_alignment::InitTFPublisher',
    #             name       = 'init_tf_publisher',
    #             parameters = [alignment_params]
    #         )
    #     ],
    #     output='screen', 
    # )
    # return LaunchDescription([container])

    cmd_pose_alignment = Node(
            package="alignment",
            executable="pose_alignment_node",
            name="init_pose_alignment",
            parameters=[alignment_params],
            output="screen",
        )

    cmd_tf_publisher = Node(
        package="alignment",
        executable="tf_publisher_node",
        name="init_tf_publisher",
        parameters=[alignment_params],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(cmd_pose_alignment)
    ld.add_action(cmd_tf_publisher)

    return ld

