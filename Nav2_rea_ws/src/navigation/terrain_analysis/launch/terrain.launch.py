from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='terrain_analysis',
            executable='terrain_analysis',
            name='terrain_analysis',
            output='screen',
            parameters=[{
                'scanVoxelSize': 0.05,
                'decayTime': 1.0,
                'noDecayDis': 1.75,
                'clearingDis': 8.0,
                'useSorting': True,
                'quantileZ': 0.17,
                'considerDrop': False,
                'limitGroundLift': False,
                'maxGroundLift': 0.15,
                'clearDyObs': True,
                'minDyObsDis': 0.4,
                'minDyObsAngle': 0.0,
                'minDyObsRelZ': -0.3,
                'absDyObsRelZThre': 0.2,
                'minDyObsVFOV': -28.0,
                'maxDyObsVFOV': 33.0,
                'minDyObsPointNum': 1,
                'noDataObstacle': False,
                'noDataBlockSkipNum': 0,
                'minBlockPointNum': 5,
                'vehicleHeight': 1.5,
                'voxelPointUpdateThre': 100,
                'voxelTimeUpdateThre': 1.0,
                'minRelZ': -1.5,
                'maxRelZ': 0.3,
                'disRatioZ': 0.35,
            }]
        )
    ])
