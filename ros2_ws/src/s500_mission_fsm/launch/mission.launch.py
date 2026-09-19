import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('s500_mission_fsm')
    params_file = os.path.join(pkg_share, 'config', 'mission_params.yaml')

    mission_node = Node(
        package='s500_mission_fsm',
        executable='mission_node',
        name='s500_mission_fsm_node',
        output='screen',
        parameters=[params_file],
        remappings=[
            # Gerekirse MAVROS namespace remapping eklenebilir
        ]
    )

    return LaunchDescription([
        mission_node
    ])
