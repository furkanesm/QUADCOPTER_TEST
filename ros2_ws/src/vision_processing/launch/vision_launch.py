import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('vision_processing')
    config_file = os.path.join(pkg_dir, 'config', 'camera_params.yaml')

    vision_node = Node(
        package='vision_processing',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        vision_node
    ])
