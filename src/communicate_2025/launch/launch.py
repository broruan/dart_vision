import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('communicate_2025'),
        'config', 'config.yaml'
    )

    load_nodes=GroupAction(
        actions=[
            Node(
                package='communicate_2025',
                executable='communicate_2025_node',
                output='screen',
                parameters=[config]
            ),
        ]
    )
    ld = LaunchDescription()
    ld.add_action(load_nodes)
    return ld