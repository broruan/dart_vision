from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mindvision_camera',
            executable='camera_main',
            namespace='rm_multi_cam',
            name='camera',
            output='both',
            emulate_tty=True,
            parameters=[PathJoinSubstitution([
                FindPackageShare('mindvision_camera'), 'config', 'camera_main.yaml'])
            ],
        ),
    ])