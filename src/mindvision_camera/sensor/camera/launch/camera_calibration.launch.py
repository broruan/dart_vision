#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # 声明launch参数
    save_directory_arg = DeclareLaunchArgument(
        'save_directory',
        default_value='./calibration_images',
        description='Directory to save calibration images'
    )
    mindvision_config_path_arg = DeclareLaunchArgument(
        'mindvision_config_path',
        default_value='',
        description='MindVision .config file path'
    )
    sn_arg = DeclareLaunchArgument(
        'sn',
        default_value='',
        description='Camera serial number, empty means first detected camera'
    )
    publish_fps_arg = DeclareLaunchArgument(
        'publish_fps',
        default_value='5.0',
        description='Publishing rate for calibration image topic'
    )

    # 相机节点
    camera_node = Node(
        package='camera',
        executable='camera_for_calibrate',
        name='camera_for_calibrate',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'mindvision_config_path': LaunchConfiguration('mindvision_config_path'),
            'sn': LaunchConfiguration('sn'),
            'publish_fps': LaunchConfiguration('publish_fps'),
        }]
    )
    
    # 标定图像采集节点
    calibration_capture_node = Node(
        package='camera',
        executable='camera_calibration_capture',
        name='camera_calibration_capture',
        output='screen',
        parameters=[{
            'save_directory': LaunchConfiguration('save_directory'),
            'use_sim_time': False,
        }]
    )
    
    return LaunchDescription([
        save_directory_arg,
        mindvision_config_path_arg,
        sn_arg,
        publish_fps_arg,
        camera_node,
        calibration_capture_node,
    ])
