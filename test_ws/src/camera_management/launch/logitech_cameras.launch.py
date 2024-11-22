#! /usr/bin/python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # TODO: Hecho con el chatgpt, falta revisar, entender y organiuzar explicación
    
    video_device_0 = LaunchConfiguration('video_device_0')
    video_device_2 = LaunchConfiguration('video_device_2')

    video_device_0_launch_arg = DeclareLaunchArgument(
        'video_device_0',
        default_value='/dev/video0',
        description="Device file for the first camera"
    )

    video_device_2_launch_arg = DeclareLaunchArgument(
        'video_device_2',
        default_value='/dev/video2',
        description="Device file for the second camera"
    )

    # First camera node
    camera_0_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_0',
        parameters=[{'video_device': video_device_0}],
        remappings=[('/image_raw', '/camera_0/image_raw')]
    )

    # Second camera node
    camera_2_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera_2',
        parameters=[{'video_device': video_device_2}],
        remappings=[('/image_raw', '/camera_2/image_raw')]
    )

    return LaunchDescription([
        video_device_0_launch_arg,
        video_device_2_launch_arg,
        camera_0_node,
        camera_2_node
    ])
