#! /usr/bin/python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Pasar argumentos desde la terminal
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    path_camara2 = LaunchConfiguration('path_camara2')
    path_camara2_arg = DeclareLaunchArgument(
        'path_camara2',
        description="Device file for the second camera",
        default_value='/dev/video2'
    )

    camara2_nodo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camara_logitech_2',
        parameters=[{'video_device': path_camara2}],
        remappings=[('/image_raw/compressed', 
                     '/camara_logitech_2/image_raw/compressed')]
    )
    
    return LaunchDescription([
        path_camara2_arg,
        camara2_nodo
    ])
