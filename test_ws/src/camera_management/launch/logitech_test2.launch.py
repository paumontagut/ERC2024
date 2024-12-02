#! /usr/bin/python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Pasar argumentos desde la terminal
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    path_camara1 = LaunchConfiguration('path_camara1') 
    path_camara1_arg = DeclareLaunchArgument(
        'path_camara1',
        description="Device file for the first camera",
        default_value='/dev/video4'
    )
    
    camara1_nodo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camara_logitech_1',    # Nombre del nodo
        parameters=[{'video_device': path_camara1}],
        remappings=[('/image_raw/compressed', 
                     '/camara_logitech_1/image_raw/compressed')]
    ) 
    
    return LaunchDescription([
        path_camara1_arg,
        camara1_nodo
    ])
