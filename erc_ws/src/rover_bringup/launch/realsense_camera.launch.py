#! /usr/bin/python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Pasar argumentos desde la terminal
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    path_camara3 = LaunchConfiguration('path_camara3')
    path_camara3_arg = DeclareLaunchArgument(
        'path_camara3',
        description="Device file for the third camera",
        default_value='/dev/video4'
    )

    # Camera 3 Node
    camara3_nodo = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='camara_realsense',
    )

    # ================= LANZAR NODOS =================
    # Return the arguments for external use and launch selected nodes
    return LaunchDescription([
        path_camara3_arg,
        camara3_nodo
    ])
