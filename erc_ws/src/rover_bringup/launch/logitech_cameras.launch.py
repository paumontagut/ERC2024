#! /usr/bin/python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Pasar argumentos desde la terminal
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch.conditions import IfCondition

def generate_launch_description():
    # ================= ARGUMENTOS =================
    # Pasados directamente desde rover_bringup.launch.py desde la terminal
    # Ver readme.md para más información
    # Se crean 2 cosas: el launchconfiguration (que es el valor que se pasa como tal, se puede usar como argumento para los nodos)...
    # y el declarelaunchargument (que es el hecho de poder recibir un argumento en sí, también necesario)
    
    # Camera 1 path
    path_camara1 = LaunchConfiguration('path_camara1')
    path_camara1_arg = DeclareLaunchArgument(
        'path_camara1',
        description="Device file for the first camera",
        default_value='/dev/video0'
    )

    # Camera 2 path
    path_camara2 = LaunchConfiguration('path_camara2')
    path_camara2_arg = DeclareLaunchArgument(
        'path_camara2',
        description="Device file for the second camera",
        default_value='/dev/video2'
    )

    # Enable Camera 1
    enable_camara1 = LaunchConfiguration('enable_camara1')
    enable_camara1_arg = DeclareLaunchArgument(
        'enable_camara1',
        description="Enable the first camera (true/false)",
        default_value='true'
    )

    # Enable Camera 2
    enable_camara2 = LaunchConfiguration('enable_camara2')
    enable_camara2_arg = DeclareLaunchArgument(
        'enable_camara2',
        description="Enable the second camera (true/false)",
        default_value='true'
    )

    # ================= NODOS A EJECUTAR =================
    # Camera 1 Node
    camara1_nodo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camara_logitech_1',    # Node name
        parameters=[{'video_device': path_camara1}],
        remappings=[('/image_raw/compressed', '/camara_logitech_1/image_raw/compressed')],
        condition=IfCondition(enable_camara1)  # Launch condition based on enable_camara1
    )

    # Camera 2 Node
    camara2_nodo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camara_logitech_2',
        parameters=[{'video_device': path_camara2}],
        remappings=[('/image_raw/compressed', '/camara_logitech_2/image_raw/compressed')],
        condition=IfCondition(enable_camara2)  # Launch condition based on enable_camara2
    )

    # ================= LANZAR NODOS =================
    # Return the arguments for external use and launch selected nodes
    return LaunchDescription([
        path_camara1_arg,
        path_camara2_arg,
        enable_camara1_arg,
        enable_camara2_arg,
        camara1_nodo,
        camara2_nodo
    ])
