#! /usr/bin/python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Pasar argumentos desde la terminal
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # ================= ARGUMENTOS =================
    # Pasados directamente desde robot_bringup.launch.py
    # También se pueden pasar desde la terminal para pruebas de este launch concreto.
    # La descripción y el valor por defecto se pueden ver con `ros2 launch ... --show-args`
    # Se crean 2 cosas: el launchconfiguration (que es el valor que se pasa como tal, se puede usar como argumento para los nodos)...
    # y el declarelaunchargument (que es el hecho de poder recibir un argumento en sí, también necesario)
    
    # Cámara 1
    path_camara1 = LaunchConfiguration('path_camara1') 
    path_camara1_arg = DeclareLaunchArgument(
        'path_camara1',
        description="Device file for the first camera",
        default_value='/dev/video0'
    )
    
    # Cámara 2
    path_camara2 = LaunchConfiguration('path_camara2')
    path_camara2_arg = DeclareLaunchArgument(
        'path_camara2',
        description="Device file for the second camera",
        default_value='/dev/video2'
    )
    
    # ================= NODOS A EJECUTAR =================
    # En este caso, son equivalentes a los nodos que se lanzarían con `ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/video0 -r /image_raw:=/camara_logitech_1/image_raw` o con lo mismo pero cambiando para la cámara 2.
    # El paquete v4l2_camera está en /opt/ros/humble...
    # En los siguientes nodos se modifica el topic donde publica cada cámara, ya que por defecto es siempre /image_raw.
    
    # Cámara 1
    camara1_nodo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camara_logitech_1',    # Nombre del nodo
        parameters=[{'video_device': path_camara1}],
        remappings=[('/image_raw/compressed', '/camara_logitech_1/image_raw/compressed')]
    ) 

    # Cámara 2
    camara2_nodo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camara_logitech_2',
        parameters=[{'video_device': path_camara2}],
        remappings=[('/image_raw/compressed', '/camara_logitech_2/image_raw/compressed')]
    )
    
    # ================= LANZAR NODOS =================
    # Necesario devolver los argumentos para que el robot_bringup pueda pasarlos
    # Y también lanzamos todos los nodos creados
    
    return LaunchDescription([
        path_camara1_arg,
        path_camara2_arg,
        camara1_nodo,
        camara2_nodo
    ])
