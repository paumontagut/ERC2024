#! /usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node

# Pasar argumentos desde la terminal
from launch.actions import DeclareLaunchArgument        # Permite recibir argumentos desde la terminal
from launch.substitutions import LaunchConfiguration    # Convierte el argumento en un valor que se puede usar en los nodos
from launch.conditions import IfCondition               # Para lanzar nodos condicionalmente

def generate_launch_description():
    # ================= ARGUMENTOS =================
    # Pasados directamente desde rover_bringup.launch.py desde la terminal
    # Ver readme.md para más información
    # Se crean 2 cosas: el launchconfiguration (que es el valor que se pasa como tal, se puede usar como argumento para los nodos)...
    # y el declarelaunchargument (que es el hecho de poder recibir un argumento en sí, también necesario)
    # Camera1 -> derecha
    # Camera2 -> izquierda

    # Camera Derecha path
    path_camara_derecha_arg = DeclareLaunchArgument(
        'path_camara_derecha',
        description="Device file for the right camera",
        default_value='/dev/logitech_derecha'
    )

    # Camera Izquierda path
    path_camara_izquierda_arg = DeclareLaunchArgument(
        'path_camara_izquierda',
        description="Device file for the left camera",
        default_value='/dev/logitech_izquierda'
    )

    # Enable Camera Derecha
    enable_camara_derecha_arg = DeclareLaunchArgument(
        'enable_camara_derecha',
        description="Enable the right camera (true/false)",
        default_value='true'
    )

    # Enable Camera 2
    enable_camara_izquierda_arg = DeclareLaunchArgument(
        'enable_camara_izquierda',
        description="Enable the left camera (true/false)",
        default_value='true'
    )
    
    resolution_arg = DeclareLaunchArgument(
        'image_size', 
        default_value='[640, 480]',    # [1280, 720], [1024, 768], [640, 480], [320, 240]
        description='Resolución de la cámara'
    )
    
    frame_rate_arg = DeclareLaunchArgument(     # TODO: No funciona en portátil, probar jetson
        'time_per_frame', 
        default_value='[1, 60]',
        description='Fotogramas por segundo'
    )


    # ================= NODOS A EJECUTAR =================
    camara_derecha_nodo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camara_logitech_derecha',
        parameters=[
            {'video_device': LaunchConfiguration('path_camara_derecha')},
            {'image_size': LaunchConfiguration('image_size')},
            {'time_per_frame': LaunchConfiguration('time_per_frame')},
            {'pixel_format': 'YUYV'},
        ],
        

        remappings=[('/image_raw/compressed', '/camara_logitech_derecha/image_raw/compressed')],
        condition=IfCondition(LaunchConfiguration('enable_camara_derecha'))  # Launch condition based on enable_camara_derecha
    )

    camara_izquierda_nodo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camara_logitech_izquierda',
        parameters=[{'video_device': LaunchConfiguration('path_camara_izquierda')},
                    {'image_size': LaunchConfiguration('image_size')},
                    {'time_per_frame': LaunchConfiguration('time_per_frame')},
                    {'pixel_format': 'YUYV'}],
        remappings=[('/image_raw/compressed', '/camara_logitech_izquierda/image_raw/compressed')],
        condition=IfCondition(LaunchConfiguration('enable_camara_izquierda'))
    )

    # ================= LANZAR NODOS =================
    # Return the arguments for external use and launch selected nodes
    return LaunchDescription([
        path_camara_derecha_arg,
        path_camara_izquierda_arg,
        enable_camara_derecha_arg,
        enable_camara_izquierda_arg,
        resolution_arg,
        frame_rate_arg,
        camara_derecha_nodo,
        camara_izquierda_nodo
    ])
