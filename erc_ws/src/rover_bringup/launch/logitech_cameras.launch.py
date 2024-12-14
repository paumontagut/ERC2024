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
    
    # Camera 1 path
    path_camara1_arg = DeclareLaunchArgument(
        'path_camara1',
        description="Device file for the first camera",
        default_value='/dev/video0'
    )

    # Camera 2 path
    path_camara2_arg = DeclareLaunchArgument(
        'path_camara2',
        description="Device file for the second camera",
        default_value='/dev/video2'
    )

    # Enable Camera 1
    enable_camara1_arg = DeclareLaunchArgument(
        'enable_camara1',
        description="Enable the first camera (true/false)",
        default_value='true'
    )

    # Enable Camera 2
    enable_camara2_arg = DeclareLaunchArgument(
        'enable_camara2',
        description="Enable the second camera (true/false)",
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
    # Camera 1 Node
    camara1_nodo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camara_logitech_1',    # Node name
        parameters=[
            {'video_device': LaunchConfiguration('path_camara1')},
            {'image_size': LaunchConfiguration('image_size')},
            {'time_per_frame': LaunchConfiguration('time_per_frame')},
            {'pixel_format': 'YUYV'},
        ],
        

        remappings=[('/image_raw/compressed', '/camara_logitech_1/image_raw/compressed')],
        condition=IfCondition(LaunchConfiguration('enable_camara1'))  # Launch condition based on enable_camara1
    )

    # Camera 2 Node
    camara2_nodo = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camara_logitech_2',
        parameters=[{'video_device': LaunchConfiguration('path_camara2')},
                    {'image_size': LaunchConfiguration('image_size')},
                    {'time_per_frame': LaunchConfiguration('time_per_frame')},
                    {'pixel_format': 'YUYV'}],
        remappings=[('/image_raw/compressed', '/camara_logitech_2/image_raw/compressed')],
        condition=IfCondition(LaunchConfiguration('enable_camara2'))
    )

    # ================= LANZAR NODOS =================
    # Return the arguments for external use and launch selected nodes
    return LaunchDescription([
        path_camara1_arg,
        path_camara2_arg,
        enable_camara1_arg,
        enable_camara2_arg,
        resolution_arg,
        frame_rate_arg,
        camara1_nodo,
        camara2_nodo
    ])
