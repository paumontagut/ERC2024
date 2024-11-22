#! /usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # TODO: Acabar
    # Ver argumentos posibles para pasar al bringup desde la terminal:
    # ros2 launch bringup robot_bringup.launch.py --show-args
    # ros2 launch bringup robot_bringup.launch.py usb_port:=/dev/ttyACM0 ...
    
    ############## QUÉ ES ESTO FOUNDERS?????? ################
    
    # usb_port = LaunchConfiguration('usb_port')
    # usb_port_launch_arg = DeclareLaunchArgument(
    #     'usb_port',
    #     default_value='/dev/ttyACM0'
    # )
    
    ############## Logitech Camera ##############
    # TODO: Configuración parámetros cámara, resolución, frames... en un yaml?
    
    LEFT_CAMERA_PATH = '/dev/video0'
    RIGHT_CAMERA_PATH = '/dev/video2'
    
    logitech_camera_launch = IncludeLaunchDescription(
        # Aquí se llama al logitech_cameras.launch.py y se le pasan los argumentos que hagan falta. 
        PythonLaunchDescriptionSource(
            # Se une el path del paquete + carpeta launch + el launch que queremos lanzar.
            os.path.join(get_package_share_directory('camera_management'), 'launch', 'logitech_cameras.launch.py')
        ),
        launch_arguments={
            'path_camara1': LEFT_CAMERA_PATH,
            'path_camara2': RIGHT_CAMERA_PATH
        }.items()
    )
    
    ############### Realsense Camera ##############
    # TODO: Falta testear
    
    realsense_align_depth = LaunchConfiguration('camera_align_depth')
    realsense_pointcloud = LaunchConfiguration('camera_pointcloud')
    realsense_depth = LaunchConfiguration('camera_depth')

    realsense_align_depth_launch_arg = DeclareLaunchArgument(
        'camera_align_depth',
        default_value='False'
    )
    realsense_pointcloud_launch_arg = DeclareLaunchArgument(
        'camera_pointcloud',
        default_value='False'
    )
    realsense_depth_launch_arg = DeclareLaunchArgument(
        'camera_depth',
        default_value='True'
    )

    realsense_camera_launch = IncludeLaunchDescription(
        # Usamos el launch ya creado en realsense2_camera, a diferencia de lo que se hizo con las Logitech.
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': realsense_align_depth,
            'pointcloud.enable': realsense_pointcloud,
            'enable_depth': realsense_depth
        }.items()
    )

    ############## Unitree Lidar ##################
    # TODO: Testear ft. Pau. Poner ficheros de ros2_ws de la jetson aquí, agregar configuración rviz para visualizar...
    PUERTO_USB_LIDAR = '/dev/ttyUSB0'
    
    unitree_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('unitree_lidar_ros2'), 'launch', 'launch.py')
        ),
        launch_arguments={
            'serial_port': PUERTO_USB_LIDAR
        }.items()
    )

    ############## SIYI CAMERA ##############
    # TODO: Falta implementar
    
    ############## SEMÁFORO ##################
    # TODO: Falta implementar
    
    
    return LaunchDescription([
        # TODO: Descomentar para probar
        logitech_camera_launch,
        # realsense_camera_launch,
        # lidar_node,
        # siyi_camera_node
        # semaforo_launch_arg,
        # usb_port_launch_arg,
    ])
