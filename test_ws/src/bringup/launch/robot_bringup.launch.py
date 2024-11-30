#! /usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ================= MAIN =================
# ros2 launch bringup robot_bringup.launch.py --show-args

def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(camaras_logitech())
    ld.add_action(camara_realsense())
    # ld.add_action(ruedas())
    # ld.add_action(lidar_unitree())
    # ld.add_action(camara_siyi())
    # ld.add_action(semaforo())
    return ld
    
# ================= FUNCIONES =================
    
def camaras_logitech():
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
    
    return logitech_camera_launch

def ruedas():
    # TODO: Testear
    motor_controller_node = Node(
        package='bringup',
        executable='motor_controller',
        name='motor_controller_node',
        output='screen'
    )
    
    return motor_controller_node

def camara_realsense():
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
            os.path.join(
                get_package_share_directory('realsense2_camera'), 
                'launch', 
                'rs_launch.py')
        ),
        launch_arguments={
            'align_depth.enable': realsense_align_depth,
            'pointcloud.enable': realsense_pointcloud,
            'enable_depth': realsense_depth
        }.items()
    )
    
    return [
        realsense_align_depth_launch_arg,
        realsense_pointcloud_launch_arg,
        realsense_depth_launch_arg,
        realsense_camera_launch
    ]
    
def lidar_unitree():
    # TODO: Testear ft. Pau. Poner ficheros de ros2_ws de la jetson aquí, agregar configuración rviz para visualizar...
    # USB port for the LIDAR
    PUERTO_USB_LIDAR = '/dev/ttyUSB0'

    # Include the Unitree LIDAR launch file
    unitree_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('unitree_lidar_ros2'),
                'launch',
                'launch.py')
        ),
        launch_arguments={
            'serial_port': PUERTO_USB_LIDAR
        }.items()
    )

    return unitree_lidar_launch
