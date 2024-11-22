#! /usr/bin/python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # TODO: arreglar topics, los comentados son pq no se si se usan
    
    usb_port = LaunchConfiguration('usb_port')
    semaforo_topic = LaunchConfiguration('semaforo')

    usb_port_launch_arg = DeclareLaunchArgument(                          
        'usb_port',
        default_value = '/dev/ttyACM0'
    )
    
    semaforo_launch_arg = DeclareLaunchArgument(
        'semaforo_topic',
        default_value='/semaforo',
        description="Topic to control the emergency lamp"
    )
    
    
    ############## Logitech Camera Node ####
    
    # TODO: Haciendo, revisar que vaya
    logitech_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('camera_management'), 'launch', 'logitech_cameras.launch.py')
        ),
        launch_arguments={
            'video_device_0': '/dev/video0',
            'video_device_2': '/dev/video2'
        }.items()
    )

    logitech_cameras_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='logitech_camera',
        parameters=[{'video_device': '/dev/video0'}]  # Adjust device path as needed
    )

    
    ########### Realsense Cámera ###################
    
    camera_align_depth = LaunchConfiguration('camera_align_depth')
    camera_pointcloud = LaunchConfiguration('camera_pointcloud')
    camera_depth = LaunchConfiguration('camera_depth')
    
    
    camera_align_depth_launch_arg = DeclareLaunchArgument(
        'camera_align_depth',
        default_value='False',
        description="Enable alignment of depth with color"
    )

    camera_pointcloud_launch_arg = DeclareLaunchArgument(
        'camera_pointcloud',
        default_value='False',
        description="Enable pointcloud streaming"
    )

    camera_depth_launch_arg = DeclareLaunchArgument(
        'camera_depth',
        default_value='True',
        description="Enable depth camera"
    )

    realsense_camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_launch.py']),
            
        launch_arguments = {
            'align_depth.enable': camera_align_depth,
            'pointcloud.enable': camera_pointcloud,
            'enable_depth': camera_depth
        }.items(),
    )
    
    ############## LIDAR ##################
    
    # TODO: esto ya estaba así, está bien?
    lidar_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('sllidar_ros2'), 'launch'),
            '/view_sllidar_s2_launch.py'])
    )


    ############## SIYI CAMERA ##############
    
    # TODO: Falta implementar
    siyi_camera_node = Node(
        package='siyi_camera',
        executable='siyi_camera_node',
        name='siyi_camera',
        parameters=[{'camera_topic': '/siyi_camera/image_raw'}]
    )
    
    ############## UNITREE CONTROL ###########
    
    # TODO: ns que es esto aun, falta ver esto, esto es del chat
    unitree_node = Node(
        package='unitree_ros',
        executable='unitree_control',
        name='unitree_controller',
        parameters=[{'cmd_vel_topic': '/cmd_vel'}]
    )
    
    ############## SEMÁFORO ##################

    # TODO: Revisar bien
    semaforo_node = Node(
        package='semaforo_control',
        executable='semaforo_node',
        name='semaforo',
        parameters=[{'semaforo_topic': semaforo_topic}]
    )


    return LaunchDescription(
        logitech_camera_launch,
        
        usb_port_launch_arg,
        semaforo_launch_arg,
        camera_align_depth_launch_arg,
        camera_pointcloud_launch_arg,
        camera_depth_launch_arg,
        realsense_camera_node,
        lidar_node,
        siyi_camera_node,
        unitree_node,
        semaforo_node
    )
