#! /usr/bin/python3
# -*- coding: utf-8 -*-

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ================= MAIN =================
def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(ruedas())
    #ld.add_action(camaras_logitech())
    # ld.add_action(camara_realsense())
    # ld.add_action(lidar_unitree())
    # ld.add_action(camara_siyi())
    # ld.add_action(semaforo())
    return ld
    
# ================= FUNCIONES =================
    
def camaras_logitech():
    logitech_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_bringup'), 'launch', 'logitech_cameras.launch.py')
        )
    )
    
    return logitech_camera_launch

def ruedas():
    ruedas_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_bringup'), 'launch', 'ruedas.launch.py')
        )
    )
    
    return ruedas_launch

def camara_realsense():
    realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_bringup'), 'launch', 'realsense_camera.launch.py')
        )
    )
    
    return realsense_camera_launch
    
def lidar_unitree():
    unitree_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_bringup'), 'launch', 'unitree_lidar.launch.py')
        )
    )

    return unitree_lidar_launch
