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
    ld.add_action(unitree_lidar())
    
    #ld.add_action(logitech_cameras())
    #ld.add_action(realsense_camera())
    #ld.add_action(zed2_motors())
    return ld
    
# ================= FUNCIONES =================

def ruedas():
    ruedas_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_bringup'), 'launch', 'ruedas.launch.py')
        )
    )
    return ruedas_launch

def logitech_cameras():
    logitech_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_bringup'), 'launch', 'logitech_cameras.launch.py')
        )
    )
    return logitech_camera_launch

def realsense_camera():
    realsense_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_bringup'), 'launch', 'realsense_camera.launch.py')
        )
    )
    return realsense_camera_launch
    
def unitree_lidar():
    unitree_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_bringup'), 'launch', 'unitree_lidar.launch.py')
        )
    )
    return unitree_lidar_launch

def zed2_motors():
    zed2_motors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rover_bringup'), 'launch', 'zed2_motors.launch.py')
        )
    )
    return zed2_motors_launch
