#! /usr/bin/python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Pasar argumentos desde la terminal
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    zed_rotation_node = Node(
        package='rover_motor_controller',
        executable='zed2_rotation',
        name='zed2_rotation_node',
    )

    # ================= LANZAR NODOS =================
    return LaunchDescription([
        zed_rotation_node
    ])
