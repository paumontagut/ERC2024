#! /usr/bin/python3

import os
from launch import LaunchDescription
from launch_ros.actions import Node

# Pasar argumentos desde la terminal
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ruedas_node = Node(
        package='rover_motor_controller',
        # executable='motor_controller',
        executable='motor_vel_controller',
        name='motor_vel_controller_node',
        arguments=['/dev/ruedas'],
    )

    # ================= LANZAR NODOS =================
    # Return the arguments for external use and launch selected nodes
    return LaunchDescription([
        ruedas_node
    ])
