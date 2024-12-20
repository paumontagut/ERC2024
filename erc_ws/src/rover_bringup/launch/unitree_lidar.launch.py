import os
import subprocess

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB1',
        description='Serial port for the Unitree LiDAR'
    )

    node1 = Node(
        package='unitree_lidar_ros2',       # está en sensores_actuadores/unitree_lidar_ros2/src
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        output='screen',
        parameters= [
                {'port': LaunchConfiguration('serial_port')},
                {'rotate_yaw_bias': 0.0},
                {'range_scale': 0.001},
                {'range_bias': 0.0},
                {'range_max': 50.0},
                {'range_min': 0.0},
                {'cloud_frame': "unilidar_lidar"},
                {'cloud_topic': "unilidar/cloud"},
                {'cloud_scan_num': 18},
                {'imu_frame': "unilidar_imu"},
                {'imu_topic': "unilidar/imu"}]
    )
    
    return LaunchDescription([
        serial_port_arg,
        node1
    ])
