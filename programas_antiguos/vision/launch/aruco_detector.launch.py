from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vision',
            executable='aruco_detector',
            output='screen',
            parameters=[
                {'aruco_dict': 'DICT_4X4_250'},
                {'camera_activation': True},
            ]),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            output='screen',
            parameters=[
                
            ]),
    ])
