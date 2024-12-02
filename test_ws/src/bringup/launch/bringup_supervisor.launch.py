from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    supervisor_node = Node(
        package='supervisor_package',
        executable='supervisor_node',
        name='supervisor_node',
        output='screen'
    )

    return LaunchDescription([
        supervisor_node
    ])