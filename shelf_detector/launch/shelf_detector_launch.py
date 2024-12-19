from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='shelf_detector',
            executable='attach_service',
            name='attach_service',
            parameters=[{'mode': 1}],
        ),
        Node(
            package='shelf_detector',
            executable='shelf_client',
            name='shelf_client',
            parameters=[{'mode': 1}],
        ),
    ])