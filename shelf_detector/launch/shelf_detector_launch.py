from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare the mode argument
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='0',
        description='Mode parameter for both nodes'
    )

    mode = LaunchConfiguration('mode')

    return LaunchDescription([
        mode_arg,
        Node(
            package='shelf_detector',
            executable='attach_service',
            name='attach_service',
            parameters=[{'mode': mode}],
        ),
        Node(
            package='shelf_detector',
            executable='shelf_client',
            name='shelf_client',
            parameters=[{'mode': mode}],
        ),
    ])
