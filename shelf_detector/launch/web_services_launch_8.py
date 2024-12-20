import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare a launch argument for the server directory
    web_dir_arg = DeclareLaunchArgument(
        'web_dir',
        default_value='/home/user/webpage_ws/detector_app',
        description='Directory from which to run the HTTP server'
    )

    # Use a LaunchConfiguration substitution for the directory
    web_dir = LaunchConfiguration('web_dir')

    return LaunchDescription([
        web_dir_arg,

        # HTTP server using the provided directory
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '7000'],
            cwd=web_dir,
            output='screen'
        ),

        # ROSBridge websocket server
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            parameters=[{'port': 8080}],
        ),

        # tf_relay.py script
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/ros2_ws/src/shelf_detector/shelf_detector/scripts/tf_relay.py')],
            output='screen'
        ),

        # voice_processor.py script
        ExecuteProcess(
            cmd=['python3', os.path.expanduser('~/ros2_ws/src/shelf_detector/shelf_detector/scripts/voice_processor.py')],
            output='screen'
        ),
    ])