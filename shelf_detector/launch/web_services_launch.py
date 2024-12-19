import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # HTTP server in the detector_app directory
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '7000'],
            cwd=os.path.expanduser('~/webpage_ws/detector_app'),
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
