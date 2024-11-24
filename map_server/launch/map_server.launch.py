import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():

    DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map_sim.yaml'
    )
    
    map_file = PathJoinSubstitution([
        get_package_share_directory('map_server'),
        'config',
        LaunchConfiguration('map_file')
    ])
    rviz_config_file = os.path.join(get_package_share_directory('map_server'), 'config', 'map_server_config.rviz')

    return LaunchDescription([
        
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename': map_file}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['map_server']}]
        )
    ])