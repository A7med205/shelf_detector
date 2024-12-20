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
        get_package_share_directory('localization_server'),
        'config',
        LaunchConfiguration('map_file')
    ])

    DeclareLaunchArgument(
        'amcl_yaml',
        default_value='amcl_config_sim.yaml'
    )
    
    amcl_yaml = PathJoinSubstitution([
        get_package_share_directory('localization_server'),
        'config',
        LaunchConfiguration('amcl_yaml')
    ])

    DeclareLaunchArgument(
        'sim_time',
        default_value= 'True'
    )

    rviz_config_file = os.path.join(get_package_share_directory('localization_server'), 'config', 'localization_config.rviz')


    return LaunchDescription([

        #Node(
        #    package='rviz2',
        #    executable='rviz2',
        #    name='rviz2',
        #    output='screen',
        #    arguments=['-d', rviz_config_file]
        #),


        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename':map_file}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_yaml]
        ),

        # Static transform from "map" to "odom"
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher_map_to_odom',
            output='log',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('sim_time')},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        )
    ])