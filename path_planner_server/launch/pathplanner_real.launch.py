import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_real.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery_real.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server_real.yaml')
    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller_real.yaml')

    DeclareLaunchArgument(
        'sim_time',
        default_value= 'True'
    )

    DeclareLaunchArgument(
        'teleop_topic',
        default_value= '/cmd_vel'
    )

    return LaunchDescription([

        Node(
            package='nav2_apps',
            executable='approach_service_real',
            name='approach_shelf_service',
            output='screen'),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            remappings=[
            ('/cmd_vel', LaunchConfiguration('teleop_topic'))],
            output='screen',
            parameters=[controller_yaml]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='nav2_behaviors',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanner',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('sim_time')},
                        {'autostart': True},
                        {'node_names': ['planner_server',
                                        'controller_server',
                                        'nav2_behaviors',
                                        'bt_navigator']}])
    ])