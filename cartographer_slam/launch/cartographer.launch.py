from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    cartographer_config_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'config')
    rviz_config_file = os.path.join(get_package_share_directory('cartographer_slam'), 'config', 'cartographer_config.rviz')
    DeclareLaunchArgument(
        'sim_time',
        default_value= 'True'
    )

    DeclareLaunchArgument(
        'cartographer_config',
        default_value='cartographer_config_sim.lua'
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        ),
    
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('sim_time')}],
            arguments=['-configuration_directory', cartographer_config_dir,
                   '-configuration_basename', LaunchConfiguration('cartographer_config')]
        )
    ])