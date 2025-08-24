
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    map_yaml = LaunchConfiguration('map_yaml')

    pkg_share = get_package_share_directory('tb3_exploration')
    slam_params = os.path.join(pkg_share, 'config', 'slam_toolbox_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim', default_value='true',
            description='If true: use static map_server. If false: run live SLAM (slam_toolbox).'
        ),
        DeclareLaunchArgument(
            'map_yaml', default_value='',
            description='Path to a YAML map. If provided, map_server will load it.'
        ),

        # Static map server when using a prebuilt map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml}],
            condition=IfCondition(use_sim)
        ),

        # Live SLAM when not using simulation / static map
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[slam_params, {'use_sim_time': use_sim}],
            output='screen',
            condition=UnlessCondition(use_sim)
        ),
    ])

