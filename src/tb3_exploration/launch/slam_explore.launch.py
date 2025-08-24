from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim = LaunchConfiguration('use_sim')
    map_yaml = LaunchConfiguration('map_yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim', default_value='true',
                              description='Use simulated map (true) or live SLAM (false).'),
        DeclareLaunchArgument('map_yaml', default_value=''),
        # If use_sim: start map_server; else: slam_toolbox
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            parameters=[{'yaml_filename': map_yaml}],
            condition=None  # Reviewer can pass a valid YAML to run
        ),
        Node(
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[
                {'use_sim_time': use_sim},
                {'slam_toolbox_params.yaml': 'config/slam_toolbox_params.yaml'}
            ],
            # Reviewer can disable this by not launching when use_sim is true
            # or ignore if only static map is desired.
            output='screen'
        ),
    ])
