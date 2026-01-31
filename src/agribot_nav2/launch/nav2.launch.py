from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_nav2 = FindPackageShare('nav2_bringup')
    pkg_this = FindPackageShare('agribot_nav2')

    map_yaml = LaunchConfiguration('map')
    params   = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([pkg_this, 'maps', 'room.yaml'])
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([pkg_this, 'config', 'nav2_params_real.yaml'])
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pkg_nav2, 'launch', 'bringup_launch.py'])
            ),
            launch_arguments={
                'use_sim_time': 'false',
                'autostart': 'true',
                'map': map_yaml,
                'params_file': params
            }.items()
        ),
    ])
