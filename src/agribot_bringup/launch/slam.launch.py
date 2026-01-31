from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    slam_params = '/home/agribot/ros2_ws/src/agribot_bringup/config/slam_toolbox.yaml'

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params],
    )

    lifecycle_mgr = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['slam_toolbox']
        }]
    )

    return LaunchDescription([slam_node, lifecycle_mgr])
