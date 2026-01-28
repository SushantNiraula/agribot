from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('state_estimation')
    params = os.path.join(pkg, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[params],
            remappings=[
                # leave as-is unless your topics differ
                ('/odometry/filtered', '/odometry/filtered'),
            ]
        )
    ])
