from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    description_share = get_package_share_directory("agribot_bringup")
    gazebo_launch = os.path.join(description_share, "launch", "gazebo.launch.py")
    description_path = get_package_share_directory("agribot_description")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch),
        launch_arguments={
            # Many setups pass use_sim_time through. If your gazebo.launch.py doesn't
            # declare it, it will be ignored.
            "use_sim_time": "true"
        }.items(),
    )
    # Controller spawners (these call controller_manager services)
    # IMPORTANT: names must match your agribot_controller.yaml
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_base_controller",
            "--controller-manager", "/controller_manager",
        ],
        output="screen",
    )
     # Why TimerAction? Because Gazebo + gz_ros2_control + controller_manager need a moment
    # to exist and advertise services. Without a delay, spawners randomly fail.
    spawn_controllers = TimerAction(
        period=3.0,
        actions=[
            joint_state_broadcaster_spawner,
            diff_drive_spawner,
        ],
    )
    ekf_node = Node(
  package="robot_localization",
  executable="ekf_node",
  name="ekf_filter_node",
  output="screen",
  parameters=[os.path.join(description_path, "config", "ekf.yaml")]
)


    return LaunchDescription([
        gazebo,
        spawn_controllers,
        ekf_node,
    ])