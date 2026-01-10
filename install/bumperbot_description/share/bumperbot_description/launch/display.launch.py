from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

def generate_launch_description():
    pkg_share = get_package_share_directory('bumperbot_description')

    default_model_path = os.path.join(pkg_share, 'urdf', 'bumperbot.urdf.xacro')
    default_rviz_config = os.path.join(pkg_share, "config", 'display.rviz')

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=default_model_path,
        description="Absolute path to robot xacro file"
    )

    gui_arg = DeclareLaunchArgument(
        name = "gui",
        default_value="true",
        description= "Enable joint_state_publisher_gui"
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str
    )

    ##NOdes

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description":robot_description
        }]
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        condition = UnlessCondition(LaunchConfiguration("gui"))
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition = IfCondition(LaunchConfiguration("gui"))
    )
    rviz_node= Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", default_rviz_config],
        output= "screen"
    )

    return LaunchDescription([
        model_arg,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])