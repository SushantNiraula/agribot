from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    esp_port   = LaunchConfiguration('esp_port')
    esp_baud   = LaunchConfiguration('esp_baud')
    lidar_port = LaunchConfiguration('lidar_port')
    lidar_baud = LaunchConfiguration('lidar_baud')
    ekf_yaml   = LaunchConfiguration('ekf_yaml')

    use_lidar  = LaunchConfiguration('use_lidar')
    use_imu    = LaunchConfiguration('use_imu')

    sllidar_launch = os.path.join(
        get_package_share_directory('sllidar_ros2'),
        'launch',
        'sllidar_a1_launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument('esp_port',   default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('esp_baud',   default_value='115200'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB1'),
        DeclareLaunchArgument('lidar_baud', default_value='115200'),
        DeclareLaunchArgument('ekf_yaml',   default_value='/home/agribot/agribot/src/state_estimation/config/ekf.yaml'),
        DeclareLaunchArgument('use_lidar',  default_value='true'),
        DeclareLaunchArgument('use_imu',    default_value='true'),

        # micro-ROS agent (no extra sourcing needed if ~/.bashrc already sourced microros_ws)
        ExecuteProcess(
            cmd=[
                'bash', '-lc',
                'ros2 run micro_ros_agent micro_ros_agent serial --dev "$ESP_PORT" -b "$ESP_BAUD"'
            ],
            additional_env={
                'ESP_PORT': esp_port,
                'ESP_BAUD': esp_baud
            },
            output='screen'
        ),

        # LiDAR launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch),
            condition=IfCondition(use_lidar),
            launch_arguments={
                'serial_port': lidar_port,
                'serial_baudrate': lidar_baud
            }.items()
        ),

        # Delay these so agent + lidar are up
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    condition=IfCondition(use_imu),
                    package='mpu6050_imu',
                    executable='mpu6050_node',
                    name='mpu6050_node',
                    output='screen'
                ),
                Node(
                    package='robot_localization',
                    executable='ekf_node',
                    name='ekf_filter_node',
                    output='screen',
                    parameters=[ekf_yaml]
                ),
                Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    name='base_to_laser_tf',
                    output='screen',
                    arguments=['0.10','0.00','0.20','0','0','0','base_link','laser']
                ),
            ]
        ),
    ])
