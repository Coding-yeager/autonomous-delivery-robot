import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    default_sensors_params = os.path.abspath(os.path.join(pkg_bringup, '..', '..', '..', 'config', 'sensors_params.yaml'))

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    params_file = LaunchConfiguration('params_file')

    declare_use_mock = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Run sensors in simulated/mock mode if true'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_sensors_params,
        description='Full path to sensors_params.yaml'
    )

    # 1. RPLIDAR A1M8 Node (Launched only on real hardware)
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[params_file],
        condition=UnlessCondition(use_mock_hardware)
    )

    # 2. MPU-6050 IMU Node
    mpu6050_node = Node(
        package='robot_sensors',
        executable='mpu6050_node',
        name='mpu6050_node',
        output='screen',
        parameters=[params_file, {'use_mock_sensor': use_mock_hardware}]
    )

    # 3. Holybro NEO-M9N GNSS Node
    gps_node = Node(
        package='robot_sensors',
        executable='gps_node',
        name='gps_node',
        output='screen',
        parameters=[params_file, {'use_mock_sensor': use_mock_hardware}]
    )

    # 4. Sensor Health Monitor
    health_monitor_node = Node(
        package='robot_sensors',
        executable='sensor_health_monitor',
        name='sensor_health_monitor',
        output='screen',
        parameters=[params_file]
    )

    return LaunchDescription([
        declare_use_mock,
        declare_params,
        rplidar_node,
        mpu6050_node,
        gps_node,
        health_monitor_node
    ])
