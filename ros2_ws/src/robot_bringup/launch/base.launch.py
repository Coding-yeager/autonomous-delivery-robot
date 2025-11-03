import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    default_robot_params = os.path.abspath(os.path.join(pkg_bringup, '..', '..', '..', 'config', 'robot_params.yaml'))

    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    params_file = LaunchConfiguration('params_file')

    declare_use_mock = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Run base controller in simulated/mock mode'
    )
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_robot_params,
        description='Full path to robot_params.yaml'
    )

    base_controller_node = Node(
        package='robot_base',
        executable='base_controller_node',
        name='robot_base_controller',
        output='screen',
        parameters=[params_file, {'use_mock_hardware': use_mock_hardware}]
    )

    return LaunchDescription([
        declare_use_mock,
        declare_params,
        base_controller_node
    ])
