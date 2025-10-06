import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('robot_localization')
    
    ekf_local_config = os.path.join(pkg_share, 'config', 'ekf_local.yaml')
    ekf_global_config = os.path.join(pkg_share, 'config', 'ekf_global.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_gps_fusion = LaunchConfiguration('enable_gps_fusion')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    declare_gps_fusion = DeclareLaunchArgument(
        'enable_gps_fusion',
        default_value='true',
        description='Enable global EKF and NavSat transform nodes for GPS fusion'
    )

    # 1. Local EKF (publishes odom -> base_footprint and /odometry/filtered/local)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[ekf_local_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered/local')]
    )

    # 2. Global EKF (publishes map -> odom and /odometry/filtered/global)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[ekf_global_config, {'use_sim_time': use_sim_time}],
        remappings=[('odometry/filtered', '/odometry/filtered/global')]
    )

    # 3. NavSat Transform Node (converts GPS + IMU -> /odometry/gps)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[navsat_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('imu', '/imu/data'),
            ('gps/fix', '/gps/fix'),
            ('odometry/filtered', '/odometry/filtered/local'),
            ('odometry/gps', '/odometry/gps')
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_gps_fusion,
        ekf_local_node,
        ekf_global_node,
        navsat_transform_node
    ])
