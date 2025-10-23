import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav = get_package_share_directory('robot_navigation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_params_file = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    
    # Paths to default maps
    default_map_path = os.path.abspath(os.path.join(pkg_nav, '..', '..', '..', 'maps', 'iit_mandi_campus.yaml'))
    default_mask_path = os.path.abspath(os.path.join(pkg_nav, '..', '..', '..', 'maps', 'stairs_keepout_mask.yaml'))

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    map_yaml_file = LaunchConfiguration('map')
    mask_yaml_file = LaunchConfiguration('mask')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )
    declare_mask = DeclareLaunchArgument(
        'mask',
        default_value=default_mask_path,
        description='Full path to stairs keepout mask yaml file to load'
    )

    # Nav2 Bringup Launch
    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart
        }.items()
    )

    # Costmap Filter Info Server (Keepout Filter)
    costmap_filter_info_server_node = Node(
        package='nav2_map_server',
        executable='costmap_filter_info_server',
        name='costmap_filter_info_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Filter Mask Server (Publishes /stairs_keepout_mask)
    filter_mask_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='filter_mask_server',
        output='screen',
        parameters=[params_file, {'yaml_filename': mask_yaml_file, 'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_map,
        declare_mask,
        costmap_filter_info_server_node,
        filter_mask_server_node,
        nav2_bringup_cmd
    ])
