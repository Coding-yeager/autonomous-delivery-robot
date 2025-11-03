import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')

    use_rviz = LaunchConfiguration('use_rviz')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'use_mock_hardware': 'true',
            'enable_localization': 'true',
            'enable_navigation': 'true',
            'use_rviz': use_rviz
        }.items()
    )

    return LaunchDescription([
        declare_use_rviz,
        bringup_cmd
    ])
