import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_bringup = get_package_share_directory('robot_bringup')
    pkg_desc = get_package_share_directory('robot_description')
    pkg_loc = get_package_share_directory('robot_localization_config')
    pkg_nav = get_package_share_directory('robot_navigation')

    default_xacro_path = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    default_rviz_path = os.path.join(pkg_desc, 'rviz', 'robot_view.rviz')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    enable_localization = LaunchConfiguration('enable_localization')
    enable_navigation = LaunchConfiguration('enable_navigation')
    use_rviz = LaunchConfiguration('use_rviz')

    # Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )
    declare_use_mock_hardware = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Use mock hardware drivers (safe for dev workstations)'
    )
    declare_enable_localization = DeclareLaunchArgument(
        'enable_localization',
        default_value='true',
        description='Start Dual EKF and NavSat transform nodes'
    )
    declare_enable_navigation = DeclareLaunchArgument(
        'enable_navigation',
        default_value='true',
        description='Start Nav2 navigation stack'
    )
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 for visualization'
    )

    # 1. Robot State Publisher (URDF / TF tree)
    robot_description = ParameterValue(
        Command(['xacro ', default_xacro_path]),
        value_type=str
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # 2. Robot Base (Motor Controller, Kinematics, Watchdog, Odometry)
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'base.launch.py')),
        launch_arguments={'use_mock_hardware': use_mock_hardware}.items()
    )

    # 3. Sensors (RPLIDAR, MPU-6050, Holybro NEO-M9N, Health Monitor)
    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_bringup, 'launch', 'sensors.launch.py')),
        launch_arguments={'use_mock_hardware': use_mock_hardware}.items()
    )

    # 4. Localization (Dual EKF + NavSat Transform)
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_loc, 'launch', 'localization.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(enable_localization)
    )

    # 5. Navigation (Nav2 + Costmap Keepout Filter for Stairs)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_nav, 'launch', 'navigation.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(enable_navigation)
    )

    # 6. RViz2 (Optional, disabled by default for Raspberry Pi headless operation)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', default_rviz_path],
        condition=IfCondition(use_rviz)
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_mock_hardware,
        declare_enable_localization,
        declare_enable_navigation,
        declare_use_rviz,
        robot_state_publisher_node,
        base_launch,
        sensors_launch,
        localization_launch,
        navigation_launch,
        rviz_node
    ])
