"""Unitree Go2-W bringup using one unified navigation configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('pct_scan_navigation'),
        'config',
        'unitree_go2w',
        'navigation.yaml',
    ])
    base_launch = PathJoinSubstitution([
        FindPackageShare('pct_scan_navigation'),
        'launch',
        'local_pct_scan_navigation.launch.py',
    ])
    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value=''),
        DeclareLaunchArgument('navigation_mode', default_value=''),
        DeclareLaunchArgument('start_open3d_loc', default_value=''),
        DeclareLaunchArgument('start_pct_planner', default_value=''),
        DeclareLaunchArgument('start_go2_bridge', default_value=''),
        DeclareLaunchArgument('network_interface', default_value=''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'config_profile': 'unitree_go2w',
                'config_file': LaunchConfiguration('config_file'),
                'navigation_mode': LaunchConfiguration('navigation_mode'),
                'start_go2_bridge': LaunchConfiguration('start_go2_bridge'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'start_open3d_loc': LaunchConfiguration('start_open3d_loc'),
                'start_pct_planner': LaunchConfiguration('start_pct_planner'),
                'network_interface': LaunchConfiguration('network_interface'),
            }.items(),
        ),
    ])
