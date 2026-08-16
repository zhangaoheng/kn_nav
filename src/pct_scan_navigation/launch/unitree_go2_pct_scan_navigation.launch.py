# ============================================================================
# unitree_go2_pct_scan_navigation.launch.py
# ----------------------------------------------------------------------------
# 宇树 Go2 机型入口 launch：把参数透传给统一的
# local_pct_scan_navigation.launch.py 完成实际节点组装。
#
# 职责：
#   * 固定 config_profile=unitree_go2，默认配置文件取自安装树
#     config/unitree_go2/navigation.yaml。
#   * 透传 navigation_mode / start_go2_bridge / use_sim_time 等全部开关参数。
# ============================================================================

"""Unitree Go2 bringup using one unified navigation configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# 组装 LaunchDescription：声明可覆盖参数后，Include 底层统一 launch，
# 并把自身的 LaunchConfiguration 原样转发（config_profile 固定为 unitree_go2）。
def generate_launch_description():
    default_config = PathJoinSubstitution([
        FindPackageShare('pct_scan_navigation'),
        'config',
        'unitree_go2',
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
                'config_profile': 'unitree_go2',
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
