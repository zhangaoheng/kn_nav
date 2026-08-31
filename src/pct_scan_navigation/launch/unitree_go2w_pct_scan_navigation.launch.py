# ============================================================================
# unitree_go2w_pct_scan_navigation.launch.py
# ----------------------------------------------------------------------------
# 宇树 Go2-W 机型入口 launch：把参数透传给统一的
# local_pct_scan_navigation.launch.py 完成实际节点组装。
#
# 职责：
#   * 固定 config_profile=unitree_go2w。
#   * 默认 config_file 指向 /home/code/work_space/kn_nav/src/pct_scan_navigation/config/unitree_go2w/navigation.yaml，
#     运行时配置以 /home/code/work_space/kn_nav/src/pct_scan_navigation/config 为唯一数据源。
#   * 透传 navigation_mode / start_go2_bridge / use_sim_time 等全部开关参数。
# ============================================================================

"""Unitree Go2-W bringup using one unified navigation configuration."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


# 组装 LaunchDescription：声明可覆盖参数后，Include 底层统一 launch，
# 并把自身的 LaunchConfiguration 原样转发（config_profile 固定为 unitree_go2w）。
def generate_launch_description():
    # Runtime map/navigation configuration lives outside the ROS install tree.
    # This keeps /home/code/work_space/kn_nav/src/pct_scan_navigation/config as
    # the single source of truth and avoids stale generated copies under install/.
    default_config = '/home/code/work_space/kn_nav/src/pct_scan_navigation/config/unitree_go2w/navigation.yaml'
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
        DeclareLaunchArgument('start_global_relocalization', default_value=''),
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
                'start_global_relocalization': LaunchConfiguration('start_global_relocalization'),
                'start_pct_planner': LaunchConfiguration('start_pct_planner'),
                'network_interface': LaunchConfiguration('network_interface'),
            }.items(),
        ),
    ])
