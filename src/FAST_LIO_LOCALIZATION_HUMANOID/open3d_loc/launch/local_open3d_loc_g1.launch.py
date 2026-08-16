# ============================================================================
# 文件：local_open3d_loc_g1.launch.py
# 说明：宇树 G1 机器人 open3d_loc 局部定位节点启动入口：
#       同时拉起全局定位节点 global_localization_node 与
#       定位服务节点 localization_service_node，并设置 ROS 日志目录。
# ============================================================================
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import glob
import os
import time


LOG_KEEP_DAYS = 7
LOG_MAX_FILES = 50


# 清理 open3d_loc/log 目录下的旧日志：保留最近 LOG_KEEP_DAYS 天、
# 最多 LOG_MAX_FILES 个文件，防止日志无限膨胀。
def clean_open3d_logs(log_dir):
    if not os.path.isdir(log_dir):
        return

    now = time.time()
    keep_seconds = LOG_KEEP_DAYS * 24 * 60 * 60
    log_files = []

    for name in os.listdir(log_dir):
        path = os.path.join(log_dir, name)
        if not os.path.isfile(path):
            continue
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            continue
        if now - mtime > keep_seconds:
            try:
                os.remove(path)
            except OSError:
                pass
            continue
        log_files.append((mtime, path))

    log_files.sort(reverse=True)
    for _, path in log_files[LOG_MAX_FILES:]:
        try:
            os.remove(path)
        except OSError:
            pass


# 定位源目录解析与日志目录准备 -> 声明 use_sim_time ->
# 以 loc_param_g1_local.yaml 为参数启动全局定位节点与定位服务节点。
def generate_launch_description():
    # 获取包路径
    open3d_loc_share = get_package_share_directory('open3d_loc')
    workspace_root = open3d_loc_share.split('/install/')[0] if '/install/' in open3d_loc_share else ''
    source_candidates = glob.glob(os.path.join(workspace_root, 'src', '**', 'open3d_loc'), recursive=True)
    open3d_loc_dir = source_candidates[0] if source_candidates else open3d_loc_share
    log_dir = os.path.join(open3d_loc_dir, 'log')
    os.makedirs(log_dir, exist_ok=True)
    clean_open3d_logs(log_dir)

    ros_log_dir = SetEnvironmentVariable('ROS_LOG_DIR', log_dir)

    # 声明 use_sim_time 参数
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    # 配置文件路径
    config_file = os.path.join(open3d_loc_share, 'config', 'loc_param_g1_local.yaml')

    # 全局定位节点
    global_localization_node = Node(
        package='open3d_loc',
        executable='global_localization_node',
        name='global_localization_node',
        output='both',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    localization_service_node = Node(
        package='open3d_loc',
        executable='localization_service_node',
        name='localization_service_node',
        output='both',
        parameters=[
            config_file,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ]
    )

    return LaunchDescription([
        ros_log_dir,
        use_sim_time_arg,
        global_localization_node,
        localization_service_node,
    ])
