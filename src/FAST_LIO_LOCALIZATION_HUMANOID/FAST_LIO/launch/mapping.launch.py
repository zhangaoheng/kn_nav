# ============================================================
# FAST-LIO 建图/里程计启动脚本（ROS 2 launch，上游开源算法配套）
# 作用：启动 fastlio_mapping 主节点（ESKF 里程计 + 点云配准建图），
#       并可选启动 RViz 可视化；日志统一重定向到 FAST_LIO/log 目录。
# 工作区内作为机器人前端连续里程计的启动入口使用。
# ============================================================
import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition

from launch_ros.actions import Node


# 生成 launch 描述：解析包路径与日志目录、声明命令行参数
# （use_sim_time / config_path / config_file / rviz / rviz_cfg），
# 组装 fastlio_mapping 主节点与可选 rviz2 节点后返回。
def generate_launch_description():
    package_path = get_package_share_directory('fast_lio')
    workspace_root = package_path.split('/install/')[0] if '/install/' in package_path else ''
    source_path = os.path.join(workspace_root, 'src', 'FAST_LIO_LOCALIZATION_HUMANOID', 'FAST_LIO')
    fast_lio_dir = source_path if os.path.isdir(source_path) else package_path
    log_dir = os.path.join(fast_lio_dir, 'log')
    os.makedirs(log_dir, exist_ok=True)

    default_config_path = os.path.join(package_path, 'config')
    default_rviz_config_path = os.path.join(
        package_path, 'rviz', 'fastlio.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    config_path = LaunchConfiguration('config_path')
    config_file = LaunchConfiguration('config_file')
    rviz_use = LaunchConfiguration('rviz')
    rviz_cfg = LaunchConfiguration('rviz_cfg')

    ros_log_dir = SetEnvironmentVariable('ROS_LOG_DIR', log_dir)

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path', default_value=default_config_path,
        description='Yaml config file path'
    )
    decalre_config_file_cmd = DeclareLaunchArgument(
        'config_file', default_value='mid360.yaml',
        # 'config_file', default_value='avia.yaml',
        description='Config file'
    )
    declare_rviz_cmd = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Use RViz to monitor results'
    )
    declare_rviz_config_path_cmd = DeclareLaunchArgument(
        'rviz_cfg', default_value=default_rviz_config_path,
        description='RViz config file path'
    )

    # FAST-LIO 主节点：加载 yaml 配置文件并启动里程计与建图流程
    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[PathJoinSubstitution([config_path, config_file]),
                    {'use_sim_time': use_sim_time}],
        output='both',
        emulate_tty=True
    )
    # RViz 可视化节点定义（带 IfCondition 条件），当前未加入动作列表
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_cfg],
        condition=IfCondition(rviz_use)
    )

    ld = LaunchDescription()
    ld.add_action(ros_log_dir)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)
    ld.add_action(decalre_config_file_cmd)
    ld.add_action(declare_rviz_cmd)
    ld.add_action(declare_rviz_config_path_cmd)

    ld.add_action(fast_lio_node)
    # ld.add_action(rviz_node)

    return ld
