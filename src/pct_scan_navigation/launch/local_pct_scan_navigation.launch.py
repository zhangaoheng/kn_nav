# ============================================================================
# local_pct_scan_navigation.launch.py
# ----------------------------------------------------------------------------
# PCT 全局规划 + SCAN 局部规划的统一启动入口（A2/Go2/Go2-W 共用的底层 launch）。
#
# 职责：
#   * 从统一配置文件 navigation.yaml（config_profile 指定机型目录）读取全部
#     节点参数并启动：FAST-LIO、open3d 定位、PCT 全局规划、SCAN 规划、
#     closed_loop_controller、pct_scan_coordinator、nav_manager、go2 桥。
#   * 关键 launch 参数：navigation_mode（1=RViz 目标直连 SCAN；
#     2=完整 PCT 参考路径链路）、config_profile、start_go2_bridge、
#     start_open3d_loc、start_pct_planner、start_global_relocalization。
#   * 统一规划 ROS 日志目录 run_YYYYmmdd_HHMMSS，并维护 latest 软链接。
#
# 数据流（Mode 2）：
#   RViz goal -> pct_global_planner(/pct_path) -> pct_scan_coordinator
#   (/scan_planner/waypoints) -> scan_planner_node -> closed_loop_controller
#   (/cmd_vel) -> go2_cmd_vel_bridge -> 机器人。
#   定位链路：fastlio_mapping(/Odometry_loc) -> global_localization_node
#   (/Odometry_open3d) -> SCAN 规划与闭环控制。
# ============================================================================

"""Bring up PCT + SCAN navigation from one unified YAML configuration."""

import os
import shutil
import time
from pathlib import Path

import launch.logging
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable, Shutdown
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# 计算日志根目录：优先取环境变量 KN_NAV_WS_LOG_DIR，
# 否则从 install 树反推工作区 src/log，保证日志落在源码树而不是 install 目录。
def _workspace_log_root():
    override = os.environ.get('KN_NAV_WS_LOG_DIR')
    if override:
        return Path(override).expanduser().resolve()

    share = Path(get_package_share_directory('pct_scan_navigation')).resolve()
    parts = share.parts
    if 'install' in parts:
        workspace = Path(*parts[:parts.index('install')])
        return workspace / 'src' / 'log'
    if share.name == 'pct_scan_navigation' and share.parent.name == 'src':
        return share.parent / 'log'
    return share.parent / 'log'


# 创建本次运行的日志目录并软链 latest：ROS 日志、节点控制台日志都汇入其中；
# 只保留最近 20 次运行，旧的 run_* 目录会被清理。
def _prepare_log_directory(context):
    launch_log_file = Path(launch.logging.launch_config.log_dir).resolve() / 'launch.log'
    log_root = _workspace_log_root()
    log_root.mkdir(parents=True, exist_ok=True)

    run_dir = log_root / time.strftime('run_%Y%m%d_%H%M%S')
    suffix = 0
    while run_dir.exists():
        suffix += 1
        run_dir = log_root / f"{time.strftime('run_%Y%m%d_%H%M%S')}_{suffix}"
    run_dir.mkdir(parents=True, exist_ok=True)

    console_log_link = run_dir / 'launch.log'
    try:
        console_log_link.symlink_to(launch_log_file)
        print(
            f"[pct_scan_navigation] Full console log: "
            f"{console_log_link} -> {launch_log_file}"
        )
    except OSError as error:
        print(f"[pct_scan_navigation] Unable to link full console log: {error}")

    runs = sorted(
        [path for path in log_root.iterdir() if path.is_dir() and path.name.startswith('run_')],
        key=lambda path: path.stat().st_mtime,
        reverse=True,
    )
    for old_run in runs[20:]:
        shutil.rmtree(old_run, ignore_errors=True)

    latest = log_root / 'latest'
    try:
        if latest.is_symlink() or latest.exists():
            latest.unlink()
        latest.symlink_to(run_dir.name)
    except OSError:
        pass

    print(f"[pct_scan_navigation] ROS logs: {run_dir}")
    launch.logging.launch_config.log_dir = str(run_dir)
    return [
        SetEnvironmentVariable('ROS_LOG_DIR', str(run_dir)),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '0'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
    ]


# 宽松解析布尔型 launch 参数（支持 1/true/yes/on 等写法），解析失败直接报错。
def _bool_value(value, setting_name):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{setting_name} must be true or false, got: {value!r}')


# 取值规则：非空的 launch 命令行覆盖优先，否则回落到统一 YAML 的 launch 段，
# 最后才是函数默认值。这样既能在命令行临时覆盖，又不重复维护两处配置。
def _setting(context, name, config, default):
    """Use a non-empty launch override, otherwise read the unified YAML."""
    override = LaunchConfiguration(name).perform(context).strip()
    return override if override else config.get(name, default)


# 从统一配置的 nodes.<node_name> 段取出某节点的参数字典，
# 缺失或类型错误时抛异常，保证 launch 阶段尽早失败。
def _node_parameters(config, node_name):
    parameters = config.get('nodes', {}).get(node_name)
    if not isinstance(parameters, dict):
        raise ValueError(
            f'unified navigation config requires nodes.{node_name} dictionary')
    return dict(parameters)


# 加载并校验统一配置：要求 version==1，且 launch/topics/maps/nodes 四段齐全。
def _load_unified_config(config_path):
    if not config_path.is_file():
        raise FileNotFoundError(
            f'unified navigation config does not exist: {config_path}')
    with config_path.open('r', encoding='utf-8') as handle:
        config = yaml.safe_load(handle) or {}
    if not isinstance(config, dict):
        raise ValueError('unified navigation config root must be a dictionary')
    if config.get('version') != 1:
        raise ValueError('unified navigation config requires version: 1')
    for section in ('launch', 'topics', 'maps', 'nodes'):
        if not isinstance(config.get(section, {}), dict):
            raise ValueError(
                f'unified navigation config {section} section must be a dictionary')
    return config


# 启动描述核心：读统一配置 -> 解析 launch 开关 -> 按开关组装节点列表。
# 定位组（FAST-LIO/open3d）由 start_open3d_loc 控制；PCT 规划器仅在
# navigation_mode==2 时启动（Mode 1 不需要全局路径）；机器人侧执行器
# （scan_planner/closed_loop_controller/coordinator/nav_manager）始终启动。
def _launch_setup(context):
    config_path = Path(
        LaunchConfiguration('config_file').perform(context)
    ).expanduser().resolve()
    config = _load_unified_config(config_path)
    launch_config = config.get('launch', {})
    topics = config.get('topics', {})

    use_sim_time = _bool_value(
        _setting(context, 'use_sim_time', launch_config, False), 'use_sim_time')
    navigation_mode = int(_setting(context, 'navigation_mode', launch_config, 2))
    if navigation_mode not in (1, 2):
        raise ValueError('navigation_mode must be 1 or 2')
    start_open3d_loc = _bool_value(
        _setting(context, 'start_open3d_loc', launch_config, True),
        'start_open3d_loc')
    start_global_relocalization = _bool_value(
        _setting(context, 'start_global_relocalization', launch_config, False),
        'start_global_relocalization')
    start_pct_planner = _bool_value(
        _setting(context, 'start_pct_planner', launch_config, True),
        'start_pct_planner')
    start_go2_bridge = _bool_value(
        _setting(context, 'start_go2_bridge', launch_config, False),
        'start_go2_bridge')
    network_interface = str(
        _setting(context, 'network_interface', launch_config, 'enp2s0'))
    full_restart_command = str(
        _setting(context, 'full_restart_command', launch_config, ''))
    initial_map_name = str(launch_config.get('initial_map_name', '')).strip()
    if not initial_map_name:
        raise ValueError(
            'unified navigation config launch.initial_map_name is required')
    maps = config.get('maps', {})
    if initial_map_name not in maps:
        raise ValueError(
            'unified navigation config launch.initial_map_name must reference '
            f'an entry in maps: {initial_map_name}')

    common_overrides = {'use_sim_time': use_sim_time}
    actions = []

    if start_open3d_loc:
        actions.extend([
            Node(
                package='fast_lio', executable='fastlio_mapping',
                name='fastlio_mapping', output='both', emulate_tty=True,
                parameters=[
                    _node_parameters(config, 'fastlio_mapping'),
                    common_overrides,
                ],
            ),
            Node(
                package='pct_scan_navigation',
                executable='fastlio_monitor_node.py',
                name='fastlio_monitor', output='both',
                parameters=[
                    dict(config.get('nodes', {}).get('fastlio_monitor', {})),
                    common_overrides,
                ],
            ),
            Node(
                package='pct_scan_navigation',
                executable='imu_timing_probe',
                name='imu_timing_probe', output='both',
                parameters=[
                    dict(config.get('nodes', {}).get('imu_timing_probe', {})),
                    common_overrides,
                ],
            ),
            Node(
                package='open3d_loc', executable='global_localization_node',
                name='global_localization_node', output='both',
                parameters=[
                    _node_parameters(config, 'global_localization_node'),
                    common_overrides,
                ],
            ),
            Node(
                package='open3d_loc', executable='localization_service_node',
                name='localization_service_node', output='both',
                parameters=[
                    _node_parameters(config, 'localization_service_node'),
                    common_overrides,
                ],
            ),
        ])

        if start_global_relocalization:
            actions.append(Node(
                package='open3d_loc', executable='global_relocalization_node',
                name='global_relocalization_node', output='both',
                parameters=[
                    _node_parameters(config, 'global_relocalization_node'),
                    common_overrides,
                ],
            ))

    # Mode 2（PCT 参考路径）才启动全局规划器；Mode 1 下 SCAN 直接
    # 消费 RViz 的 /goal_pose，无需 /pct_path。
    if navigation_mode == 2 and start_pct_planner:
        actions.append(Node(
            package='pct_planner', executable='run_ros2_global_planner',
            name='pct_global_planner', output='both',
            parameters=[
                _node_parameters(config, 'pct_global_planner'),
                common_overrides,
            ],
        ))

    actions.extend([
        Node(
            package='scan_planner', executable='scan_planner_node',
            name='scan_planner_node', output='both',
            parameters=[
                _node_parameters(config, 'scan_planner_node'),
                {'fsm.navi_mode': navigation_mode, **common_overrides},
            ],
            # scan_planner 话题重映射统一取自配置 topics 段（缺省值保证旧行为
            # 不变），例如机器人位姿 body_pose/sensor_pose 默认都是 /Odometry_open3d。
            remappings=[
                ('body_pose', topics.get('body_pose', '/Odometry_open3d')),
                ('sensor_pose', topics.get('sensor_pose', '/Odometry_open3d')),
                ('cloud', topics.get('cloud', '/scan_map')),
                ('move_base_simple/goal', topics.get('goal', '/goal_pose')),
                ('waypoints', topics.get('waypoints', '/scan_planner/waypoints')),
                ('initial_path', topics.get('initial_path', '/initial_path')),
            ],
        ),
        Node(
            package='scan_planner', executable='closed_loop_controller',
            name='closed_loop_controller', output='both',
            parameters=[
                _node_parameters(config, 'closed_loop_controller'),
                common_overrides,
            ],
            remappings=[
                ('body_pose', topics.get('body_pose', '/Odometry_open3d')),
            ],
        ),
        Node(
            package='pct_scan_navigation', executable='pct_scan_coordinator',
            name='pct_scan_coordinator', output='both',
            parameters=[
                _node_parameters(config, 'pct_scan_coordinator'),
                {'mode': navigation_mode, **common_overrides},
            ],
            # coordinator 异常退出时触发整体 Shutdown，避免机器人停在半路。
            on_exit=Shutdown(reason='pct_scan_coordinator exited'),
        ),
        Node(
            package='pct_scan_navigation', executable='nav_manager_node.py',
            name='nav_manager_node', output='both',
            parameters=[
                _node_parameters(config, 'nav_manager_node'),
                {
                    'navigation_config_path': str(config_path),
                    'initial_map_name': initial_map_name,
                    'full_restart_command': full_restart_command,
                    **common_overrides,
                },
            ],
        ),
    ])

    # go2 桥（cmd_vel -> 宇树机器人底盘）按需启动，通常实机开启、仿真关闭。
    if start_go2_bridge:
        actions.append(Node(
            package='pure_pursuit_planner', executable='go2_cmd_vel_bridge',
            name='go2_cmd_vel_bridge', output='both',
            parameters=[
                _node_parameters(config, 'go2_cmd_vel_bridge'),
                {
                    'network_interface': network_interface,
                    **common_overrides,
                },
            ],
        ))

    print(f'[pct_scan_navigation] Unified config: {config_path}')
    print(
        f'[pct_scan_navigation] mode={navigation_mode}, map={initial_map_name}, '
        f'bridge={start_go2_bridge}, interface={network_interface}'
    )
    return actions


# launch 入口：声明全部可覆盖参数（config_profile/config_file/各开关），
# 先准备日志目录，再注入宇树 SDK 动态库路径，最后执行 _launch_setup 组装节点。
def generate_launch_description():
    navigation_share = FindPackageShare('pct_scan_navigation')
    config_profile = LaunchConfiguration('config_profile')
    default_config = PathJoinSubstitution([
        navigation_share, 'config', config_profile, 'navigation.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('config_profile', default_value='local'),
        DeclareLaunchArgument('config_file', default_value=default_config),
        DeclareLaunchArgument('use_sim_time', default_value=''),
        DeclareLaunchArgument(
            'navigation_mode', default_value='',
            description='1: direct RViz goal to SCAN, 2: complete PCT reference path',
        ),
        DeclareLaunchArgument('start_open3d_loc', default_value=''),
        DeclareLaunchArgument('start_global_relocalization', default_value=''),
        DeclareLaunchArgument('start_pct_planner', default_value=''),
        DeclareLaunchArgument('start_go2_bridge', default_value=''),
        DeclareLaunchArgument('network_interface', default_value=''),
        DeclareLaunchArgument('full_restart_command', default_value=''),
        OpaqueFunction(function=_prepare_log_directory),
        SetEnvironmentVariable('LD_LIBRARY_PATH', [
            '/opt/unitree_robotics/lib:',
            EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
        ]),
        OpaqueFunction(function=_launch_setup),
    ])
