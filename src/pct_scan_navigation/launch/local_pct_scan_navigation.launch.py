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


def _bool_value(value, setting_name):
    if isinstance(value, bool):
        return value
    normalized = str(value).strip().lower()
    if normalized in ('1', 'true', 'yes', 'on'):
        return True
    if normalized in ('0', 'false', 'no', 'off'):
        return False
    raise ValueError(f'{setting_name} must be true or false, got: {value!r}')


def _setting(context, name, config, default):
    """Use a non-empty launch override, otherwise read the unified YAML."""
    override = LaunchConfiguration(name).perform(context).strip()
    return override if override else config.get(name, default)


def _node_parameters(config, node_name):
    parameters = config.get('nodes', {}).get(node_name)
    if not isinstance(parameters, dict):
        raise ValueError(
            f'unified navigation config requires nodes.{node_name} dictionary')
    return dict(parameters)


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
