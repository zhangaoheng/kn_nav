#!/usr/bin/env python3
# ============================================================================
# check_navigation_config.py
# ----------------------------------------------------------------------------
# 统一配置校验工具：不启动任何 ROS 节点，仅离线校验一份 navigation.yaml。
#
# 校验项：
#   * 结构：version==1，launch/topics/maps/nodes 四段为字典。
#   * launch：navigation_mode 必须为 1 或 2；initial_map_name 必须存在于 maps。
#   * maps：每个地图 profile 必须含非空 pcd_path / tomo_path。
#   * nodes：必需节点集合齐全且每项为字典；默认地图路径与定位/规划器
#     配置里的 pcd_path / tomo_path 必须一致（避免"启动加载错地图"）。
#   * 数值：扫描/控制/桥接的关键速度与网格参数必须为正。
#
# 用法：python3 check_navigation_config.py <config_file>
# 退出码 0 表示通过，1 表示校验失败。
# ============================================================================

"""Validate one unified navigation.yaml without starting ROS nodes."""

import argparse
from pathlib import Path

import yaml


# 统一配置中必须出现的节点集合（launch 会按开关启动这些节点）。
REQUIRED_NODES = {
    'fastlio_mapping',
    'global_localization_node',
    'localization_service_node',
    'pct_global_planner',
    'scan_planner_node',
    'closed_loop_controller',
    'pct_scan_coordinator',
    'nav_manager_node',
    'go2_cmd_vel_bridge',
}


# 逐项校验配置并收集错误列表；返回空列表表示通过。
def validate(config):
    errors = []
    if not isinstance(config, dict):
        return ['YAML root must be a dictionary']
    if config.get('version') != 1:
        errors.append('version must be 1')
    for section in ('launch', 'topics', 'maps', 'nodes'):
        if not isinstance(config.get(section), dict):
            errors.append(f'{section} must be a dictionary')
    if errors:
        return errors

    launch = config['launch']
    mode = launch.get('navigation_mode')
    if mode not in (1, 2):
        errors.append('launch.navigation_mode must be 1 or 2')

    initial_map = launch.get('initial_map_name')
    maps = config['maps']
    if not initial_map or initial_map not in maps:
        errors.append(
            'launch.initial_map_name must reference an entry in maps')
    for map_name, profile in maps.items():
        if not isinstance(profile, dict):
            errors.append(f'maps.{map_name} must be a dictionary')
            continue
        for key in ('pcd_path', 'tomo_path'):
            if not profile.get(key):
                errors.append(f'maps.{map_name}.{key} must not be empty')

    nodes = config['nodes']
    missing = REQUIRED_NODES - set(nodes)
    if missing:
        errors.append(f'nodes is missing: {", ".join(sorted(missing))}')
    for node_name in REQUIRED_NODES & set(nodes):
        if not isinstance(nodes[node_name], dict):
            errors.append(f'nodes.{node_name} must be a dictionary')

    if initial_map in maps:
        profile = maps[initial_map]
        localization = nodes.get('global_localization_node', {})
        global_planner = nodes.get('pct_global_planner', {})
        if localization.get('path_map') != profile.get('pcd_path'):
            errors.append(
                'default map pcd_path differs from '
                'nodes.global_localization_node.path_map')
        if global_planner.get('tomo_path') != profile.get('tomo_path'):
            errors.append(
                'default map tomo_path differs from '
                'nodes.pct_global_planner.tomo_path')

    planner = nodes.get('scan_planner_node', {})
    controller = nodes.get('closed_loop_controller', {})
    bridge = nodes.get('go2_cmd_vel_bridge', {})
    positive_values = {
        'nodes.scan_planner_node.grid_map.resolution':
            planner.get('grid_map.resolution'),
        'nodes.scan_planner_node.grid_map.double_cylinder_radius':
            planner.get('grid_map.double_cylinder_radius'),
        'nodes.scan_planner_node.manager.max_vel':
            planner.get('manager.max_vel'),
        'nodes.closed_loop_controller.max_vx':
            controller.get('max_vx'),
        'nodes.go2_cmd_vel_bridge.max_vx':
            bridge.get('max_vx'),
    }
    for name, value in positive_values.items():
        if not isinstance(value, (int, float)) or value <= 0:
            errors.append(f'{name} must be greater than 0')
    return errors


# CLI 入口：解析参数 -> 加载 YAML（解析失败即 INVALID 退出）-> 校验 ->
# 通过则打印 mode/map/bridge 摘要，失败则逐条列出错误并退出码 1。
def main():
    parser = argparse.ArgumentParser(
        description='Validate a unified PCT + SCAN navigation YAML')
    parser.add_argument('config_file', type=Path)
    args = parser.parse_args()

    config_path = args.config_file.expanduser().resolve()
    try:
        with config_path.open('r', encoding='utf-8') as handle:
            config = yaml.safe_load(handle)
    except (OSError, yaml.YAMLError) as error:
        raise SystemExit(f'INVALID: {config_path}: {error}') from error

    errors = validate(config)
    if errors:
        print(f'INVALID: {config_path}')
        for error in errors:
            print(f'  - {error}')
        raise SystemExit(1)

    launch = config['launch']
    print(f'VALID: {config_path}')
    print(
        f'  mode={launch["navigation_mode"]}, '
        f'map={launch["initial_map_name"]}, '
        f'bridge={launch.get("start_go2_bridge", False)}, '
        f'interface={launch.get("network_interface", "")}')


if __name__ == '__main__':
    main()
