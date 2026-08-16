#!/usr/bin/env python3
# ============================================================================
# migrate_navigation_config.py
# ----------------------------------------------------------------------------
# 配置迁移工具：把某个机型目录下的 legacy 拆分 YAML（fast_lio.yaml、
# open3d_loc.yaml、scan_planner.yaml、coordinator.yaml、map_profiles.yaml、
# go2_bridge.yaml 等）合并成一份统一 navigation.yaml。
#
# 用途：
#   * 新增/重做机型配置时一键生成统一文件，保证 nodes.<node> 与旧拆分
#     文件内容完全一致（test_navigation_contract.py 会校验这一点）。
#
# 用法：python3 migrate_navigation_config.py <legacy_dir> [-o OUTPUT]
# 默认输出到 LEGACY_DIR/navigation.yaml。
# ============================================================================

"""Merge a legacy pct_scan_navigation profile into one navigation.yaml."""

import argparse
from pathlib import Path

import yaml


# 读取单个 YAML 文件并确保根节点是字典。
def _load(path):
    with path.open('r', encoding='utf-8') as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise ValueError(f'{path} must contain a YAML dictionary')
    return data


# 从 legacy 拆分文件里取出指定节点的 ros__parameters 段；
# fastlio_mapping 兼容 /** 顶层写法（FAST-LIO 默认参数表挂在 /** 下）。
def _ros_parameters(path, node_name):
    data = _load(path)
    node = data.get(node_name)
    if node is None and node_name == 'fastlio_mapping':
        node = data.get('/**')
    if not isinstance(node, dict) or not isinstance(
            node.get('ros__parameters'), dict):
        raise ValueError(f'{path} has no parameters for {node_name}')
    return node['ros__parameters']


# 核心合并逻辑：按统一配置 schema 组装 version/launch/topics/maps/nodes；
# launch 段按机型名推断默认值（local 不开 go2 桥、网卡 enp2s0，其余机型
# 开桥、网卡 eth0）；nav_manager_node 使用固定服务/话题契约。
def build_unified_config(legacy_dir):
    profile_name = legacy_dir.name
    open3d = _load(legacy_dir / 'open3d_loc.yaml')
    scan = _load(legacy_dir / 'scan_planner.yaml')
    global_loc = open3d['global_localization_node']['ros__parameters']

    return {
        'version': 1,
        'launch': {
            'use_sim_time': False,
            'navigation_mode': 2,
            'start_open3d_loc': True,
            'start_pct_planner': True,
            'start_go2_bridge': profile_name != 'local',
            'network_interface': 'enp2s0' if profile_name == 'local' else 'eth0',
            'initial_map_name': global_loc.get('map_name', 'outdoor'),
            'full_restart_command': '',
        },
        'topics': {
            'body_pose': '/Odometry_open3d',
            'sensor_pose': '/Odometry_open3d',
            'cloud': '/scan_map',
            'goal': '/goal_pose',
            'waypoints': '/scan_planner/waypoints',
            'initial_path': '/initial_path',
        },
        'maps': _load(legacy_dir / 'map_profiles.yaml').get('maps', {}),
        'nodes': {
            'fastlio_mapping': _ros_parameters(
                legacy_dir / 'fast_lio.yaml', 'fastlio_mapping'),
            'global_localization_node': global_loc,
            'localization_service_node': (
                open3d['localization_service_node']['ros__parameters']),
            'pct_global_planner': _ros_parameters(
                legacy_dir / 'pct_global_planner.yaml', 'pct_global_planner'),
            'scan_planner_node': (
                scan['scan_planner_node']['ros__parameters']),
            'closed_loop_controller': (
                scan['closed_loop_controller']['ros__parameters']),
            'pct_scan_coordinator': _ros_parameters(
                legacy_dir / 'coordinator.yaml', 'pct_scan_coordinator'),
            'nav_manager_node': {
                'service_timeout_s': 8.0,
                'load_localization_service':
                    '/global_localization_node/load_map',
                'load_tomogram_service': '/pct_global_planner/load_tomogram',
                'coordinator_reset_service':
                    '/pct_scan_coordinator/reset_route',
                'scan_reset_service':
                    '/scan_planner_node/reset_navigation',
                'cmd_vel_topic': '/cmd_vel',
                'waypoints_topic': '/scan_planner/waypoints',
            },
            'go2_cmd_vel_bridge': _ros_parameters(
                legacy_dir / 'go2_bridge.yaml', 'go2_cmd_vel_bridge'),
        },
    }


# CLI 入口：解析 legacy 目录与输出路径，合并后写 navigation.yaml 并打印结果。
def main():
    parser = argparse.ArgumentParser(
        description='Merge one legacy navigation profile into navigation.yaml')
    parser.add_argument('legacy_dir', type=Path)
    parser.add_argument(
        '-o', '--output', type=Path,
        help='Output file (default: LEGACY_DIR/navigation.yaml)')
    args = parser.parse_args()

    legacy_dir = args.legacy_dir.expanduser().resolve()
    output = (
        args.output.expanduser().resolve()
        if args.output else legacy_dir / 'navigation.yaml')
    config = build_unified_config(legacy_dir)
    with output.open('w', encoding='utf-8') as handle:
        handle.write(
            '# Unified PCT + SCAN navigation configuration.\n'
            '# Edit this file instead of the legacy split YAML files.\n')
        yaml.safe_dump(
            config, handle, sort_keys=False, allow_unicode=True,
            default_flow_style=False)
    print(f'generated: {output}')


if __name__ == '__main__':
    main()
