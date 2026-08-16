#!/usr/bin/env python3
# ============================================================================
# nav_manager_node.py
# ----------------------------------------------------------------------------
# 导航运行时管理节点（轻量"看门狗"），三大职责：
#
#   1. 地图切换：/switch_map 服务按 map_profiles 加载定位 PCD 与规划 tomogram，
#      切换前先软重置，成功后更新 /current_map 状态（latched 话题）。
#   2. 软重置 / 重启：/restart_navigation 服务支持 SOFT_RESET（停机器 + 清
#      空规划链路）与 FULL_RESTART（拉起外部重启命令，常用于整系统重启）。
#   3. 定位丢失保护：订阅 /localization_status，一旦进入 TRACKING_LOST 立即
#      软重置停止导航，避免在错误位姿下继续运动。
#
# 数据流：
#   /switch_map -> LoadLocalizationMap(global_localization_node)
#                -> LoadTomogram(pct_global_planner)
#   /restart_navigation / 定位丢失 -> 零速 /cmd_vel + 清 coordinator 路线
#   + 空 /scan_planner/waypoints + Trigger(scan_planner_node 重置)。
# ============================================================================

"""Lightweight navigation runtime manager for PCT + SCAN navigation."""

import os
import shlex
import subprocess
import threading
from typing import Any, Dict, Optional

import rclpy
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger

from pct_scan_navigation.msg import LocalizationStatus, MapStatus
from pct_scan_navigation.srv import (
    LoadLocalizationMap,
    LoadTomogram,
    RestartNavigation,
    SwitchMap,
)


# 运行时管理器节点：统一封装地图切换、软重置/全重启、定位丢失保护。
# 所有服务/话题回调共享 ReentrantCallbackGroup，配合 MultiThreadedExecutor
# 并行处理，避免一个慢服务阻塞其他回调。
class NavManagerNode(Node):
    # 初始化：声明参数 -> 建发布器（/current_map 为 latched、/cmd_vel、
    # /scan_planner/waypoints 为 latched）-> 建四个服务客户端 -> 订阅
    # /localization_status -> 提供 /switch_map 与 /restart_navigation 服务。
    def __init__(self):
        super().__init__('nav_manager_node')

        self.declare_parameter('navigation_config_path', '')
        self.declare_parameter('map_profiles_path', '')
        self.declare_parameter('initial_map_name', '')
        self.declare_parameter('full_restart_command', '')
        self.declare_parameter('service_timeout_s', 8.0)
        self.declare_parameter('load_localization_service', '/global_localization_node/load_map')
        self.declare_parameter('load_tomogram_service', '/pct_global_planner/load_tomogram')
        self.declare_parameter(
            'coordinator_reset_service', '/pct_scan_coordinator/reset_route')
        self.declare_parameter('scan_reset_service', '/scan_planner_node/reset_navigation')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('waypoints_topic', '/scan_planner/waypoints')

        self.navigation_config_path = self.get_parameter(
            'navigation_config_path').value
        self.map_profiles_path = self.get_parameter('map_profiles_path').value
        self.current_map_name = self.get_parameter('initial_map_name').value
        self.full_restart_command = self.get_parameter('full_restart_command').value
        self.service_timeout_s = float(self.get_parameter('service_timeout_s').value)
        self.load_localization_service = self.get_parameter('load_localization_service').value
        self.load_tomogram_service = self.get_parameter('load_tomogram_service').value
        self.coordinator_reset_service = self.get_parameter(
            'coordinator_reset_service').value
        self.scan_reset_service = self.get_parameter('scan_reset_service').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.waypoints_topic = self.get_parameter('waypoints_topic').value
        self.callback_group = ReentrantCallbackGroup()

        latched = QoSProfile(depth=1)
        latched.reliability = ReliabilityPolicy.RELIABLE
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.map_status_pub = self.create_publisher(MapStatus, '/current_map', latched)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.waypoints_pub = self.create_publisher(Path, self.waypoints_topic, latched)

        self.load_localization_cli = self.create_client(
            LoadLocalizationMap, self.load_localization_service,
            callback_group=self.callback_group)
        self.load_tomogram_cli = self.create_client(
            LoadTomogram, self.load_tomogram_service,
            callback_group=self.callback_group)
        self.coordinator_reset_cli = self.create_client(
            Trigger, self.coordinator_reset_service, callback_group=self.callback_group)
        self.scan_reset_cli = self.create_client(
            Trigger, self.scan_reset_service, callback_group=self.callback_group)

        self.create_subscription(
            LocalizationStatus, '/localization_status', self._localization_status_cb,
            10, callback_group=self.callback_group)

        self.create_service(
            SwitchMap, '/switch_map', self._switch_map_cb,
            callback_group=self.callback_group)
        self.create_service(
            RestartNavigation, '/restart_navigation', self._restart_cb,
            callback_group=self.callback_group)

        self._profiles = self._load_profiles()
        self._last_localization_state: Optional[int] = None

        if self.current_map_name:
            self._publish_map_status(MapStatus.LOADED, self.current_map_name, 'startup')
        else:
            self._publish_map_status(MapStatus.UNLOADED, '', 'startup')

        self.get_logger().info(
            'Nav manager ready: profiles='
            f'{self.navigation_config_path or self.map_profiles_path or "<unset>"}')

    # 从统一配置 navigation.yaml 的 maps 段读取全部地图 profile
    # （每项含 pcd_path / tomo_path / localization 阈值），供 /switch_map 使用。
    def _load_profiles(self) -> Dict[str, Any]:
        profile_path = self.navigation_config_path or self.map_profiles_path
        if not profile_path:
            self.get_logger().warn(
                'navigation_config_path and map_profiles_path are empty; '
                '/switch_map will fail')
            return {}
        if not os.path.exists(profile_path):
            self.get_logger().warn(
                f'map profile config does not exist: {profile_path}')
            return {}
        with open(profile_path, 'r', encoding='utf-8') as handle:
            data = yaml.safe_load(handle) or {}
        maps = data.get('maps', {})
        if not isinstance(maps, dict):
            self.get_logger().warn(
                f'{profile_path} has no valid "maps" dictionary')
            return {}
        return maps

    # 带超时的同步式服务调用封装：等待服务上线 -> call_async ->
    # 等待结果或超时；返回 (响应, 错误字符串)，避免阻塞 executor 线程。
    def _call_service(self, client, request):
        if not client.wait_for_service(timeout_sec=self.service_timeout_s):
            return None, f'service unavailable: {client.srv_name}'
        future = client.call_async(request)
        done_event = threading.Event()
        result_box = {'result': None, 'error': ''}

        def _done_callback(done_future):
            try:
                result_box['result'] = done_future.result()
            except Exception as exc:  # pragma: no cover - defensive ROS boundary
                result_box['error'] = f'service error: {client.srv_name}: {exc}'
            finally:
                done_event.set()

        future.add_done_callback(_done_callback)
        if not done_event.wait(timeout=self.service_timeout_s):
            return None, f'service timeout: {client.srv_name}'
        if result_box['error']:
            return None, result_box['error']
        return result_box['result'], ''

    # 地图切换流程：校验 map_name 与路径 -> 发布 LOADING 状态 -> 软重置
    # 停车清路 -> 依次加载定位地图与 tomogram；任一步失败都回滚为 FAILED
    # 状态并返回错误，全部成功才更新 current_map_name 并发布 LOADED。
    def _switch_map_cb(self, request, response):
        profile = self._profiles.get(request.map_name)
        if profile is None:
            response.success = False
            response.message = f'unknown map_name: {request.map_name}'
            self._publish_map_status(MapStatus.FAILED, request.map_name, response.message)
            return response

        pcd_path = str(profile.get('pcd_path', ''))
        tomo_path = str(profile.get('tomo_path', ''))
        if not pcd_path or not tomo_path:
            response.success = False
            response.message = f'map profile {request.map_name} requires pcd_path and tomo_path'
            self._publish_map_status(MapStatus.FAILED, request.map_name, response.message)
            return response

        self._publish_map_status(MapStatus.LOADING, request.map_name, 'switching')
        self._soft_reset(reason='switch_map')

        loc_req = LoadLocalizationMap.Request()
        loc_req.map_name = request.map_name
        loc_req.pcd_path = pcd_path
        loc_cfg = profile.get('localization', {}) or {}
        loc_req.use_localization_thresholds = bool(loc_cfg)
        loc_req.fitness_eval_threshold = float(
            loc_cfg.get('fitness_eval_threshold', 0.0))
        loc_req.threshold_fitness = float(loc_cfg.get('threshold_fitness', 0.0))
        loc_req.threshold_fitness_init = float(
            loc_cfg.get('threshold_fitness_init', 0.0))
        loc_resp, error = self._call_service(self.load_localization_cli, loc_req)
        if error or loc_resp is None or not loc_resp.success:
            message = error or loc_resp.message
            response.success = False
            response.message = f'localization map load failed: {message}'
            self._publish_map_status(MapStatus.FAILED, request.map_name, response.message)
            return response

        tomo_req = LoadTomogram.Request()
        tomo_req.map_name = request.map_name
        tomo_req.tomo_path = tomo_path
        tomo_resp, error = self._call_service(self.load_tomogram_cli, tomo_req)
        if error or tomo_resp is None or not tomo_resp.success:
            message = error or tomo_resp.message
            response.success = False
            response.message = f'tomogram load failed: {message}'
            self._publish_map_status(MapStatus.FAILED, request.map_name, response.message)
            return response

        self.current_map_name = request.map_name
        self._publish_map_status(MapStatus.LOADED, request.map_name, 'ok')
        response.success = True
        response.message = f'switched to map: {request.map_name}'
        return response

    # 重启服务：SOFT_RESET 走软重置；FULL_RESTART 则用 shlex 解析
    # full_restart_command 并后台拉起（节点本身不等待其结束）。
    def _restart_cb(self, request, response):
        if request.mode == RestartNavigation.Request.SOFT_RESET:
            self._soft_reset(reason='restart_navigation')
            response.accepted = True
            response.message = 'soft reset complete'
            return response

        if request.mode == RestartNavigation.Request.FULL_RESTART:
            if not self.full_restart_command:
                response.accepted = False
                response.message = 'full_restart_command is empty'
                return response
            try:
                subprocess.Popen(shlex.split(self.full_restart_command))
            except Exception as exc:
                response.accepted = False
                response.message = f'failed to start full restart command: {exc}'
                return response
            response.accepted = True
            response.message = 'full restart command accepted'
            return response

        response.accepted = False
        response.message = f'unknown restart mode: {request.mode}'
        return response

    # 软重置核心（幂等）：1) 发布零速 Twist 立即停车；2) 触发 coordinator
    # 清空路线；3) 发布空 waypoints 让 SCAN 丢弃当前跟踪；4) 触发 SCAN 重置。
    # 任一服务失败只告警不中断，保证后续步骤继续执行。
    def _soft_reset(self, reason: str):
        self.cmd_vel_pub.publish(Twist())

        req = Trigger.Request()
        resp, error = self._call_service(self.coordinator_reset_cli, req)
        if error:
            self.get_logger().warn(f'coordinator reset skipped: {error}')
        elif resp is not None and not resp.success:
            self.get_logger().warn(f'coordinator reset failed: {resp.message}')

        empty_path = Path()
        empty_path.header.stamp = self.get_clock().now().to_msg()
        empty_path.header.frame_id = 'map'
        self.waypoints_pub.publish(empty_path)

        req = Trigger.Request()
        resp, error = self._call_service(self.scan_reset_cli, req)
        if error:
            self.get_logger().warn(f'scan reset skipped: {error}')
        elif resp is not None and not resp.success:
            self.get_logger().warn(f'scan reset failed: {resp.message}')

    # 发布 /current_map（latched）状态消息，供 RViz/监控端显示地图状态。
    def _publish_map_status(self, state: int, map_name: str, reason: str):
        msg = MapStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.state = state
        msg.map_name = map_name
        msg.reason = reason
        self.map_status_pub.publish(msg)

    # 定位状态回调（定位丢失保护核心）：
    #   * 同步地图名：以 global_localization_node 上报的名称为准（定位 YAML
    #     是启动地图名的唯一数据源），避免维护两处默认值。
    #   * 上升沿检测 TRACKING_LOST：仅在状态从非丢失变为丢失时触发一次
    #     软重置（用 _last_localization_state 去抖），防止持续告警刷屏。
    def _localization_status_cb(self, msg: LocalizationStatus):
        # The localization YAML is the single source of truth for the startup
        # map name. Mirror the name reported by global_localization_node instead
        # of maintaining a separate launch-file default.
        reported_map_name = msg.map_name.strip()
        if reported_map_name and reported_map_name != self.current_map_name:
            self.current_map_name = reported_map_name
            self._publish_map_status(
                MapStatus.LOADED,
                reported_map_name,
                'localization_status',
            )

        if (
            msg.state == LocalizationStatus.TRACKING_LOST and
            self._last_localization_state != LocalizationStatus.TRACKING_LOST
        ):
            self.get_logger().warn('Localization lost; soft-stopping navigation')
            self._soft_reset(reason='localization_lost')
        self._last_localization_state = msg.state


# 入口：多线程执行器（4 线程）跑 NavManagerNode，服务与回调可并行。
def main(args=None):
    rclpy.init(args=args)
    node = NavManagerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
