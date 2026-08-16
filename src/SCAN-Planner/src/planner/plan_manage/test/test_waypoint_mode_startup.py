# ============================================================================
# 文件名：test_waypoint_mode_startup.py
# 用途：验证“动态航点模式”（fsm.navi_mode=2）在无静态 fsm.waypoints 参数时
#       也能正常启动，且局部目标可视化话题 /scan_planner/local_target 会
#       以 Marker 类型对外发布（验证状态机本地目标发布链路）。
# 结构：
#   - generate_test_description：以 fsm.navi_mode=2 覆盖参数启动规划节点
#   - TestWaypointModeStartup：进程启动 + 话题类型断言两个用例
# 依赖：launch_testing、pytest、rclpy、scan_planner 包
# ============================================================================
"""Verify that dynamic waypoint mode starts without static fsm.waypoints."""

import os
import time

import launch
import launch_ros.actions
import launch_testing.actions
import pytest
import rclpy


@pytest.mark.launch_test
    # 生成测试启动描述：参数文件中强制 fsm.navi_mode=2（动态航点模式）。
def generate_test_description():
    config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "scan_planner.yaml")
    )
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[config, {"fsm.navi_mode": 2}],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


    # 航点模式启动测试类：进程启动检查 + 局部目标话题发布检查。
class TestWaypointModeStartup:
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)

        # 话题断言：轮询 ROS 话题列表，确认 /scan_planner/local_target 以
        # visualization_msgs/msg/Marker 类型发布（限时 10s）。
    def test_local_target_marker_topic(self):
        rclpy.init()
        node = rclpy.create_node("local_target_topic_test")
        try:
            deadline = time.monotonic() + 10.0
            topic_types = {}
            while time.monotonic() < deadline:
                topic_types = dict(node.get_topic_names_and_types())
                if "/scan_planner/local_target" in topic_types:
                    break
                rclpy.spin_once(node, timeout_sec=0.1)

            assert topic_types.get("/scan_planner/local_target") == [
                "visualization_msgs/msg/Marker"
            ]
        finally:
            node.destroy_node()
            rclpy.shutdown()
