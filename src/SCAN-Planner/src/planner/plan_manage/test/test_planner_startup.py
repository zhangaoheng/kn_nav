# ============================================================================
# 文件名：test_planner_startup.py
# 用途：SCAN-Planner 迁移后的冒烟测试（launch_test）：仅验证规划节点能够
#       用包内参数文件正常启动，不验证导航功能本身。
# 结构：
#   - generate_test_description：构造含 scan_planner_node 的启动描述
#   - TestPlannerStartup.test_process_starts：断言节点在 15s 内完成启动
# 依赖：launch_testing、pytest、scan_planner 包（config/scan_planner.yaml）
# ============================================================================
"""Smoke-test that the migrated planner accepts its ROS 2 parameter file."""

import os

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest


@pytest.mark.launch_test
    # 生成测试启动描述：以绝对路径参数文件启动 scan_planner_node，
    # 返回（LaunchDescription, 节点字典）供测试用例引用。
def generate_test_description():
    config = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "config", "scan_planner.yaml")
    )
    planner = launch_ros.actions.Node(
        package="scan_planner",
        executable="scan_planner_node",
        name="scan_planner_node",
        parameters=[config],
        output="screen",
    )
    return (
        launch.LaunchDescription([planner, launch_testing.actions.ReadyToTest()]),
        {"planner": planner},
    )


    # 冒烟测试类：验证规划进程能成功启动。
class TestPlannerStartup:
    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=15)
