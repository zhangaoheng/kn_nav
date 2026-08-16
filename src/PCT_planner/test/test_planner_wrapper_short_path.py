# =============================================================================
# test_planner_wrapper_short_path.py — planner_wrapper 短路径单元测试
# 用伪造的本地库（stub lib 模块）与假规划器验证 TomogramPlanner.plan 在
# A* 路径只有一个/两个节点时的行为：短路径不应访问 GPMP 优化器。
# =============================================================================
import sys
import types
from pathlib import Path

import numpy as np


PCT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PCT_ROOT / 'planner' / 'scripts'))

native_lib = types.ModuleType('lib')
native_lib.a_star = types.SimpleNamespace(Astar=object)
native_lib.ele_planner = types.SimpleNamespace()
native_lib.traj_opt = types.SimpleNamespace(GPMPOptimizer=object)

previous_native_lib = sys.modules.get('lib')
sys.modules['lib'] = native_lib
try:
    import planner_wrapper
finally:
    if previous_native_lib is None:
        sys.modules.pop('lib')
    else:
        sys.modules['lib'] = previous_native_lib


# 假 A* 路径查找器：固定返回预设的路径矩阵。
class FakePathFinder:
    def __init__(self, path):
        self.path = path

    def get_result_matrix(self):
        return self.path


# 假 GPMP 优化器：返回固定的初始化值、层号与优化结果，供测试比对。
class FakeOptimizer:
    def get_opt_init_value(self):
        return np.zeros((4, 2))

    def get_opt_init_layer(self):
        return np.zeros(2)

    def get_result_matrix(self):
        return np.array([[1.0, 0.0, 2.0, 0.0],
                         [3.0, 0.0, 4.0, 0.0]])

    def get_layers(self):
        return np.zeros(2)

    def get_heights(self):
        return np.array([0.5, 0.75])


# 假底层规划器：记录是否访问过优化器，用于断言短路径不会触发 GPMP。
class FakePlanner:
    def __init__(self, plan_success, path, optimizer=None):
        self.plan_success = plan_success
        self.path_finder = FakePathFinder(path)
        self.optimizer = optimizer
        self.optimizer_requested = False

    def plan(self, start_idx, end_idx, optimize):
        return self.plan_success

    def get_path_finder(self):
        return self.path_finder

    def get_trajectory_optimizer(self):
        self.optimizer_requested = True
        if self.optimizer is None:
            raise AssertionError('short paths must not access GPMP results')
        return self.optimizer


# 不调用 __init__ 直接构造 TomogramPlanner 测试对象，注入假规划器与
# 位置/索引转换桩函数，隔离出待测的 plan 逻辑。
def make_planner(plan_success, path, optimizer=None):
    subject = planner_wrapper.TomogramPlanner.__new__(
        planner_wrapper.TomogramPlanner
    )
    subject.last_astar_traj = None
    subject.start_idx = np.zeros(3, dtype=np.int32)
    subject.end_idx = np.zeros(3, dtype=np.int32)
    subject.use_quintic = False
    subject.resolution = 1.0
    subject.map_dim = [10, 10]
    subject.center = np.zeros(2)
    subject.planner = FakePlanner(plan_success, path, optimizer)
    subject.pos2layer = lambda pos: 0
    subject.pos2idx = lambda pos: np.zeros(2, dtype=np.int32)
    subject.astar_path_to_map = lambda astar_path: np.asarray(astar_path)
    subject.sample_traj_heights = (
        lambda layers, cols, rows, fallback: np.asarray(fallback)
    )
    return subject


# A* 仅一个节点时：plan 应直接返回 [起点, 终点] 两点路径，且不访问优化器。
def test_single_astar_node_returns_exact_two_point_path():
    subject = make_planner(True, np.zeros((1, 6)))
    start = np.array([1.125, 2.25, 0.375])
    goal = np.array([1.5, 2.625, 0.5])

    result = subject.plan(start, goal)

    np.testing.assert_array_equal(result, np.stack([start, goal]))
    assert subject.last_astar_traj is not None
    assert not subject.planner.optimizer_requested


# 起终点相同（单节点）时：应返回仅含该点的一行路径。
def test_single_astar_node_at_same_position_returns_one_point_path():
    subject = make_planner(True, np.zeros((1, 6)))
    goal = np.array([1.125, 2.25, 0.375])

    result = subject.plan(goal.copy(), goal)

    assert result.shape == (1, 3)
    np.testing.assert_array_equal(result[0], goal)
    assert not subject.planner.optimizer_requested


# A* 路径多于一个节点时：仍应走优化器分支，返回优化结果。
def test_multi_node_astar_path_still_returns_optimizer_result(monkeypatch):
    optimizer = FakeOptimizer()
    subject = make_planner(True, np.zeros((2, 6)), optimizer)
    monkeypatch.setattr(
        planner_wrapper,
        'transTrajGrid2Map',
        lambda map_dim, center, resolution, traj: traj
    )

    result = subject.plan(np.zeros(3), np.ones(3))

    np.testing.assert_array_equal(
        result,
        np.array([[1.0, 2.0, 0.5], [3.0, 4.0, 0.75]])
    )
    assert subject.planner.optimizer_requested


# A* 规划失败时：plan 返回 None 且不访问优化器。
def test_failed_astar_returns_none():
    subject = make_planner(False, np.empty((0, 6)))

    assert subject.plan(np.zeros(3), np.ones(3)) is None
    assert subject.last_astar_traj is None
    assert not subject.planner.optimizer_requested
