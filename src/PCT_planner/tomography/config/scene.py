# =============================================================================
# scene.py — 仿真场景参数基类
# 每个场景由三部分参数组成：点云文件（ScenePCD）、建图参数（SceneMap）、
# 可通行性评估参数（SceneTrav）。
# =============================================================================
# 场景点云文件（位于 rsc/pcd/ 下）。
class ScenePCD():
    file_name = None


# 建图参数：分辨率、地面高度、切片高度间隔。
class SceneMap():
    resolution = 0.10
    ground_h = 0.0
    slice_dh = 0.5


# 可通行性评估参数：核尺寸、间距阈值、坡度/台阶限制、代价屏障，
# 以及安全边距与膨胀半径（影响离线建图与在线规划）。
class SceneTrav():
    kernel_size = 7
    interval_min = 0.50
    interval_free = 0.65
    slope_max = 0.36
    step_max = 0.20
    standable_ratio = 0.20
    cost_barrier = 50.0

    safe_margin = 0.4
    inflation = 0.2