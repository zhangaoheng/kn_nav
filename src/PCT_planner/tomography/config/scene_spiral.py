# =============================================================================
# scene_spiral.py — 螺旋坡道场景配置
# 指定点云文件、建图参数与可通行性参数，构成一个具体仿真场景。
# 该场景为螺旋坡道：坡度/台阶限制与安全边距取值更宽松。
# =============================================================================
from .scene import ScenePCD, SceneMap, SceneTrav


# 螺旋场景参数实例：基于基类字段赋值，覆盖为螺旋 PCD 调优的参数。
class SceneSpiral():
    pcd = ScenePCD()
    pcd.file_name = 'spiral0.3_2.pcd'

    map = SceneMap()
    map.resolution = 0.20
    map.ground_h = 0.0
    map.slice_dh = 0.5

    trav = SceneTrav()
    trav.kernel_size = 7
    trav.interval_min = 0.50
    trav.interval_free = 0.65
    trav.slope_max = 0.40
    trav.step_max = 0.30
    trav.standable_ratio = 0.40
    trav.cost_barrier = 50.0
    trav.safe_margin = 1.2
    trav.inflation = 0.2

