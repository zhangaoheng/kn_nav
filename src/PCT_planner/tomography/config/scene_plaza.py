# =============================================================================
# scene_plaza.py — 广场场景配置
# 指定点云文件、建图参数与可通行性参数，构成一个具体仿真场景。
# =============================================================================
from .scene import ScenePCD, SceneMap, SceneTrav


# 广场场景参数实例：基于基类字段赋值，覆盖为广场 PCD 调优的参数。
class ScenePlaza():
    pcd = ScenePCD()
    pcd.file_name = 'plaza3_10.pcd'

    map = SceneMap()
    map.resolution = 0.10
    map.ground_h = 0.0
    map.slice_dh = 0.5

    trav = SceneTrav()
    trav.kernel_size = 7
    trav.interval_min = 0.50
    trav.interval_free = 0.65
    trav.slope_max = 0.36
    trav.step_max = 0.17
    trav.standable_ratio = 0.2
    trav.cost_barrier = 50.0
    trav.safe_margin = 0.4
    trav.inflation = 0.2

