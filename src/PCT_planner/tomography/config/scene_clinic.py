# =============================================================================
# scene_clinic.py — 诊所场景配置
# 指定点云文件、建图参数与可通行性参数，构成一个具体仿真场景。
# 注意 ground_h=-13.1：该场景点云原点位于地下，地面高度需相应下移。
# =============================================================================
from .scene import ScenePCD, SceneMap, SceneTrav


# 诊所场景参数实例：基于基类字段赋值，覆盖为诊所 PCD 调优的参数。
class SceneClinic():
    pcd = ScenePCD()
    pcd.file_name = 'clinic.pcd'

    map = SceneMap()
    map.resolution = 0.10
    map.ground_h = -13.1
    map.slice_dh = 0.5

    trav = SceneTrav()
    trav.kernel_size = 7
    trav.interval_min = 0.50
    trav.interval_free = 0.65
    trav.slope_max = 0.40
    trav.step_max = 0.17
    trav.standable_ratio = 0.20
    trav.cost_barrier = 50.0
    trav.safe_margin = 0.4
    trav.inflation = 0.2
