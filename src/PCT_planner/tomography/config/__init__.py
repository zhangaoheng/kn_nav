# =============================================================================
# config 包：tomography（离线建图）的配置与场景定义集合。
# 导出点云字段原型、通用配置 Config 及各仿真场景（螺旋/楼宇/广场/诊所）。
# =============================================================================
from .prototype import POINT_FIELDS_XYZI, GRID_POINTS_XYZI

from .param import Config

from .scene_spiral import SceneSpiral
from .scene_building import SceneBuilding
from .scene_plaza import ScenePlaza
from .scene_clinic import SceneClinic