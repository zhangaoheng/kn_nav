# =============================================================================
# convertion.py — 坐标转换工具
# 把规划器内部使用的网格坐标系轨迹转换为地图（map）坐标系轨迹。
# =============================================================================
import numpy as np


# 网格坐标 -> 地图坐标：先按网格中心做偏移（offset），再乘分辨率得到物理
# 尺寸，最后平移到地图中心并交换 x/y 轴（网格按行主序存储，地图以 x-y 为序）。
def transTrajGrid2Map(grid_dim, center, resolution, traj_grid):
    offset = np.array([grid_dim[1] // 2, grid_dim[0] // 2, 0])
    center_ = np.array([center[1], center[0], 0.5])

    traj_grid = (traj_grid - offset) * resolution + center_

    traj_map = np.stack([traj_grid[:, 1], traj_grid[:, 0], traj_grid[:, 2]], axis=1)

    return traj_map
