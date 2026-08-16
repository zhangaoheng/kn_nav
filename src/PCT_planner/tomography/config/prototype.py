# =============================================================================
# prototype.py — 点云/网格原型构造
# POINT_FIELDS_XYZI：PointCloud2 的 xyz+intensity 字段定义；
# GRID_POINTS_XYZI：生成以地图中心为原点的网格索引与点坐标原型。
# =============================================================================
import numpy as np
from sensor_msgs.msg import PointField


# xyz + intensity 四字段 PointCloud2 字段原型（FLOAT32）。
POINT_FIELDS_XYZI = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


# 生成 dim_x*dim_y 网格：返回网格索引原型与以中心为原点的点坐标原型，
# z 置 0、intensity 置 1，供逐层可视化取点复用。
def GRID_POINTS_XYZI(resolution, dim_x, dim_y):
    index_proto = np.zeros((dim_x * dim_y, 2), dtype=int)
    lx = np.linspace(0, dim_x - 1, dim_x, dtype=int)
    ly = np.linspace(0, dim_y - 1, dim_y, dtype=int)
    ix, iy = np.meshgrid(lx, ly)
    index_proto[:, 0] = ix.flatten()
    index_proto[:, 1] = iy.flatten()

    point_proto = np.zeros((dim_x * dim_y, 4), dtype=np.float32)
    point_proto[:, :2] = index_proto[:, :2].astype(np.float32, copy=True)
    point_proto[:, 0] -= 0.5 * dim_x
    point_proto[:, 1] -= 0.5 * dim_y
    point_proto[:, :2] *= resolution
    point_proto[:, 3] = 1.0

    return index_proto, point_proto