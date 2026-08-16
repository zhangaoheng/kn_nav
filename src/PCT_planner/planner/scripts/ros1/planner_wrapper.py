# 文件：ros1/planner_wrapper.py（ROS 1 版规划器封装）
# 用途：与 planner/scripts/planner_wrapper.py 同源的历史版本，供 ROS 1 脚本使用；
#       加载离线 tomogram 并封装底层 C++ 规划器（ele_planner/a_star/traj_opt）。
# 结构：TomogramPlanner —— 核心封装类（loadTomogram/initPlanner/plan）。
# 数据流：config.Config → TomogramPlanner → lib 底层 C++ → (N,3) map 系轨迹。
import os
import sys
import pickle
import numpy as np
from scipy.ndimage import distance_transform_edt, maximum_filter

from utils import *

sys.path.append('../')
from lib import a_star, ele_planner, traj_opt

rsg_root = os.path.dirname(os.path.abspath(__file__)) + '/../..'


# TomogramPlanner：PCT 规划器封装类。职责：管理 tomogram 元数据与坐标换算，
# 预处理代价地图（clearance/gateway），并把世界坐标起终点转为网格索引交给底层规划。
class TomogramPlanner(object):
    # 读取配置并初始化状态；tomo_dir 指向 tomogram 存放目录，地图元数据初始为空。
    def __init__(self, cfg):
        self.cfg = cfg

        self.use_quintic = self.cfg.planner.use_quintic
        self.max_heading_rate = self.cfg.planner.max_heading_rate
        self.a_star_cost_threshold = self.cfg.planner.a_star_cost_threshold
        self.step_cost_weight = self.cfg.planner.step_cost_weight
        self.optimizer_cost_threshold = self.cfg.planner.optimizer_cost_threshold
        self.use_clearance_cost = self.cfg.planner.use_clearance_cost
        self.clearance_cost_mode = getattr(
            self.cfg.planner, 'clearance_cost_mode', 'absolute'
        )
        self.clearance_cost_weight = self.cfg.planner.clearance_cost_weight
        self.clearance_cost_decay = self.cfg.planner.clearance_cost_decay
        self.clearance_cost_local_radius = getattr(
            self.cfg.planner, 'clearance_cost_local_radius', 1.0
        )
        self.clearance_cost_cap = getattr(
            self.cfg.planner, 'clearance_cost_cap',
            self.clearance_cost_weight
        )

        self.tomo_dir = rsg_root + self.cfg.wrapper.tomo_dir

        self.resolution = None
        self.center = None
        self.n_slice = None
        self.slice_h0 = None
        self.slice_dh = None
        self.map_dim = []
        self.offset = None
        self.elev_g = None
        self.raw_trav = None
        self.planning_trav = None
        self.clearance = None
        self.last_astar_traj = None

        self.start_idx = np.zeros(3, dtype=np.int32)
        self.end_idx = np.zeros(3, dtype=np.int32)

    # 从 pickle 加载 tomogram（data 为 5 通道：通行代价、梯度 x/y、高程、顶面高程），
    # 填充元数据（分辨率/中心/切片数等）后调用 initPlanner 构建底层规划器。
    def loadTomogram(self, tomo_file):
        with open(self.tomo_dir + tomo_file + '.pickle', 'rb') as handle:
            data_dict = pickle.load(handle)

            tomogram = np.asarray(data_dict['data'], dtype=np.float32)

            self.resolution = float(data_dict['resolution'])
            self.center = np.asarray(data_dict['center'], dtype=np.double)
            self.n_slice = tomogram.shape[1]
            self.slice_h0 = float(data_dict['slice_h0'])
            self.slice_dh = float(data_dict['slice_dh'])
            self.map_dim = [tomogram.shape[2], tomogram.shape[3]]
            self.offset = np.array([int(self.map_dim[0] / 2), int(self.map_dim[1] / 2)], dtype=np.int32)

        trav = tomogram[0]
        trav_gx = tomogram[1]
        trav_gy = tomogram[2]
        self.raw_trav = trav.copy()
        elev_g_raw = tomogram[3]
        self.elev_g = elev_g_raw.copy()
        elev_g = np.nan_to_num(elev_g_raw, nan=-100)
        elev_c = tomogram[4]
        elev_c = np.nan_to_num(elev_c, nan=1e6)

        self.initPlanner(trav, trav_gx, trav_gy, elev_g, elev_c)
        
    # 计算 gateway（层间可穿越门：上下层代价骤变且高程接近时标记），叠加 clearance
    # 代价，将各层展平后传给 C++ 的 OfflineElePlanner.init_map。
    def initPlanner(self, trav, trav_gx, trav_gy, elev_g, elev_c):
        diff_t = trav[1:] - trav[:-1]
        diff_g = np.abs(elev_g[1:] - elev_g[:-1])

        gateway_up = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t < -8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[1:]))
        gateway_up[:-1] = np.logical_and(mask_t, mask_g)

        gateway_dn = np.zeros_like(trav, dtype=bool)
        mask_t = diff_t > 8.0
        mask_g = (diff_g < 0.1) & (~np.isnan(elev_g[:-1]))
        gateway_dn[1:] = np.logical_and(mask_t, mask_g)
        
        gateway = np.zeros_like(trav, dtype=np.int32)
        gateway[gateway_up] = 2
        gateway[gateway_dn] = -2

        planning_trav, planning_gx, planning_gy = self.add_clearance_cost(
            trav, trav_gx, trav_gy
        )
        self.planning_trav = planning_trav

        self.planner = ele_planner.OfflineElePlanner(
            max_heading_rate=self.max_heading_rate, use_quintic=self.use_quintic
        )
        self.planner.init_map(
            self.a_star_cost_threshold,
            self.optimizer_cost_threshold,
            self.resolution,
            self.n_slice,
            self.step_cost_weight,
            trav.reshape(-1, trav.shape[-1]).astype(np.double),
            planning_trav.reshape(-1, planning_trav.shape[-1]).astype(np.double),
            elev_g.reshape(-1, elev_g.shape[-1]).astype(np.double),
            elev_c.reshape(-1, elev_c.shape[-1]).astype(np.double),
            gateway.reshape(-1, gateway.shape[-1]),
            planning_gy.reshape(-1, planning_gy.shape[-1]).astype(np.double),
            -planning_gx.reshape(-1, planning_gx.shape[-1]).astype(np.double)
        )

    # 对可通行区域叠加“离障碍距离”的平滑代价：absolute 模式用指数衰减，relative
    # 模式用局部窗口内相对间距；代价梯度同步叠加到 trav_gx/trav_gy 上。
    def add_clearance_cost(self, trav, trav_gx, trav_gy):
        """Add a smooth distance-to-obstacle preference to traversable cells."""
        planning_trav = trav.copy()
        planning_gx = trav_gx.copy()
        planning_gy = trav_gy.copy()
        self.clearance = np.zeros_like(trav, dtype=np.float32)

        if not self.use_clearance_cost or self.clearance_cost_weight <= 0.0:
            return planning_trav, planning_gx, planning_gy

        traversable = np.isfinite(trav) & (trav <= self.a_star_cost_threshold)
        clearance_free = np.isfinite(trav) & (trav <= self.optimizer_cost_threshold)
        for layer in range(trav.shape[0]):
            self.clearance[layer] = distance_transform_edt(
                clearance_free[layer]
            ).astype(np.float32) * self.resolution

        mode = str(self.clearance_cost_mode).lower()
        if mode == 'absolute':
            if self.clearance_cost_decay <= 0.0:
                raise ValueError('clearance_cost_decay must be greater than zero')
            clearance_cost = self.clearance_cost_weight * np.exp(
                -self.clearance / self.clearance_cost_decay
            )
        elif mode == 'relative':
            local_radius_cells = max(
                1,
                int(np.ceil(self.clearance_cost_local_radius / self.resolution))
            )
            window = 2 * local_radius_cells + 1
            local_width = maximum_filter(
                self.clearance,
                size=(1, window, window),
                mode='nearest'
            )
            relative_clearance = self.clearance / (local_width + 1e-6)
            relative_clearance = np.clip(relative_clearance, 0.0, 1.0)
            clearance_cost = self.clearance_cost_weight * (
                1.0 - relative_clearance
            ) ** 2
            clearance_cost = np.clip(
                clearance_cost,
                0.0,
                self.clearance_cost_cap
            )
        else:
            raise ValueError(
                "clearance_cost_mode must be 'relative' or 'absolute'"
            )

        clearance_cost[~traversable] = 0.0
        planning_trav += clearance_cost

        # Tomogram gradients use unnormalised central differences, so calculate
        # the clearance contribution in the same convention.
        clearance_gx = np.zeros_like(clearance_cost)
        clearance_gy = np.zeros_like(clearance_cost)
        clearance_gx[:, 1:-1, :] = (
            clearance_cost[:, 2:, :] - clearance_cost[:, :-2, :]
        )
        clearance_gy[:, :, 1:-1] = (
            clearance_cost[:, :, 2:] - clearance_cost[:, :, :-2]
        )
        planning_gx += clearance_gx
        planning_gy += clearance_gy

        return planning_trav, planning_gx, planning_gy

    # 主规划接口：世界坐标 → 网格索引 → A* 搜索 → GPMP 优化，输出 (N,3) map 系轨迹；
    # A* 失败或路径为空返回 None。
    def plan(self, start_pos, end_pos):
        self.last_astar_traj = None
        self.start_idx[0] = self.pos2layer(start_pos)
        self.end_idx[0] = self.pos2layer(end_pos)
        self.start_idx[1:] = self.pos2idx(start_pos[:2])
        self.end_idx[1:] = self.pos2idx(end_pos[:2])

        print("Start idx:", self.start_idx, "End idx:", self.end_idx)

        self.planner.plan(self.start_idx, self.end_idx, True)
        path_finder: a_star.Astar = self.planner.get_path_finder()
        path = path_finder.get_result_matrix()
        if len(path) == 0:
            return None

        self.last_astar_traj = self.astar_path_to_map(path)

        optimizer: traj_opt.GPMPOptimizer = (
            self.planner.get_trajectory_optimizer()
            if not self.use_quintic
            else self.planner.get_trajectory_optimizer_wnoj()
        )

        opt_init = optimizer.get_opt_init_value()
        init_layer = optimizer.get_opt_init_layer()
        traj_raw = optimizer.get_result_matrix()
        layers = optimizer.get_layers()
        heights = optimizer.get_heights()

        opt_init = np.concatenate([opt_init.transpose(1, 0), init_layer.reshape(-1, 1)], axis=-1)
        traj = np.concatenate([traj_raw, layers.reshape(-1, 1)], axis=-1)
        y_idx = (traj.shape[-1] - 1) // 2
        heights = self.sample_traj_heights(
            layers, traj[:, 0], traj[:, y_idx], heights
        )
        traj_3d = np.stack([traj[:, 0], traj[:, y_idx], heights / self.resolution], axis=1)
        traj_3d = transTrajGrid2Map(self.map_dim, self.center, self.resolution, traj_3d)

        return traj_3d

    # 返回最近一次规划的 A* 原始路径（调试用）。
    def getLastAstarPath(self):
        return self.last_astar_traj

    # 沿优化后的 XY 轨迹逐点采样 tomogram 高程，避免输出 z 方向平坦。
    def sample_traj_heights(self, layers, cols, rows, fallback_heights):
        """Sample tomogram elevation along optimized XY to avoid flat z output."""
        sampled = np.asarray(fallback_heights, dtype=np.float64).copy()
        if self.elev_g is None:
            return sampled

        layers = np.rint(layers).astype(np.int32)
        rows = np.rint(rows).astype(np.int32)
        cols = np.rint(cols).astype(np.int32)
        for i in range(sampled.shape[0]):
            h = self.nearest_elevation(layers[i], rows[i], cols[i])
            if h is not None:
                sampled[i] = h
        return sampled

    # 在指定层 (row,col) 邻域内搜索最近的有限高程值，找不到返回 None。
    def nearest_elevation(self, layer, row, col, search_radius=2):
        layer = int(np.clip(layer, 0, self.n_slice - 1))
        row = int(np.clip(row, 0, self.map_dim[0] - 1))
        col = int(np.clip(col, 0, self.map_dim[1] - 1))

        h = self.elev_g[layer, row, col]
        if np.isfinite(h):
            return float(h)

        x0 = max(0, row - search_radius)
        x1 = min(self.map_dim[0], row + search_radius + 1)
        y0 = max(0, col - search_radius)
        y1 = min(self.map_dim[1], col + search_radius + 1)
        local_elev = self.elev_g[layer, x0:x1, y0:y1]
        finite = np.isfinite(local_elev)
        if not np.any(finite):
            return None

        local_rows, local_cols = np.where(finite)
        dist_sq = (
            (local_rows + x0 - row) ** 2 +
            (local_cols + y0 - col) ** 2
        )
        nearest = int(np.argmin(dist_sq))
        return float(local_elev[local_rows[nearest], local_cols[nearest]])

    # 将 A* 网格路径 [layer, row, col] 转为 map 系 XYZ 坐标（含高程）。
    def astar_path_to_map(self, path):
        """Convert the raw [layer, row, col] A* path to map-frame XYZ."""
        path_idx = np.rint(path).astype(np.int32)
        layers = path_idx[:, 0]
        rows = path_idx[:, 1]
        cols = path_idx[:, 2]
        heights = self.elev_g[layers, rows, cols]

        traj_grid = np.stack(
            [cols, rows, heights / self.resolution], axis=1
        ).astype(np.float64)
        return transTrajGrid2Map(
            self.map_dim, self.center, self.resolution, traj_grid
        )
    
    # 世界 XY 坐标 → 网格 [row, col]（与 pos2array_idx 的索引顺序互换）。
    def pos2idx(self, pos):
        idx = self.pos2array_idx(pos)
        idx = np.array([idx[1], idx[0]], dtype=np.int32)
        return idx

    # 世界 XY 坐标 → 网格 [x_idx, y_idx]（先减 center，除以分辨率，再加 offset）。
    def pos2array_idx(self, pos):
        pos = np.asarray(pos, dtype=np.float64) - self.center
        idx = np.round(pos / self.resolution).astype(np.int32) + self.offset
        return idx

    # 世界点 → 切片层号：优先在 XY 邻域内按高程匹配；越界或无法匹配时回退到 z 换算。
    def pos2layer(self, pos):
        pos = np.asarray(pos, dtype=np.float64)
        if pos.shape[0] < 3 or not np.isfinite(pos[2]) or self.elev_g is None:
            return 0

        idx = self.pos2array_idx(pos[:2])
        if (
            idx[0] < 0 or idx[0] >= self.map_dim[0] or
            idx[1] < 0 or idx[1] >= self.map_dim[1]
        ):
            fallback = self.z2slice_layer(pos[2])
            print("Clicked point outside tomogram grid, fallback layer:", fallback)
            return fallback

        layer = self.nearest_layer_at_idx(idx, pos[2])
        print(
            "Selected layer %d for clicked z %.3f at grid [%d, %d]" %
            (layer, pos[2], idx[0], idx[1])
        )
        return layer

    # 在指定网格位置的所有切片中，按高程差最小选出最匹配的层。
    def nearest_layer_at_idx(self, idx, z):
        search_radius = 2
        x0 = max(0, idx[0] - search_radius)
        x1 = min(self.map_dim[0], idx[0] + search_radius + 1)
        y0 = max(0, idx[1] - search_radius)
        y1 = min(self.map_dim[1], idx[1] + search_radius + 1)

        local_elev = self.elev_g[:, x0:x1, y0:y1]
        finite = np.isfinite(local_elev)
        if not np.any(finite):
            return self.z2slice_layer(z)

        scores = np.abs(local_elev - z)
        scores[~finite] = np.inf
        layer = int(np.unravel_index(np.argmin(scores), scores.shape)[0])
        return layer

    # 由高度 z 直接换算切片层号（z - slice_h0）/ slice_dh，并裁剪到合法范围。
    def z2slice_layer(self, z):
        layer = int(np.round((z - self.slice_h0) / self.slice_dh))
        return int(np.clip(layer, 0, self.n_slice - 1))
