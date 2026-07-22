# KN 导航工作区 — 架构与代码梳理

> 基于 Ubuntu 22.04 / ROS 2 Humble，面向 Unitree Go2 / Go2-W / G1，使用 Livox MID360。

---

## 1. 总体架构

```
MID360 雷达
  │ livox_ros_driver2
  ▼
FAST-LIO（激光惯性里程计 + 去畸变点云）
  │
  ├─ /Odometry_loc
  ▼
Open3D Localization（离线点云地图 ICP 匹配）
  │
  ├─ /Odometry_open3d  +  map→base_link TF
  ▼
┌────────────────────────────────────────────────┐
│           两条导航链路（二选一）                  │
│                                                │
│  链路 A（默认 / Mode 2）：                       │
│    PCT 全局规划 → /pct_path                     │
│      → pct_scan_coordinator（滚动 waypoint）     │
│      → SCAN-Planner（局部规划 + 避障）            │
│      → closed_loop_controller → /cmd_vel         │
│                                                │
│  链路 B（测试链路）：                             │
│    PCT 全局规划 → /pct_path                     │
│      → Pure Pursuit 直接跟踪 → /cmd_vel          │
│                                                │
│  链路 C（Mode 1）：                              │
│    RViz /goal_pose → SCAN-Planner 直接           │
└────────────────────────────────────────────────┘
  │
  ▼
Go2 CmdVel Bridge（安全限幅 + Unitree SportClient）
  │ Unitree SDK
  ▼
Go2 / Go2-W 机器人
```

---

## 2. 包清单（6 个功能包，13 编译目标）

### 2.1 `FAST_LIO_LOCALIZATION_HUMANOID`

**语言**：C++ / Python  
**路径**：`src/FAST_LIO_LOCALIZATION_HUMANOID/`  
**编译目标**：`fast_lio`、`open3d_loc`

| 子模块 | 职责 |
|---|---|
| `FAST_LIO/` | FAST-LIO2 激光惯性里程计。接收 Livox 点云 + IMU，输出 `/Odometry_loc` 和去畸变点云。含 `ikd-Tree` 动态 kd-tree、`IKFoM` 流形卡尔曼滤波。 |
| `open3d_loc/` | 基于离线点云地图的全局重定位。用 Open3D C++ SDK 做 ICP 匹配，提供 `/localization_3d`、`/localization_3d_confidence`、`/localization_3d_delay_ms` 和重定位服务 `/relocalize`。 |
| `data/map.ply` | 离线点云地图（Git 跟踪）。 |

**关键节点**：
- `fastlio_mapping` — 里程计
- `global_localization_node` — 全局定位
- `localization_service_node` — 重定位服务

**特点**：
- 支持 G1 倒装 MID360（roll=180° 外参修正）
- 定位延迟 < 30ms（150m×50m×30m 地图，i5-11400H 测试）
- 离线地图消除长时间累计漂移

---

### 2.2 `PCT_planner`

**语言**：Python（含 C++ pybind11 扩展）  
**路径**：`src/PCT_planner/`  
**编译目标**：`pct_planner`

| 子模块 | 职责 |
|---|---|
| `tomography/` | 离线：点云 → 3D 体素可通行性分析（CUDA）。评估坡度、台阶高度、净空、障碍物膨胀，输出 pickle。 |
| `planner/lib/` | C++ pybind 扩展：A\* 搜索 (`a_star`)、高程规划 (`ele_planner`)、GPMP 轨迹优化 (`traj_opt`)、地图管理 (`map_manager`)。 |
| `planner/lib/3rdparty/` | 本地第三方库：GTSAM 4.1.1、OSQP、pybind11。**不入 git**，需 `build_thirdparty.sh` 本地编译。 |
| `planner/scripts/` | Python 入口：`plan.py`、`plan_click_rviz2.py`、`planner_wrapper.py`。 |

**ROS 2 入口**：`scripts/run_ros2_global_planner.py`

**工作流程**：
1. **Tomography（离线）**：`tomography/scripts/run_standalone.py --scene Clinic` 从点云生成可通行性 tomogram
2. **A\* 搜索**：在 2.5D 高程图上查找路径，考虑障碍物膨胀、净空、坡度
3. **GPMP 优化**：5 次样条轨迹平滑，可配置航向角速率上限

**输出**：`/pct_path`（`nav_msgs/Path`，三维路径）

**关键参数**（详见 `PARAMETERS.md`）：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `trav.safe_margin` | 0.4 m | 障碍物硬膨胀距离 |
| `trav.slope_max` | 0.40 rad (23°) | 最大可爬坡度 |
| `trav.step_max` | 0.17 m | 最大可跨台阶高度 |
| `trav.interval_min` | 0.50 m | 最小净空 |
| `map.resolution` | 0.10 m | 栅格分辨率 |
| `clearance_cost_weight` | 8.0 | 路径居中偏好强度 |

---

### 2.3 `SCAN-Planner`

**语言**：C++  
**路径**：`src/SCAN-Planner/`  
**编译目标**：`plan_env`、`path_searching`、`bspline_opt`、`traj_utils`、`scan_planner_msgs`、`scan_planner`

| 子模块 | 职责 |
|---|---|
| `plan_env/` | 体素占栅格环境建模 (`ESDFMap`) |
| `path_searching/` | 动态 A\* 路径搜索 |
| `bspline_opt/` | B-spline 轨迹平滑 |
| `traj_utils/` | 轨迹工具（多项式、可视化） |
| `scan_planner_msgs/` | 自定义 ROS 消息 |
| `plan_manage/` | 主节点 + FSM 状态机 + 配置文件 |

**ROS 2 入口节点**：
- `scan_planner_node` — 规划节点
- `closed_loop_controller` — 闭环速度控制器

**输入**：`/Odometry_open3d`（里程计）、`/scan_map`（map 系点云）  
**输出**：`/cmd_vel`

**模式**：
- Mode 1：直接接收 RViz `/goal_pose`，自主规划到目标
- Mode 2：接收 coordinator 的 `/scan_planner/waypoints`，沿 waypoint 序列逐点前进

---

### 2.4 `pure_pursuit_planner`

**语言**：C++  
**路径**：`src/pure_pursuit_planner/`  
**编译目标**：`pure_pursuit_planner`

| 源文件 | 职责 |
|---|---|
| `pure_pursuit_planner_component.cpp` | Pure Pursuit 核心算法：look-ahead 前视距离、曲率→速度映射、终点朝向对准 |
| `go2_cmd_vel_bridge.cpp` | Go2 安全桥：Unitree SportClient 封装 + 三层超时安全 + 速度硬限幅 |
| `go2_safety_controller.cpp` | 安全控制器逻辑单元 |

**接口**：

| 方向 | 话题 | 类型 |
|---|---|---|
| 输入 | 里程计 | `nav_msgs/Odometry` |
| 输入 | 目标路径 | `nav_msgs/Path` |
| 输入 | 运动状态 | `rt/sportmodestate`（Unitree DDS） |
| 输出 | 速度命令 | `geometry_msgs/Twist` → `/cmd_vel` |
| 服务 | 使能/禁用 | `/go2_cmd_vel_bridge/enable`（`std_srvs/SetBool`） |

**安全保护**（Go2 Bridge）：

| 保护项 | 行为 |
|---|---|
| 速度硬限幅 | vx ∈ [0, 0.25] m/s，vyaw ∈ [-0.5, 0.5] rad/s，禁止后退/横向 |
| cmd_vel 超时 | 停止运动 + 锁 disable |
| 里程计超时 | 停止运动 + 锁 disable |
| SportState 心跳超时 | 停止运动 + 锁 disable |
| 数值异常 | 停止运动 + 锁 disable |
| SDK 错误 | `StopMove()` + 锁 disable |

**注意**：桥**默认禁用**，需手动 `ros2 service call /go2_cmd_vel_bridge/enable ...` 使能。

---

### 2.5 `pct_scan_navigation`

**语言**：C++ / Python  
**路径**：`src/pct_scan_navigation/`  
**编译目标**：`pct_scan_navigation`

这是**顶层协调 + 启动编排**包。提供：

| 文件 | 职责 |
|---|---|
| `src/pct_scan_coordinator.cpp` | 路径采样 + 滚动 waypoint 协调器 |
| `src/waypoint_utils.cpp` | waypoint 工具函数 |
| `launch/local_pct_scan_navigation.launch.py` | 本机全套 bringup |
| `launch/unitree_go2_pct_scan_navigation.launch.py` | Go2 实机 bringup |
| `launch/unitree_go2w_pct_scan_navigation.launch.py` | Go2-W 实机 bringup |
| `launch/global_path_follow_test.launch.py` | PCT + Pure Pursuit 测试链路 |
| `config/local/`、`unitree_go2/`、`unitree_go2w/` | 按机器人分 profile 的配置文件 |

**Coordinator (Mode 2) 行为**：

1. 订阅 `/pct_path`（PCT 全局路径）
2. 每隔 `waypoint_spacing`（默认 1.0m）采样 waypoint
3. 根据机器人累计移动距离，滚动发布剩余 waypoint 到 `/scan_planner/waypoints`
4. 到达终点 XY 容差（`goal_tolerance`，默认 0.15m）后停止滚动
5. 中间点仅用于塑形 SCAN 参考轨迹，最后一点始终是唯一终点

**启动命令**：

```bash
# 本机调试（默认 Mode 2）
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py

# Go2 实机
ros2 launch pct_scan_navigation unitree_go2_pct_scan_navigation.launch.py network_interface:=eth0

# Go2-W 实机
ros2 launch pct_scan_navigation unitree_go2w_pct_scan_navigation.launch.py network_interface:=eth0

# Mode 1（跳过 PCT，SCAN 直达 goal_pose）
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py navigation_mode:=1

# PCT + Pure Pursuit 直接跟踪测试
ros2 launch pct_scan_navigation global_path_follow_test.launch.py network_interface:=eth0
```

---

### 2.6 `tools/`

**路径**：`src/tools/`

| 文件 | 用途 |
|---|---|
| `goal_points_cli.py` | 批量目标点命令行工具 |
| `goal_points.json` | 预设目标点列表 |
| `monitor_scan_planner_cpu.py` | SCAN-Planner CPU 监控 |
| `open_go2_nav_tmux.sh` | tmux 多窗口快速启动脚本 |
| `fastlio.rviz` | FAST-LIO RViz 配置 |

---

## 3. 坐标系与话题合约

| 话题 | 类型 | 产生方 | 消费方 |
|---|---|---|---|
| `/Odometry_loc` | `nav_msgs/Odometry` | FAST-LIO | Open3D 定位 |
| `/Odometry_open3d` | `nav_msgs/Odometry` | Open3D 定位 | 所有下游 |
| `/scan_map` | `sensor_msgs/PointCloud2` | Open3D 定位 | SCAN-Planner |
| `/pct_path` | `nav_msgs/Path` | PCT 规划器 | coordinator / Pure Pursuit |
| `/scan_planner/waypoints` | `nav_msgs/Path` | coordinator | SCAN-Planner |
| `/goal_pose` | `geometry_msgs/PoseStamped` | RViz | SCAN-Planner (Mode 1) |
| `/cmd_vel` | `geometry_msgs/Twist` | SCAN / Pure Pursuit | Go2 Bridge |
| `/initial_path` | `nav_msgs/Path` | SCAN-Planner | (可视化) |
| `/localization_3d` | `geometry_msgs/PoseStamped` | Open3D | 可视化/监控 |
| `/localization_3d_confidence` | `std_msgs/Float32` | Open3D | 监控 |
| `/localization_3d_delay_ms` | `std_msgs/Float32` | Open3D | 监控 |
| `tf` (map→base_link) | `tf2_msgs/TFMessage` | Open3D | 所有 |

**坐标系约定**：
- 全局帧：`map`
- 定位、路径、点云均在 `map` 系
- 定位地图和 PCT 地面地图需保持原点/朝向/尺度一致

---

## 4. 配置分层

`pct_scan_navigation/config/` 按机器人分三个配置 profile：

```
config/
├── local/           # 本机调试
│   ├── coordinator.yaml
│   ├── fast_lio.yaml
│   ├── go2_bridge.yaml
│   ├── open3d_loc.yaml
│   ├── pct_global_planner.yaml
│   ├── pure_pursuit.yaml
│   └── scan_planner.yaml
├── unitree_go2/     # Go2 实机
│   └── (同上)
└── unitree_go2w/    # Go2-W 实机
    └── (同上)
```

每个 profile 内独立配置：雷达话题、网卡接口、坐标变换、速度限制等。

---

## 5. 编译说明

### 5.1 前置依赖

```bash
# 系统依赖
sudo apt install -y python3-colcon-common-extensions python3-pip python3-rosdep \
  libeigen3-dev libpcl-dev libopencv-dev libboost-filesystem-dev libboost-system-dev

# Python 依赖
pip install open3d numpy scipy

# Open3D C++ SDK（需额外安装，非 apt）
# Unitree SDK2（安装到 /opt/unitree_robotics）
```

### 5.2 首次编译

```bash
# PCT 第三方库本地编译（GTSAM、OSQP）
cd src/PCT_planner/planner
bash build_thirdparty.sh
bash build.sh

# 工作区编译
cd ~/xq/code/work_space/kn_nav_ws
source /opt/ros/humble/setup.bash
export CMAKE_PREFIX_PATH=/opt/unitree_robotics:$CMAKE_PREFIX_PATH
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH
export Open3D_DIR=/path/to/open3d/lib/cmake/Open3D
colcon build --symlink-install
source install/setup.bash
```

### 5.3 已知编译问题

| 问题 | 原因 | 解决 |
|---|---|---|
| GTSAM 找不到 | PCT 本地编译物不入 git | `build_thirdparty.sh` 或从旧工作区恢复 `install/` 目录 |
| OSQP 找不到 | 同上 | 同上 |
| Open3D 找不到 | CMakeLists.txt 路径硬编码 | 设置 `Open3D_DIR` 环境变量或修改 `open3d_loc/CMakeLists.txt` |
| FastDEM catkin 报错 | ROS 1 包混入 ROS 2 工作区 | 已通过 `COLCON_IGNORE` 解决 |

---

## 6. 运行流程

### 6.1 标准运行（Mode 2，PCT + SCAN）

```bash
# 终端 1：启动全部节点
ros2 launch pct_scan_navigation local_pct_scan_navigation.launch.py

# 终端 2：使能底盘（仅实机需要）
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool '{data: true}'

# 终端 3：在 RViz 中用 "2D Goal Pose" 或 "Publish Point" 工具点击目标
# 也可以直接发布 goal_pose：
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: "map"}, pose: {...}}'
```

### 6.2 常用调试命令

```bash
# 查看 PCT 路径
ros2 topic echo /pct_path

# 查看导航状态
ros2 topic echo /pct_scan_navigation/status

# 取消当前任务
ros2 service call /pct_scan_navigation/cancel std_srvs/srv/Trigger '{}'

# 禁用底盘
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool '{data: false}'
```

---

## 7. 关键参数速查

### Coordinator (`pct_scan_navigation`)

| 参数 | 默认 | 说明 |
|---|---|---|
| `waypoint_spacing` | 1.0 m | waypoint 采样间距 |
| `waypoint_z_offset` | 0.0 m | waypoint Z 偏移 |
| `goal_tolerance` | 0.15 m | 终点 XY 容差 |

### PCT Planner

| 参数 | 默认 | 说明 |
|---|---|---|
| `trav.safe_margin` | 0.4 m | 障碍物碰撞半径 |
| `trav.slope_max` | 0.40 rad (23°) | 最大可爬坡度 |
| `trav.step_max` | 0.17 m | 最大台阶高度 |
| `a_star_cost_threshold` | 20.0 | A* 可通行性截断 |
| `use_quintic` | True | 5 次样条平滑 |

### Pure Pursuit

| 参数 | 默认 | 说明 |
|---|---|---|
| `target_vel` | 0.4 m/s | 目标线速度 |
| `Lfc` | 前视距离 | look-ahead 距离 |
| `goal_threshold` | - | 目标位置容差 |

### Go2 Bridge

| 参数 | 默认 | 说明 |
|---|---|---|
| `max_vx` | 0.25 m/s | 前进速度上限 |
| `max_vyaw` | 0.5 rad/s | 偏航角速度上限 |
| `sdk_timeout` | 0.5 s | SDK 调用超时 |
| `control_rate` | 20 Hz | 控制频率 |

---

## 8. 已知限制

1. **PCT 第三方库不入 git**：GTSAM / OSQP 编译产物需每台机器本地生成
2. **Open3D C++ 路径敏感**：`CMakeLists.txt` 中 `Open3D_DIR` 需按环境配置
3. **定位地图 vs PCT 地面地图**：两个独立文件，需保证原点/朝向/尺度完全一致
4. **Z 轴未接入控制**：Pure Pursuit 只跟踪 XY；PCT Z 仅用于楼层选择
5. **局部避障未完全接入**：SCAN-Planner 的 `obstacle_th` 等参数尚未启用
6. **多入口启动**：定位、PCT、跟踪控制分属不同 launch，非一键 bringup
7. **Mode 3 未实现**：`pct_scan_coordinator` 直接拒绝 mode=3
8. **Go2 Bridge 默认禁用**：每次启动后需手动 `enable`
