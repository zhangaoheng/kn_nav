# 导航 Service 与状态 Topic 接口文档

本文档描述 `pct_scan_navigation` 导航链路提供的业务 Service，以及本导航管理功能新增的状态
Topic。ROS 2 每个节点自动生成的 `describe_parameters`、`get_parameters`、`set_parameters`
等参数 Service 不在本文档范围内。

## 使用准备

```bash
cd work_space/kn_nav_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

查看当前实际存在的 Service：

```bash
ros2 service list -t
```

所有位姿接口均使用 `map` 坐标系。四元数顺序为 `qx/qy/qz/qw`；输入四元数会先校验，
再归一化。包含非有限数值或模长过小的四元数会被拒绝。

## Service 总览

| Service | 类型 | 提供节点 | 启动条件 |
|---|---|---|---|
| `/open3d_loc/relocalize` | `open3d_loc/srv/Relocalize` | `localization_service_node` | `start_open3d_loc:=true` |
| `/open3d_loc/get_pose` | `open3d_loc/srv/GetPose` | `localization_service_node` | `start_open3d_loc:=true` |
| `/open3d_loc/publish_goal` | `open3d_loc/srv/PublishGoal` | `localization_service_node` | `start_open3d_loc:=true` |
| `/open3d_loc/pose_deviation` | `open3d_loc/srv/PoseDeviation` | `localization_service_node` | `start_open3d_loc:=true` |
| `/switch_map` | `pct_scan_navigation/srv/SwitchMap` | `nav_manager_node` | 启动导航管理节点 |
| `/restart_navigation` | `pct_scan_navigation/srv/RestartNavigation` | `nav_manager_node` | 启动导航管理节点 |
| `/global_localization_node/load_map` | `pct_scan_navigation/srv/LoadLocalizationMap` | `global_localization_node` | 启动 Open3D 定位节点 |
| `/pct_global_planner/load_tomogram` | `pct_scan_navigation/srv/LoadTomogram` | `pct_global_planner` | 启动 PCT 全局规划节点 |
| `/scan_planner_node/reset_navigation` | `std_srvs/srv/Trigger` | `scan_planner_node` | 启动 SCAN 局部规划节点 |
| `/map_save` | `std_srvs/srv/Trigger` | `fastlio_mapping` | `start_open3d_loc:=true` |
| `/go2_cmd_vel_bridge/enable` | `std_srvs/srv/SetBool` | `go2_cmd_vel_bridge` | `start_go2_bridge:=true`；Go2/Go2-W 实机启动默认开启该节点 |

`/switch_map` 和 `/restart_navigation` 是推荐给上层系统调用的管理接口。
`/global_localization_node/load_map`、`/pct_global_planner/load_tomogram`、
`/scan_planner_node/reset_navigation` 是导航管理节点内部编排使用的接口，一般不建议绕过
`nav_manager_node` 直接调用，除非在单节点调试。

## 状态 Topic 总览

| Topic | 类型 | 发布节点 | 订阅/使用节点 | 说明 |
|---|---|---|---|---|
| `/localization_status` | `pct_scan_navigation/msg/LocalizationStatus` | `global_localization_node` | `nav_manager_node`、上层监控 | Open3D 定位状态，周期发布 |
| `/navigation_status` | `pct_scan_navigation/msg/NavigationStatus` | `scan_planner_node` | 上层监控 | 导航执行状态；唯一可信发布源是 `scan_planner_node` |
| `/current_map` | `pct_scan_navigation/msg/MapStatus` | `nav_manager_node` | `scan_planner_node`、上层监控 | 当前地图状态；QoS 为 transient local |

### 新增的关键话题监听关系

- `nav_manager_node` 监听 `/localization_status`：当定位进入 `TRACKING_LOST` 时触发 soft reset，
  停止当前导航输出。
- `scan_planner_node` 监听 `/current_map`：地图加载中发布 `MAP_SWITCHING` 导航状态，地图加载完成后
  更新 `/navigation_status.map_name`。
- `nav_manager_node` 在 soft reset 和切图前会发布：
  - `/cmd_vel`：一帧零速度；
  - `/scan_planner/waypoints`：空 `nav_msgs/Path`，用于清空动态 waypoint。

## `/switch_map`

按地图名切换 Open3D 定位 PCD 和 PCT tomogram pickle。该接口由 `nav_manager_node` 提供，
是上层系统切图时推荐调用的入口。

### 类型

```text
pct_scan_navigation/srv/SwitchMap
```

### 请求

```text
string map_name
```

`map_name` 必须存在于 `map_profiles.yaml` 的 `maps` 字段下，并且对应 profile 至少包含：

```yaml
pcd_path: "/path/to/map.pcd"
tomo_path: "/path/to/tomogram.pickle"
```

### 响应

```text
bool success
string message
```

### 行为

- 发布 `/current_map` 为 `LOADING`。
- 先执行 soft reset：发布零 `/cmd_vel`、发布空 `/scan_planner/waypoints`，并尝试调用
  `/scan_planner_node/reset_navigation`。
- 调用 `/global_localization_node/load_map` 加载新 PCD。
- 调用 `/pct_global_planner/load_tomogram` 加载新 pickle。
- 两者都成功后发布 `/current_map` 为 `LOADED`。
- 任意一步失败时返回 `success=false`，并发布 `/current_map` 为 `FAILED`。
- 切图成功后定位会回到 `UNINITIALIZED`，不会沿用旧定位结果，也不会自动恢复旧目标点。

### 调用示例

```bash
ros2 service call /switch_map pct_scan_navigation/srv/SwitchMap "{map_name: outdoor}"
```

## `/restart_navigation`

导航运行管理重启接口。当前 v1 支持 ROS 内部软复位，并预留外部完整重启入口。

### 类型

```text
pct_scan_navigation/srv/RestartNavigation
```

### 请求

```text
uint8 SOFT_RESET=0
uint8 FULL_RESTART=1
uint8 mode
```

### 响应

```text
bool accepted
string message
```

### 行为

- `SOFT_RESET=0`：
  - 发布零 `/cmd_vel`；
  - 发布空 `/scan_planner/waypoints`；
  - 尝试调用 `/scan_planner_node/reset_navigation`；
  - reset service 不可用或失败时只打印 warn，不阻塞主流程。
- `FULL_RESTART=1`：
  - 执行 `nav_manager_node` 参数 `full_restart_command` 指定的外部命令；
  - 如果该参数为空，返回 `accepted=false`。

### 调用示例

```bash
ros2 service call /restart_navigation pct_scan_navigation/srv/RestartNavigation "{mode: 0}"
```

## `/global_localization_node/load_map`

Open3D 定位节点的内部地图加载接口。通常由 `/switch_map` 间接调用。

### 类型

```text
pct_scan_navigation/srv/LoadLocalizationMap
```

### 请求

```text
string map_name
string pcd_path
bool use_localization_thresholds
float32 fitness_eval_threshold
float32 threshold_fitness
float32 threshold_fitness_init
```

### 响应

```text
bool success
string message
```

### 行为

- 发布 `/localization_status` 为 `MAP_SWITCHING`。
- 从 `pcd_path` 读取 PCD，并按节点参数重新构建定位用下采样地图。
- 清空 scan queue，重置 `map→odom` 为单位矩阵。
- 设置 `loc_initialized=false`，定位状态回到 `UNINITIALIZED`。
- `use_localization_thresholds=true` 时，只应用请求中大于 0 的定位阈值。
- 加载失败时恢复旧阈值，并返回 `success=false`。

### 调试调用示例

```bash
ros2 service call /global_localization_node/load_map pct_scan_navigation/srv/LoadLocalizationMap \
  "{map_name: outdoor, pcd_path: '/path/to/outdoor.pcd', use_localization_thresholds: false, fitness_eval_threshold: 0.0, threshold_fitness: 0.0, threshold_fitness_init: 0.0}"
```

## `/pct_global_planner/load_tomogram`

PCT 全局规划器的内部 tomogram reload 接口。通常由 `/switch_map` 间接调用。

### 类型

```text
pct_scan_navigation/srv/LoadTomogram
```

### 请求

```text
string map_name
string tomo_path
```

### 响应

```text
bool success
string message
```

### 行为

- 只先检查 `tomo_path` 非空且文件存在。
- 用临时 `TomogramPlanner` 加载新 pickle。
- 临时 planner 加载成功后才替换当前 planner；加载失败不会污染旧 planner。
- 成功后立即发布一次新的 `/tomogram`，并发布空 `/pct_path` 清除旧全局路径。
- 原有 `/tomogram` 1Hz 周期发布逻辑保持不变。

### 调试调用示例

```bash
ros2 service call /pct_global_planner/load_tomogram pct_scan_navigation/srv/LoadTomogram \
  "{map_name: outdoor, tomo_path: '/path/to/outdoor_global_voxel002_ground.pickle'}"
```

## `/scan_planner_node/reset_navigation`

SCAN 局部规划器内部 soft reset 接口。通常由 `/restart_navigation` 或 `/switch_map` 间接调用。

### 类型

```text
std_srvs/srv/Trigger
```

### 请求/响应

```text
---
bool success
string message
```

### 行为

- 清空 active waypoints 和 pending path。
- 清除当前目标、new target 标志、replan fail count、end yaw 等运行状态。
- 如果已有 odom，则调用 emergency stop。
- FSM 回到 `WAIT_TARGET`。
- 由 `scan_planner_node` 发布新的 `/navigation_status`。

### 调试调用示例

```bash
ros2 service call /scan_planner_node/reset_navigation std_srvs/srv/Trigger "{}"
```

## `/localization_status`

Open3D 定位状态 topic，由 `global_localization_node` 周期发布。

### 类型

```text
pct_scan_navigation/msg/LocalizationStatus
```

### 字段

```text
std_msgs/Header header

uint8 UNINITIALIZED=0
uint8 INITIALIZING=1
uint8 INIT_SUCCESS=2
uint8 TRACKING=3
uint8 TRACKING_WARN=4
uint8 TRACKING_LOST=5
uint8 MAP_SWITCHING=6
uint8 state

string map_name
float32 fitness
string reason
```

`nav_manager_node` 会监听该 topic；第一次发现 `TRACKING_LOST` 时触发 soft reset。

### 监听示例

```bash
ros2 topic echo /localization_status
```

## `/navigation_status`

导航执行状态 topic。v1 中唯一可信发布源是 `scan_planner_node`，`nav_manager_node` 不再发布或转发
该 topic。

### 类型

```text
pct_scan_navigation/msg/NavigationStatus
```

### 字段

```text
std_msgs/Header header

uint8 IDLE=0
uint8 WAITING_GOAL=1
uint8 PLANNING_GLOBAL=2
uint8 GLOBAL_READY=3
uint8 PLANNING_LOCAL=4
uint8 NAVIGATING=5
uint8 AVOIDING=6
uint8 BLOCKED=7
uint8 GOAL_REACHED=8
uint8 CANCELED=9
uint8 FAILED=10
uint8 LOCALIZATION_LOST=11
uint8 MAP_SWITCHING=12
uint8 state

string map_name
bool goal_active
float32 distance_to_goal
uint32 remaining_waypoints
string reason
```

### 状态来源

- `WAIT_TARGET`：无目标时发布 `WAITING_GOAL`，有目标待处理时发布 `PLANNING_LOCAL`。
- `GEN_NEW_TRAJ`：发布 `PLANNING_LOCAL`。
- `REPLAN_TRAJ`：发布 `AVOIDING`。
- `EXEC_TRAJ` / `FINAL_YAW_ALIGN`：发布 `NAVIGATING`。
- `EMERGENCY_STOP`：发布 `BLOCKED`。
- 目标抵达、取消、地图切换、地图失败会分别发布 `GOAL_REACHED`、`CANCELED`、
  `MAP_SWITCHING`、`FAILED`。

### 监听示例

```bash
ros2 topic echo /navigation_status
ros2 topic info /navigation_status
```

正常情况下 `ros2 topic info /navigation_status` 应只看到一个 publisher，即 `scan_planner_node`。

## `/current_map`

当前地图状态 topic，由 `nav_manager_node` 发布。QoS 使用 reliable + transient local，因此 RViz 或后启动
节点也能拿到最近一次地图状态。

### 类型

```text
pct_scan_navigation/msg/MapStatus
```

### 字段

```text
std_msgs/Header header

uint8 UNLOADED=0
uint8 LOADING=1
uint8 LOADED=2
uint8 FAILED=3
uint8 state

string map_name
string reason
```

### 监听关系

- `scan_planner_node` 监听 `/current_map`，用于更新 `/navigation_status.map_name`。
- 当 `/current_map.state=LOADING` 时，`scan_planner_node` 发布 `NavigationStatus.MAP_SWITCHING`。
- 当 `/current_map.state=FAILED` 时，`scan_planner_node` 发布 `NavigationStatus.FAILED`。

### 监听示例

```bash
ros2 topic echo /current_map
```

## `/open3d_loc/relocalize`

使用给定的 `base_link` 在 `map` 下的初始位姿触发 Open3D 重定位。

### 类型

```text
open3d_loc/srv/Relocalize
```

### 请求

```text
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
```

### 响应

```text
bool success
string message
```

### 行为

- 校验位置和四元数，并归一化四元数。
- 向 `/initialpose` 发布 `geometry_msgs/PoseWithCovarianceStamped`。
- 清除服务节点缓存的旧定位结果，等待请求之后的新 `/Odometry_open3d`。
- 新 `/Odometry_open3d` 到达时返回 `success=true`；默认 10 秒内没有结果则返回
  `success=false`。等待上限由节点参数 `relocalize_timeout_sec` 配置，不属于请求字段。
- 同一时间只允许一个重定位请求；并发请求返回失败。

### 调用示例

```bash
ros2 service call /open3d_loc/relocalize open3d_loc/srv/Relocalize \
  "{x: 1.0, y: 2.0, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

## `/open3d_loc/get_pose`

获取最新的 `base_link` 在 `map` 下的位姿。数据来自 `/Odometry_open3d`。

### 类型

```text
open3d_loc/srv/GetPose
```

### 请求

请求为空。

### 响应

```text
bool success
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
string message
```

尚未收到有效 `/Odometry_open3d` 或正在通过 Service 重定位时，返回
`success=false`，不会返回重定位前缓存的旧位姿。

### 调用示例

```bash
ros2 service call /open3d_loc/get_pose open3d_loc/srv/GetPose "{}"
```

## `/open3d_loc/publish_goal`

把给定导航点封装为 `geometry_msgs/PoseStamped` 并发布到 `/goal_pose`。

### 类型

```text
open3d_loc/srv/PublishGoal
```

### 请求

```text
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
```

### 响应

```text
bool success
string message
```

发布消息的 `header.frame_id` 固定为 `map`，时间戳使用节点当前 ROS 时间。
`success=true` 只表示导航点已经交给 ROS publisher，不表示规划成功、机器人到达或导航完成。

Mode 1 中 `/goal_pose` 直接由 SCAN-Planner 接收；Mode 2 中由 PCT 全局规划器接收并生成
`/pct_path`。

### 调用示例

```bash
ros2 service call /open3d_loc/publish_goal open3d_loc/srv/PublishGoal \
  "{x: 3.0, y: 1.0, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```

## `/open3d_loc/pose_deviation`

计算当前定位相对给定参考位姿的平面位置和航向误差，不会触发重定位。

### 类型

```text
open3d_loc/srv/PoseDeviation
```

### 请求

```text
float64 x
float64 y
float64 z
float64 qx
float64 qy
float64 qz
float64 qw
```

请求表示参考 `base_link` 位姿，坐标系为 `map`。

### 响应

```text
bool success
geometry_msgs/Pose current_pose
float64 error_x
float64 error_y
float64 distance_xy
float64 yaw_error_rad
float64 yaw_error_deg
string message
```

- `current_pose`：当前 `/Odometry_open3d` 位姿。
- `error_x/error_y`：在参考位姿自身航向坐标系下的平面误差。
- `distance_xy`：平面欧氏距离。
- `yaw_error_rad/yaw_error_deg`：当前航向减参考航向，并归一化到 `[-pi, pi]`。
- 没有有效当前定位时返回 `success=false`。

### 调用示例

```bash
ros2 service call /open3d_loc/pose_deviation open3d_loc/srv/PoseDeviation \
  "{x: 1.0, y: 2.0, z: 0.4, qx: 0.0, qy: 0.0, qz: 0.0, qw: 1.0}"
```



## `/go2_cmd_vel_bridge/enable`

启用或禁用 Unitree Go2/Go2-W 速度命令桥。桥节点启动后默认处于禁用状态。

### 类型与字段

```text
std_srvs/srv/SetBool
bool data
---
bool success
string message
```

### 启用

```bash
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool "{data: true}"
```

启用前必须同时满足：

- 已收到且未超时的 `/Odometry_open3d`；
- 已收到且未超时的 Unitree SportModeState；
- `SportClient::StopMove` 调用成功。

启用成功后桥仍会等待一条新的 `/cmd_vel`，不会立即沿用启用前的旧速度命令。

### 禁用

```bash
ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool "{data: false}"
```

禁用会清空目标速度并请求 `StopMove`。当前使能状态也会发布在
`/go2_cmd_vel_bridge/armed`。

## 常见失败说明

| `message` 示例 | 含义 |
|---|---|
| `quaternion norm is too small` | 输入四元数无效 |
| `position contains non-finite value` | XYZ 包含 NaN 或无穷值 |
| `another relocalize request is running` | 已有重定位调用正在等待结果 |
| `timeout waiting for relocalization result on /Odometry_open3d` | 等待上限内没有新的有效定位结果 |
| `no current base_link pose from /Odometry_open3d` | 尚无可返回的定位结果 |
| `cannot enable: odometry heartbeat is missing or stale` | 底盘桥缺少新鲜定位心跳 |
| `cannot enable: sport state heartbeat is missing or stale` | 底盘桥缺少新鲜 Unitree 状态 |
| `Map save disabled.` | Fast-LIO 配置未启用 PCD 保存 |
| `unknown map_name: ...` | `/switch_map` 请求的地图名不在 `map_profiles.yaml` 中 |
| `map profile ... requires pcd_path and tomo_path` | 地图 profile 缺少必填 PCD 或 pickle 路径 |
| `localization map load failed: ...` | Open3D PCD 地图加载失败 |
| `tomogram load failed: ...` | PCT tomogram pickle 加载失败 |
| `tomo_path does not exist: ...` | `/pct_global_planner/load_tomogram` 请求的 pickle 文件不存在 |
| `full_restart_command is empty` | 请求 FULL_RESTART，但未配置外部重启命令 |
