# 导航 Service 接口文档

本文档描述 `pct_scan_navigation` 导航链路提供的业务 Service。ROS 2 每个节点自动生成的
`describe_parameters`、`get_parameters`、`set_parameters` 等参数 Service 不在本文档范围内。

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

## 接口总览

| Service | 类型 | 提供节点 | 启动条件 |
|---|---|---|---|
| `/open3d_loc/relocalize` | `open3d_loc/srv/Relocalize` | `localization_service_node` | `start_open3d_loc:=true` |
| `/open3d_loc/get_pose` | `open3d_loc/srv/GetPose` | `localization_service_node` | `start_open3d_loc:=true` |
| `/open3d_loc/publish_goal` | `open3d_loc/srv/PublishGoal` | `localization_service_node` | `start_open3d_loc:=true` |
| `/open3d_loc/pose_deviation` | `open3d_loc/srv/PoseDeviation` | `localization_service_node` | `start_open3d_loc:=true` |
| `/map_save` | `std_srvs/srv/Trigger` | `fastlio_mapping` | `start_open3d_loc:=true` |
| `/go2_cmd_vel_bridge/enable` | `std_srvs/srv/SetBool` | `go2_cmd_vel_bridge` | `start_go2_bridge:=true`；Go2/Go2-W 实机启动默认开启该节点 |

`pct_global_planner`、`pct_scan_coordinator`、`scan_planner_node` 和
`closed_loop_controller` 当前不提供业务 Service，主要通过 topic 通信。

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
