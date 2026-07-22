# pure_pursuit_planner — 代码梳理

> 基于 ROS 2 Humble 的 Pure Pursuit 路径跟踪器 + Go2 安全速度桥。

---

## 1. 总体架构

```
/pct_path ─────────┐
(nav_msgs/Path)    │
                   ▼
/Odometry_open3d ─► PurePursuitNode ──► /cmd_vel ──► Go2CmdVelBridge ──► Unitree SportClient::Move
                   │  10Hz timer          (Twist)       │                 (Go2 机器人)
                   │                                    │
/final_approach ◄──┘                                    ├─ /Odometry_open3d（心跳监控）
(std_msgs/Bool)                                         ├─ rt/sportmodestate（DDS 心跳监控）
                                                        ├─ /go2_cmd_vel_bridge/enable（服务）
                                                        ├─ /go2_cmd_vel_bridge/armed（状态发布）
                                                        └─ /go2_cmd_vel_bridge/safe_cmd_vel（限幅后命令发布）
```

PurePursuitNode 和 Go2CmdVelBridge 是**两个独立 ROS 节点**，通过 `/cmd_vel` 话题耦合：

| 节点 | 进程 | 职责 |
|---|---|---|
| `pure_pursuit_planner` | `pure_pursuit_planner.cpp` | 路径跟踪算法，输出速度命令 |
| `go2_cmd_vel_bridge` | `go2_cmd_vel_bridge.cpp` | 安全限幅 + Unitree SDK 转发 |

---

## 2. 编译目标

```
pure_pursuit_planner_library (静态库)
├── go2_safety_controller.cpp
├── pure_pursuit_planner_node.cpp
└── pure_pursuit_planner_component.cpp

pure_pursuit_planner (可执行文件)
└── pure_pursuit_planner.cpp (main → spin PurePursuitNode)
    链接: pure_pursuit_planner_library

go2_cmd_vel_bridge (可执行文件)
└── go2_cmd_vel_bridge.cpp (main → spin Go2CmdVelBridge)
    链接: pure_pursuit_planner_library + unitree_sdk2
```

---

## 3. 核心组件详解

### 3.1 PurePursuitComponent（纯算法库，无 ROS 依赖）

**文件**：`include/pure_pursuit_planner/pure_pursuit_planner_component.hpp` + `src/pure_pursuit_planner_component.cpp`

纯 C++ 的 Pure Pursuit 算法实现，**不依赖 ROS**，可独立单元测试。

#### 3.1.1 数据结构

```cpp
struct Pose2D { double x, y, yaw; };

struct PurePursuitConfig {
    double k = 0.5;                  // 前视距离比例系数
    double Lfc = 0.8;                // 前视距离基础偏移
    double Kp = 1.0;                 // (未使用)
    double dt = 0.1;                 // (未使用)
    double goal_threshold = 0.4;     // 终点位置容差 (m)
    double final_heading_entry_distance = 0.20;  // 进入终点朝向对准的距离
    double final_heading_command_deadband = 0.02; // 终点朝向死区 (rad)
    double min_final_angular_velocity = 0.20;     // 终点最小旋转角速度
    double max_acceleration = 0.08;   // (仅结构体中定义，未在 component 中使用)
    double minCurvature = 0.0;       // 最小曲率
    double maxCurvature = 3.0;       // 最大曲率
    double minVelocity = 0.4;        // 最小线速度
    double maxVelocity = 0.7;        // 最大线速度
    double maxAngularVelocity = 1.3; // 最大角速度
    double rotate_to_path_threshold = 1.047;  // 原地旋转触发角 (≈60°)
    double rotate_to_path_tolerance = 0.349;   // 旋转对准容差 (≈20°)
    double goal_yaw_tolerance = 0.175;         // 终点朝向容差 (≈10°)
    double rotate_to_heading_gain = 1.0;       // 旋转 P 增益
    bool standalone_goal_completion = false;   // 独立模式：到达 goal_threshold 即停
    double obstacle_th = 0.5;        // 障碍物阈值（未接入）
    double odom_timeout = 0.3;       // (仅结构体定义，实际超时在 node 层处理)
};
```

#### 3.1.2 核心算法流程

`computeVelocity()` 是主入口，每次定时器回调调用一次：

```
computeVelocity(cx, cy, cyaw, ck, pose, velocity, final_approach)
  │
  ├─ 输入校验：路径为空 → 返回 (0, 0)
  │
  ├─ [分支1] final_approach && 距终点 < final_heading_entry_distance
  │    └─ 终点朝向对准模式
  │       ├─ 朝向误差 ≤ deadband → 返回 (0, 0)
  │       └─ 否则 → 返回 (0, calculateFinalRotationAngularVelocity)
  │
  ├─ [分支2] standalone_goal_completion && 距终点 < goal_threshold
  │    └─ 直接返回 (0, 0)
  │
  ├─ searchTargetIndex() → 找前视目标点 (ind, Lf)
  │    │
  │    ├─ Lf = k * current_velocity + Lfc   （前视距离 = 比例×当前速度 + 基础值）
  │    ├─ 找最近路径点: oldNearestPointIndex
  │    │   ├─ 首次: calcFirstNearestPointIndex() → 全路径扫描最小距离
  │    │   └─ 之后: calcOldNearestPointIndex() → 从上一最近点后向搜索
  │    └─ 从最近点沿路径向前推进，直到 Lf 距离
  │
  ├─ 计算 alpha = atan2(dy, dx) - current_yaw  （目标点相对机器人朝向角）
  │
  ├─ [分支3] |alpha| > rotate_to_path_threshold (≈60°)
  │    └─ 触发原地旋转: rotating_to_path_ = true
  │       └─ 返回 (0, calculateRotationAngularVelocity)
  │
  ├─ [分支4] rotating_to_path_ == true
  │    ├─ |alpha| ≤ rotate_to_path_tolerance → 旋转完成，继续
  │    └─ 否则 → 继续原地旋转
  │
  ├─ 曲率→速度映射: curvatureToVelocity(curvature)
  │    └─ v = (vmax-vmin) * sin³(acos(cuberoot(curv_norm))) + vmin
  │       本质是：曲率越大→速度越低，用三次根号+三角函数做平滑映射
  │
  ├─ 角速度: w = v * sin(alpha) / Lf    （Pure Pursuit 经典公式）
  │
  └─ 限幅后返回 {v, w}
```

#### 3.1.3 关键方法

| 方法 | 职责 |
|---|---|
| `calcFirstNearestPointIndex()` | 首次定位：暴力扫描全路径找最近点 |
| `calcOldNearestPointIndex()` | 增量定位：从 `oldNearestPointIndex - 20` 开始向前搜索，找距离下降的拐点 |
| `searchTargetIndex()` | 从最近点沿路径向前推进到 Lf 前视距离处的目标点 |
| `calcLf(k, v, Lfc)` | `k*v + Lfc`，速度自适应前视距离 |
| `curvatureToVelocity(curvature)` | 归一化曲率→速度平滑映射 |
| `calculateAngularVelocity(v, alpha, Lf)` | `v * sin(alpha) / Lf`，经典 Pure Pursuit 角速度 |
| `calculateRotationAngularVelocity(yaw_error)` | P 控制器：`gain * yaw_error`，限幅 |
| `calculateFinalRotationAngularVelocity(yaw_error)` | 终点旋转：带最小角速度下限，避免"爬行"到位 |
| `alphaExceptionHandling(tempAlpha)` | 角度归一化 + 在 ±π 附近加微小偏移避免 singular |
| `isGoalReached(v, w)` | 当前仅原样返回（功能未实现） |

#### 3.1.4 原地旋转逻辑

```
            ┌─────────────────────────────────┐
            │  正常跟踪                         │
            │  |alpha| < rotate_to_path_threshold │
            └──────┬──────────────────────────┘
                   │ |alpha| ≥ 60°
                   ▼
            ┌──────────────────┐
            │ rotating_to_path  │  原地旋转，v=0，w=P*yaw_error
            │    = true         │
            └──────┬───────────┘
                   │ |alpha| ≤ 20°
                   ▼
            ┌──────────────────┐
            │ 恢复正常跟踪      │
            └──────────────────┘
```

### 3.2 PurePursuitNode（ROS 节点层）

**文件**：`include/pure_pursuit_planner/pure_pursuit_planner_node.hpp` + `src/pure_pursuit_planner_node.cpp`

在 PurePursuitComponent 上封装 ROS 2 接口：

#### 3.2.1 话题接口

| 方向 | 话题 | 类型 | QoS |
|---|---|---|---|
| 订阅 | `/pct_path` | `nav_msgs/Path` | 10 |
| 订阅 | `/Odometry_open3d` | `nav_msgs/Odometry` | 10 |
| 订阅 | `/pct_scan_navigation/final_approach` | `std_msgs/Bool` | reliable transient_local |
| 发布 | `/cmd_vel` | `geometry_msgs/Twist` | 10 |

#### 3.2.2 PCT 路径适配（关键改造）

PCT 发布的路径中：
- `position.z` 存的是**楼层高度**，不是曲率
- `orientation` 发布的是恒等四元数

PurePursuitNode 在 `pathCallback()` 中做两层转换：

```cpp
// 1. 从相邻点计算切线方向 cyaw
cyaw_ = compute_yaw_from_path(raw_x, raw_y);   // atan2(Δy, Δx)

// 2. 保留路径最后一个点的显式朝向（覆盖切线估计）
//    如果 PCT 路径最后点的 orientation 有效，用其 yaw 覆盖 cyaw_.back()

// 3. 从路径几何计算离散曲率 ck
ck_ = compute_curvature_from_path(raw_x, raw_y);  // κ = Δθ/Δs
```

#### 3.2.3 定时器回调（100ms / 10Hz）

```cpp
timerCallback() {
    if (!pose_received_) return;

    // 里程计超时检测
    if (odom_age > odom_timeout) → publishZeroVelocity();

    // 无路径 → 发布零速（保持 bridge heartbeat）
    if (!path_received_) → publishZeroVelocity();

    // 调用算法
    cmd = planner_.computeVelocity(cx_, cy_, cyaw_, ck_,
                                   current_pose_, current_vx_, final_approach_);

    // 异常检测 → 零速兜底
    if (cmd invalid) → publishZeroVelocity();

    // 发布
    cmd_vel_pub_->publish(cmd);
}
```

#### 3.2.4 空路径 / 异常路径处理

```cpp
pathCallback(msg):
    if (poses.empty()) {
        清空所有内部状态
        planner_.odom_sub_flag = false   // 触发下次 compute 时重新初始化最近点搜索
        planner_.oldNearestPointIndex = -1
        publishZeroVelocity()
        return
    }
    if (点包含非有限值) {
        同样清空 + 零速
        return
    }
    // 正常：提取、计算 cyaw/ck、重建状态
```

#### 3.2.5 参数校验

`declareAndGetParameters()` 中做了**全面参数合法性检查**，启动时拒绝非法配置：

| 检查项 | 约束 |
|---|---|
| `goal_threshold` | > 0 |
| `final_heading_entry_distance` | > 0 |
| `final_heading_command_deadband` | ∈ [0, π] |
| `min_final_angular_velocity` | ≥ 0 |
| `Lfc` | > 0 |
| 速度上下限 | `0 ≤ minVelocity ≤ maxVelocity` |
| `maxCurvature` | > 0 |
| `maxAngularVelocity` | > 0 |
| 旋转阈值 | `0 < tolerance < threshold ≤ π` |
| `goal_yaw_tolerance` | ∈ (0, π] |
| `rotate_to_heading_gain` | > 0 |
| `odom_timeout` | > 0 |
| 话题名 | 非空 |

---

### 3.3 Go2SafetyController（安全状态机）

**文件**：`include/pure_pursuit_planner/go2_safety_controller.hpp` + `src/go2_safety_controller.cpp`

#### 3.3.1 状态模型

```
    ┌──────────┐
    │ DISABLED │ ◄────────────────────────────┐
    │ armed=0  │                              │
    └────┬─────┘                              │
         │ enable() + 心跳正常 + StopMove OK   │
         ▼                                    │
    ┌──────────┐        故障/超时/disable()    │
    │  ARMED   │ ─────────────────────────────┘
    │ waiting  │
    │ _for_    │
    │ _command │
    └────┬─────┘
         │ acceptCommand(non-zero)
         ▼
    ┌──────────┐
    │ COMMAND  │ ◄── tick() 执行速度斜坡 + Move()
    │ ACTIVE   │
    └──────────┘
```

#### 3.3.2 配置

```cpp
struct Go2SafetyConfig {
    double min_vx = 0.0;                    // 前进最小速度
    double max_vx = 0.25;                   // 前进最大速度
    double max_abs_vy = 0.0;                // 横向速度上限 (=0: 禁止横向)
    double max_abs_vyaw = 0.5;              // 偏航角速度上限
    double max_linear_acceleration = 0.25;  // 线加速度上限
    double max_yaw_acceleration = 0.5;      // 角加速度上限
    duration command_timeout = 0.3s;        // cmd_vel 超时
    duration odometry_timeout = 0.3s;       // 里程计心跳超时
    duration sport_state_timeout = 0.5s;    // 运动状态心跳超时
};
```

#### 3.3.3 三层安全保护

| 保护层 | 触发条件 | 行为 |
|---|---|---|
| 心跳监控 | 里程计 / SportState 超时 | `faultAndDisarm()` → `StopMove()` + `armed_=false` |
| 命令超时 | 超过 `command_timeout` 未收到新 cmd_vel | 同上 |
| 数值异常 | cmd_vel 含 NaN/Inf | 同上 |
| SDK 错误 | `SportClient::Move` 返回值 ≠ 0 | 同上 |

#### 3.3.4 速度斜坡（Slew Rate Limiting）

`tick()` 中的加速度限制：

```cpp
next.vx   = approach(last_output_.vx,   target_command_.vx,   linear_delta);
next.vyaw = approach(last_output_.vyaw, target_command_.vyaw, yaw_delta);

// approach(current, target, max_delta):
//   return current + clamp(target - current, -max_delta, max_delta);
```

作用：即使上游发布阶跃式速度命令，下发到机器人的速度也是平滑渐变的，避免急加速。

#### 3.3.5 faultAndDisarm 的原子性

每次故障都会：
1. `sendStop()` —— 先发 Move(0,0,0) 再发 StopMove()
2. `armed_ = false` —— 锁 disable
3. 清空 `target_command_` / `last_output_`
4. 记录 `last_fault_`

**恢复**必须通过 enable 服务手动重新使能（且要求心跳恢复）。

---

### 3.4 Go2CmdVelBridge（ROS 2 节点层）

**文件**：`src/go2_cmd_vel_bridge.cpp`

#### 3.4.1 接口总览

| 方向 | 名称 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/cmd_vel` | `Twist` | 来自 PurePursuitNode |
| 订阅 | `/Odometry_open3d` | `Odometry` | 里程计心跳 |
| 订阅 | `rt/sportmodestate` | Unitree DDS | 机器人运动状态心跳 |
| 服务 | `/go2_cmd_vel_bridge/enable` | `SetBool` | 手动使能/禁用 |
| 发布 | `/go2_cmd_vel_bridge/armed` | `Bool` | 当前 armed 状态 (latched) |
| 发布 | `/go2_cmd_vel_bridge/safe_cmd_vel` | `Twist` | 限幅后的安全命令 |

#### 3.4.2 关键流程

```
commandCallback(cmd_vel):
    ├─ Go2VelocityCommand{linear.x, linear.y, angular.z}
    ├─ safety_controller_->acceptCommand(...)
    │   ├─ 未 armed → 忽略
    │   ├─ 非有限值 → faultAndDisarm
    │   ├─ clampCommand() → 硬限幅
    │   └─ 零速 → 立刻 StopMove
    ├─ 发布 /armed 状态 (如果变化)
    └─ 发布 /safe_cmd_vel

controlTick() — 20Hz:
    ├─ safety_controller_->tick(now)
    │   ├─ 检查三层心跳
    │   ├─ 速度斜坡 + Move()
    │   └─ Move 失败 → faultAndDisarm
    ├─ 发布 /safe_cmd_vel + /armed
    └─ 状态变化时打日志

enableCallback(req):
    ├─ req.data == true  → safety_controller_->enable()
    │   └─ 要求里程计 + SportState 心跳均有效
    └─ req.data == false → safety_controller_->disable()
```

#### 3.4.3 Unitree SDK 集成

```cpp
// 初始化 ChannelFactory（DDS）
unitree::robot::ChannelFactory::Instance()->Init(dds_domain_id, network_interface);

// SportClient 封装
class UnitreeSportCommandClient : public SportCommandInterface {
    unitree::robot::go2::SportClient client_;
    int move(vx, vy, vyaw) → client_.Move(vx, vy, vyaw);
    int stopMove()         → client_.StopMove();
};

// SportModeState 订阅（DDS channel）
sport_state_subscription_->InitChannel(callback, 1);
```

#### 3.4.4 线程安全

所有对 `safety_controller_` 的操作都通过 `std::mutex controller_mutex_` 保护（回调可能来自不同线程）。

---

## 4. 配置参数

### 4.1 pct_params.yaml（PurePursuitNode）

```yaml
pure_pursuit_node:
  ros__parameters:
    k: 0.5                          # 前视距离比例系数
    Lfc: 0.5                        # 前视距离基础偏移 (m)
    Kp: 1.0                         # (保留，未使用)
    dt: 0.1                         # (保留，未使用)
    goal_threshold: 0.15            # 终点位置容差 (m)
    final_heading_entry_distance: 0.20   # 距终点多少米进入朝向对准
    final_heading_command_deadband: 0.02 # 终点朝向死区 (rad)
    min_final_angular_velocity: 0.20     # 终点最小旋转角速度 (rad/s)
    max_acceleration: 0.08          # (未在 component 中使用)
    minCurvature: 0.0
    maxCurvature: 3.0
    minVelocity: 0.40               # 最小线速度 (m/s)
    maxVelocity: 0.40               # 最大线速度 (m/s) → 恒速 0.4
    maxAngularVelocity: 1.0         # 最大角速度 (rad/s)
    rotate_to_path_threshold: 1.047 # 触发原地旋转角度 (rad, ≈60°)
    rotate_to_path_tolerance: 0.349 # 旋转对准容差 (rad, ≈20°)
    goal_yaw_tolerance: 0.175       # 终点朝向容差 (rad, ≈10°)
    rotate_to_heading_gain: 2.0     # 旋转 P 增益
    final_approach_topic: /pct_scan_navigation/final_approach
    obstacle_th: 0.5                # (未接入)
    odom_topic: /Odometry_open3d
    odom_timeout: 1.0               # 里程计超时 (s)
```

### 4.2 go2_bridge_params.yaml（Go2CmdVelBridge）

```yaml
go2_cmd_vel_bridge:
  ros__parameters:
    dds_domain_id: 0
    sport_state_topic: "rt/sportmodestate"
    sdk_timeout: 0.5                # SDK Move 调用超时 (s)
    control_rate: 20.0              # 控制循环频率 (Hz)
    min_vx: 0.0
    max_vx: 0.40                    # 前进速度上限 (m/s)
    max_abs_vy: 0.0                 # 横向速度=0（禁止）
    max_abs_vyaw: 1.0               # 偏航角速度上限 (rad/s)
    max_linear_acceleration: 1.0    # 线加速度上限 (m/s²)
    max_yaw_acceleration: 1.2       # 角加速度上限 (rad/s²)
    command_timeout: 0.3            # cmd_vel 超时 (s)
    odometry_timeout: 1.0           # 里程计心跳超时 (s)
    sport_state_timeout: 0.5        # 运动状态心跳超时 (s)
```

---

## 5. 原始版 vs PCT 集成版对比

`src/pure_pursuit_planner_component_origin.cpp` 保留了原始版本（未编译进库），主要差异：

| 维度 | 原始版 | PCT 集成版 |
|---|---|---|
| 架构 | 单文件 all-in-one，ROS 逻辑和算法混在一起 | 算法 (`PurePursuitComponent`) 与 ROS (`PurePursuitNode`) 分离 |
| 路径话题 | `tgt_path` | `/pct_path` |
| 里程计话题 | `odom` | `/Odometry_open3d` |
| 曲率来源 | `ck = pose.position.z`（直接取 Z 坐标） | `ck = compute_curvature_from_path()`（从几何计算） |
| 朝向来源 | `cyaw = quaternion.yaw` | `cyaw = compute_yaw_from_path()`（从切线计算） |
| 障碍物 | 订阅 `local_obstacle_markers`，有避障逻辑 | 未接入（仅保留 `obstacle_th` 参数） |
| 路径更新 | `path_subscribe_flag` 防止重复处理 | 新路径总是完全替换旧路径 |
| 终点逻辑 | 简单 `goal_dist < goal_threshold` 停车 | final_approach / standalone 双模式 |
| 原地旋转 | 无 | 有 `rotate_to_path_` 状态机 |
| 可视化 | 发布 look_ahead / obstacle / target_point marker | 无（精简） |
| 参数校验 | 无 | 全面 |

---

## 6. 测试

### 6.1 Go2SafetyController 测试（`test_go2_safety_controller.cpp`）

使用 `FakeSportClient` mock Unitree SDK，覆盖 9 个场景：

| 测试 | 验证内容 |
|---|---|
| `StartsDisabledAndRequiresFreshHeartbeats` | 初始 disabled；缺少心跳时拒绝 enable；心跳齐全后成功 enable |
| `IgnoresPreArmCommandAndRequiresNewCommand` | armed 前的 cmd_vel 被忽略；enable 后需新命令才退出 waiting |
| `ClampsAndSlewLimitsCommands` | 速度硬限幅 + 加速度斜坡 |
| `ZeroCommandStopsImmediatelyButStaysArmed` | 零速度命令立即 StopMove 但不 disarm |
| `InvalidCommandStopsAndLatchesDisabled` | NaN 命令触发 disarm 并锁死 |
| `CommandTimeoutStopsAndRequiresReenable` | cmd_vel 超时 → disarm → 必须重新 enable |
| `HeartbeatTimeoutsStopAndDisarm` | 里程计/SportState 分别超时均触发 disarm |
| `SdkMoveFailureStopsAndDisarms` | SDK Move 返回错误码 → disarm |
| `ExplicitDisableAndShutdownAlwaysStop` | disable/shutdown 总是发 StopMove，shutdown 幂等 |

### 6.2 PurePursuitComponent CSV 测试（`test/test_all_functions_csv.cpp`）

从 CSV 文件读取输入/期望输出，逐个函数验证：

| CSV 数据文件 | 对应函数 |
|---|---|
| `calcFirstNearestPointIndex.csv` | `calcFirstNearestPointIndex()` |
| `calcOldNearestPointIndex.csv` | `calcOldNearestPointIndex()` |
| `calcLf.csv` | `calcLf()` |
| `searchTargetIndex.csv` | `searchTargetIndex()` |
| `computeVelocity.csv` | `computeVelocity()` |
| `curvatureToVelocity.csv` | `curvatureToVelocity()` |
| `calculateAngularVelocity.csv` | `calculateAngularVelocity()` |
| `CalculateControl.csv` | `purePursuitControl()` (原始版) |
| `isGoalReached.csv` | `isGoalReached()` |
| `alphaExceptionHandling.csv` | `alphaExceptionHandling()` |

### 6.3 PP Safety 集成测试（`test_pp_safety.cpp`）

PurePursuitNode + Go2SafetyController 的联合测试。

---

## 7. 数据流全链路

```
1. PCT 发布 /pct_path (nav_msgs/Path, frame_id=map)
   └─ pathCallback()
      ├─ 提取 raw_x, raw_y
      ├─ 计算切线 cyaw = compute_yaw_from_path()   ← 替代 PCT 的 identity quaternion
      ├─ 保留最终点朝向
      ├─ 计算离散曲率 ck = compute_curvature_from_path() ← 替代 PCT 的 z-height
      ├─ 清空旧的 odom_sub_flag / oldNearestPointIndex
      └─ path_received_ = true

2. open3d_loc 发布 /Odometry_open3d (nav_msgs/Odometry, map→base_link)
   └─ odomCallback()
      ├─ 提取 x, y, yaw, vx
      ├─ 非有限值 → 丢弃
      └─ pose_received_ = true, 更新 last_odom_time_

3. 100ms 定时器 → timerCallback()
   └─ planner_.computeVelocity(cx_, cy_, cyaw_, ck_, current_pose_, current_vx_, final_approach_)
      └─ 详见 3.1.2 算法流程
   └─ 发布 /cmd_vel

4. Go2CmdVelBridge::commandCallback(/cmd_vel)
   └─ safety_controller_->acceptCommand()
      ├─ clamp: vx ∈ [0, max_vx], vy=0, vyaw ∈ [-max_abs_vyaw, max_abs_vyaw]
      └─ zero → StopMove

5. Go2CmdVelBridge::controlTick() — 20Hz
   └─ safety_controller_->tick()
      ├─ 三层心跳检查
      ├─ 速度斜坡 (slew rate)
      └─ sport_client_.Move(vx, vy, vyaw) → Unitree SDK → Go2
```

---

## 8. 改进点总结（相比原始版）

| 改进 | 说明 |
|---|---|
| **算法/接口分离** | `PurePursuitComponent` 不依赖 ROS，可独立单元测试 |
| **PCT 协议适配** | 自动从路径几何计算 cyaw/ck，替代 PCT 的 Z-height hack |
| **安全控制器** | 独立的 `Go2SafetyController`：三层心跳 + 速度限幅 + 加速度斜坡 |
| **终点朝向对准** | `final_approach` 模式：接近终点时原地旋转对齐朝向 |
| **原地旋转保护** | 朝向偏差 > 60° 时先原地旋转再前进，避免 Pure Pursuit 蛇形收敛 |
| **全面参数校验** | 启动时拒绝非法参数，清晰报错 |
| **空/异常路径保护** | 收到空路径或含 NaN 路径立即停车并清空状态 |
| **里程计超时保护** | node 层和 bridge 层双重超时 → 零速停车 |
| **线程安全** | Bridge 对所有 controller 操作加 mutex |
| **可测试性** | `FakeSportClient` mock + CSV 数据驱动测试 |
