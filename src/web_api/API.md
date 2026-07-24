# KN 导航 Web API 接口文档

本文档面向接口调用方，描述 KN 导航系统当前提供的 HTTP API。调用方不需要了解 ROS 2 内部实现，只需按照本文档发送 HTTP 请求并处理响应。

对应服务程序：`ros2_service_api.py`

## 1. 基本信息

| 项目 | 值 |
|---|---|
| 协议 | HTTP |
| 默认端口 | `8000` |
| 数据格式 | JSON |
| 字符编码 | UTF-8 |
| API 前缀 | `/api` |
| Swagger 文档 | `/docs` |
| OpenAPI 描述 | `/openapi.json` |
| 全局坐标系 | `map` |
| 距离单位 | 米（m） |
| 角度单位 | 弧度（rad） |

服务与调用方在同一台机器时，Base URL 为：

```text
http://127.0.0.1:8000
```

服务运行在机器人上、调用方在其他机器时，应使用机器人的实际 IP：

```text
http://<机器人IP>:8000
```

例如：

```text
http://192.168.1.100:8000
```

`0.0.0.0` 只用于服务端监听，不能作为客户端请求地址。

## 2. 通用调用约定

### 2.1 请求头

所有包含 Body 的接口必须发送：

```http
Content-Type: application/json
```

Body 必须是 JSON 对象。例如：

```json
{
  "data": false
}
```

不要把整个对象作为字符串发送。下面的格式是错误的：

```json
"{\"data\": false}"
```

如果请求被作为 `text/plain` 发送，接口可能返回 HTTP 422，并提示输入不是有效对象。

### 2.2 API Key

服务端未设置 `KN_NAV_API_KEY` 时，不需要认证。

服务端设置了 `KN_NAV_API_KEY` 时，所有 `/api/*` 请求必须增加：

```http
X-API-Key: <服务端配置的密钥>
```

密钥缺失或错误时返回 HTTP 401：

```json
{
  "detail": "Invalid or missing X-API-Key"
}
```

`GET /`、`/docs` 和 `/openapi.json` 不进行 API Key 校验。

### 2.3 HTTP 状态与业务状态

业务接口通常返回：

```json
{
  "success": true,
  "message": "业务结果说明"
}
```

调用方必须同时检查：

1. HTTP 状态码；
2. JSON 中的 `success`。

HTTP 200 只表示 HTTP 请求和 ROS Service 调用已经完成，不保证业务成功。底层 ROS 节点可能返回：

```json
{
  "success": false,
  "message": "具体失败原因"
}
```

## 3. 接口总览

| 序号 | 方法 | 路径 | 说明 |
|---:|---|---|---|
| 1 | `POST` | `/api/open3d_loc/relocalize` | 使用完整四元数执行重定位 |
| 2 | `POST` | `/api/navigation/points` | 获取当前位置并保存为命名导航点 |
| 3 | `POST` | `/api/navigation/goal` | 发布保存点或直接坐标作为导航目标 |
| 4 | `POST` | `/api/go2_cmd_vel_bridge/enable` | 开启或关闭底盘速度桥 |
| 5 | `GET` | `/api/navigation/points` | 获取全部导航点 |
| 6 | `PATCH` | `/api/navigation/points/rename` | 只修改导航点名称 |
| 7 | `POST` | `/api/navigation/queue` | 将一个导航目标加入顺序执行队列 |
| 8 | `GET` | `/api/navigation/queue` | 查询当前任务、等待队列和历史记录 |
| 9 | `GET` | `/api/robot/status` | 获取机器狗当前位置、四元数、速度和时间戳 |
| 诊断 | `GET` | `/api/health` | 获取 API 和 ROS 网关状态 |
| 诊断 | `GET` | `/api/services` | 获取底层 ROS Service 就绪状态 |

## 4. 重定位

使用给定的 `base_link` 在 `map` 坐标系下的三维位置和四元数执行 Open3D 重定位。

### 4.1 请求

```http
POST /api/open3d_loc/relocalize
Content-Type: application/json
```

请求 Body：

```json
{
  "x": 1.2327408046653152,
  "y": -0.09142521170438922,
  "z": -0.5001805743966447,
  "qx": 0.0,
  "qy": 0.0,
  "qz": 0.741289779172096,
  "qw": 0.671185118499349
}
```

### 4.2 请求字段

| 字段 | 类型 | 必填 | 说明 |
|---|---|---|---|
| `x` | number | 是 | `map` 坐标系 X，单位 m |
| `y` | number | 是 | `map` 坐标系 Y，单位 m |
| `z` | number | 是 | `map` 坐标系 Z，单位 m |
| `qx` | number | 是 | 四元数 X |
| `qy` | number | 是 | 四元数 Y |
| `qz` | number | 是 | 四元数 Z |
| `qw` | number | 是 | 四元数 W |

所有字段必须是有限数值，不允许 `NaN` 或无穷值。四元数模长必须大于等于 `1e-6`。

API 会自动归一化合法的非单位四元数，并将归一化后的四元数发送给 ROS。

### 4.3 成功响应

```json
{
  "success": true,
  "message": "relocalization succeeded",
  "pose": {
    "x": 1.2327408046653152,
    "y": -0.09142521170438922,
    "z": -0.5001805743966447,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.741289779172096,
    "qw": 0.671185118499349
  },
  "input_quaternion_norm": 1.0,
  "quaternion_normalized": false
}
```

| 响应字段 | 类型 | 说明 |
|---|---|---|
| `success` | boolean | 重定位是否成功 |
| `message` | string | ROS 重定位结果说明 |
| `pose` | object | 实际发送给 ROS 的归一化位姿 |
| `input_quaternion_norm` | number | 输入四元数归一化前的模长 |
| `quaternion_normalized` | boolean | API 是否对输入四元数进行了归一化 |

### 4.4 业务失败响应

规定时间内没有收到新的有效定位结果：

```json
{
  "success": false,
  "message": "timeout waiting for relocalization result on /Odometry_open3d",
  "pose": {
    "x": 1.0,
    "y": 2.0,
    "z": 0.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0
  },
  "input_quaternion_norm": 1.0,
  "quaternion_normalized": false
}
```

已有重定位请求正在执行时：

```json
{
  "success": false,
  "message": "another relocalize request is running",
  "pose": {
    "x": 1.0,
    "y": 2.0,
    "z": 0.0,
    "qx": 0.0,
    "qy": 0.0,
    "qz": 0.0,
    "qw": 1.0
  }
}
```

重定位是长耗时接口。调用方应等待该请求完成，避免重复提交。

### 4.5 参数错误

四元数模长过小：

```http
HTTP/1.1 422 Unprocessable Entity
```

```json
{
  "detail": "Quaternion norm is too small"
}
```

字段缺失或类型错误时返回标准 FastAPI 422 响应。

## 5. 保存导航点（打点）

该接口获取机器人当前定位，并将其保存为具有唯一名称的导航点。

底层定位成功后才会写入当前离线地图对应的 JSON 文件。定位失败不会写入文件。每个导航点都会获得一个唯一的六位字符串编码。

### 5.1 请求

```http
POST /api/navigation/points
Content-Type: application/json
```

请求 Body：

```json
{
  "name": "office",
  "overwrite": false
}
```

### 5.2 请求字段

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| `name` | string | 是 | 无 | 导航点唯一名称，去除首尾空格后不能为空，最长 128 字符 |
| `overwrite` | boolean | 否 | `false` | 同名点存在时是否覆盖 |

调用方应发送标准 JSON 布尔值（小写）：

```json
false
```

不要依赖字符串到布尔值的自动转换：

```json
"false"
```

不能使用 Markdown 符号：

```text
**false**
```

### 5.3 成功响应

```json
{
  "success": true,
  "message": "Navigation point saved: office",
  "map_name": "global_map_downsize",
  "point": {
    "name": "office",
    "code": "483271",
    "frame_id": "map",
    "child_frame_id": "base_link",
    "x": 1.520143605688493,
    "y": 7.289045952913274,
    "z": -0.5119403031633517,
    "qx": 0.0,
    "qy": 0.0,
    "qz": -0.650156,
    "qw": 0.759801,
    "roll": 0.0,
    "pitch": 0.0,
    "yaw": -1.4155799076628224
  }
}
```

### 5.4 定位不可用

尚未获得有效定位时，HTTP 状态为 200，但 `success=false`：

```json
{
  "success": false,
  "message": "no current base_link pose from /Odometry_open3d"
}
```

### 5.5 同名点冲突

同名导航点存在且 `overwrite=false` 时：

```http
HTTP/1.1 409 Conflict
```

```json
{
  "detail": "Navigation point already exists: office"
}
```

需要覆盖时发送：

```json
{
  "name": "office",
  "overwrite": true
}
```

### 5.6 导航点存储

默认存储文件位于 API 脚本同目录，文件名取当前 `/global_localization_node` 的 `path_map` 参数中的地图文件名。例如：

```text
global_map_downsize.json
```

API 启动时：

- 文件不存在：自动创建并初始化为 `{}`；
- 文件为空：自动初始化为 `{}`；
- 文件包含合法 JSON：保留原内容；
- 旧导航点没有六位编码或编码重复：启动时自动补齐唯一编码；
- 文件非空但 JSON 损坏：不会覆盖，接口返回 HTTP 500。

## 6. 获取全部导航点

返回当前离线地图 JSON 中保存的全部导航点，按名称排序。

### 6.1 请求

```http
GET /api/navigation/points
```

该接口不接收 Body，也不支持 `?name=office` 查询单个点。

### 6.2 成功响应

```json
{
  "success": true,
  "map_name": "global_map_downsize",
  "map_path": "/home/nav_map/cross-floor-kn/global_map_downsize.pcd",
  "count": 2,
  "points": [
    {
      "name": "office",
      "code": "483271",
      "frame_id": "map",
      "child_frame_id": "base_link",
      "x": 1.520143605688493,
      "y": 7.289045952913274,
      "z": -0.5119403031633517,
      "qx": 0.0,
      "qy": 0.0,
      "qz": -0.650156,
      "qw": 0.759801,
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": -1.4155799076628224
    },
    {
      "name": "start",
      "code": "906154",
      "frame_id": "map",
      "child_frame_id": "base_link",
      "x": 1.2327408046653152,
      "y": -0.09142521170438922,
      "z": -0.5001805743966447,
      "qx": 0.0,
      "qy": 0.0,
      "qz": 0.741289779172096,
      "qw": 0.671185118499349,
      "roll": 0.0,
      "pitch": 0.0,
      "yaw": 1.6699799381943505
    }
  ]
}
```

没有导航点时：

```json
{
  "success": true,
  "map_name": "global_map_downsize",
  "map_path": "/home/nav_map/cross-floor-kn/global_map_downsize.pcd",
  "count": 0,
  "points": []
}
```

### 6.3 存储文件错误

```http
HTTP/1.1 500 Internal Server Error
```

```json
{
  "detail": "Unable to read navigation points file: ..."
}
```

## 7. 发布导航目标

该接口支持两种互斥的调用方式：

1. 使用已保存导航点的名称；
2. 直接提供 `x/y/z/yaw`。

`success=true` 只表示目标已经发布到 ROS `/goal_pose`，不表示路径规划成功、机器人开始运动或机器人已经到达。

### 7.1 使用保存点名称

```http
POST /api/navigation/goal
Content-Type: application/json
```

请求 Body：

```json
{
  "name": "office"
}
```

成功响应：

```json
{
  "success": true,
  "message": "goal published on /goal_pose",
  "goal": {
    "source": "saved_point",
    "x": 1.520143605688493,
    "y": 7.289045952913274,
    "z": -0.5119403031633517,
    "yaw": -1.4155799076628224,
    "name": "office"
  }
}
```

导航点不存在时：

```http
HTTP/1.1 404 Not Found
```

```json
{
  "detail": "Navigation point not found: office"
}
```

### 7.2 使用直接坐标

请求 Body：

```json
{
  "x": 3.0,
  "y": 1.0,
  "z": 0.4,
  "yaw": 1.57
}
```

字段说明：

| 字段 | 类型 | 必填 | 默认值 | 说明 |
|---|---|---|---|---|
| `x` | number | 是 | 无 | `map` 坐标系 X，单位 m |
| `y` | number | 是 | 无 | `map` 坐标系 Y，单位 m |
| `z` | number | 否 | `0.0` | `map` 坐标系 Z，单位 m |
| `yaw` | number | 否 | `0.0` | 目标航向角，单位 rad |

成功响应：

```json
{
  "success": true,
  "message": "goal published on /goal_pose",
  "goal": {
    "source": "coordinates",
    "x": 3.0,
    "y": 1.0,
    "z": 0.4,
    "yaw": 1.57
  }
}
```

### 7.3 错误组合

不能同时提供名称和坐标：

```json
{
  "name": "office",
  "x": 3.0,
  "y": 1.0
}
```

返回 HTTP 422：

```json
{
  "detail": "Provide either name or coordinates, not both"
}
```

未提供名称时，`x` 和 `y` 都是必填字段。

## 8. 开启或关闭底盘速度桥

底盘桥只有在开启状态下才会把 `/cmd_vel` 速度消息发送给机器人。

### 8.1 开启

```http
POST /api/go2_cmd_vel_bridge/enable
Content-Type: application/json
```

请求 Body：

```json
{
  "data": true
}
```

开启成功：

```json
{
  "success": true,
  "message": "bridge enabled; waiting for a new cmd_vel"
}
```

开启前，底盘桥要求：

- 已收到有效且未超时的定位心跳；
- 已收到有效且未超时的 Unitree SportModeState 心跳；
- 机器人停止指令调用成功。

条件不满足时，HTTP 状态通常为 200，但 `success=false`：

```json
{
  "success": false,
  "message": "cannot enable: odometry heartbeat is missing or stale"
}
```

或者：

```json
{
  "success": false,
  "message": "cannot enable: sport state heartbeat is missing or stale"
}
```

### 8.2 关闭

请求 Body：

```json
{
  "data": false
}
```

成功响应：

```json
{
  "success": true,
  "message": "bridge disabled and StopMove sent"
}
```

关闭操作会清空当前目标速度并要求机器人停止。

### 8.3 Body 类型错误

该接口要求 JSON 对象。如果请求被作为字符串或 `text/plain` 发送，会返回 HTTP 422：

```json
{
  "detail": [
    {
      "type": "model_attributes_type",
      "loc": ["body"],
      "msg": "Input should be a valid dictionary or object to extract fields from",
      "input": "{\n  \"data\": false\n}"
    }
  ]
}
```

遇到该错误时应检查请求头是否为：

```http
Content-Type: application/json
```

## 9. 健康检查

### 9.1 请求

```http
GET /api/health
```

### 9.2 响应

```json
{
  "success": true,
  "node": "kn_nav_fastapi_gateway_41569",
  "services": {
    "/open3d_loc/relocalize": true,
    "/open3d_loc/get_pose": true,
    "/open3d_loc/publish_goal": true,
    "/go2_cmd_vel_bridge/enable": true
  }
}
```

`success=true` 只表示 API 内部 ROS 节点和 executor 正常。具体业务是否可用，必须检查每个 Service 对应的布尔值。

## 10. ROS Service 状态

### 10.1 请求

```http
GET /api/services
```

### 10.2 响应

```json
{
  "success": true,
  "services": {
    "/open3d_loc/relocalize": true,
    "/open3d_loc/get_pose": true,
    "/open3d_loc/publish_goal": true,
    "/go2_cmd_vel_bridge/enable": false
  }
}
```

Service 为 `false` 时，对应业务接口通常返回 HTTP 503。

## 11. HTTP 错误码

| 状态码 | 含义 | 调用方处理建议 |
|---:|---|---|
| `200` | HTTP 和 ROS 调用完成 | 继续检查 JSON 中的 `success` |
| `401` | API Key 缺失或错误 | 检查 `X-API-Key` |
| `404` | 指定导航点不存在 | 刷新导航点列表或检查名称 |
| `409` | 保存点名称冲突 | 更换名称或使用 `overwrite=true` |
| `422` | Body、字段或数值不合法 | 检查 JSON 对象、字段类型和必填字段 |
| `500` | 导航点文件损坏或服务内部错误 | 记录 `detail` 并通知服务维护人员 |
| `502` | ROS Service 调用异常 | 检查 ROS 节点日志 |
| `503` | ROS Service 不可用 | 稍后重试并检查 `/api/services` |
| `504` | ROS Service 响应超时 | 提示超时，避免无限自动重试 |

HTTP 层错误通常使用：

```json
{
  "detail": "错误说明"
}
```

FastAPI 字段校验错误使用：

```json
{
  "detail": [
    {
      "type": "错误类型",
      "loc": ["body", "字段名"],
      "msg": "错误说明",
      "input": "错误输入"
    }
  ]
}
```

## 12. 建议调用流程

建议按以下顺序组织业务调用：

1. 调用 `GET /api/health`，确认 API 正常；
2. 调用 `GET /api/services`，确认所需 ROS Service 已就绪；
3. 必要时调用重定位接口，并等待 `success=true`；
4. 调用 `GET /api/navigation/points` 获取可选目标点；
5. 需要记录新位置时调用 `POST /api/navigation/points`；
6. 确认场地和机器人状态安全后，开启底盘速度桥；
7. 调用 `POST /api/navigation/goal` 发布目标；
8. 任务结束、异常或人工停止时，立即关闭底盘速度桥。

关闭底盘桥请求：

```json
{
  "data": false
}
```

## 13. 重试建议

- HTTP 422：请求数据错误，不应原样重试；
- HTTP 401、404、409：修正请求后再调用；
- HTTP 503：可以间隔重试，并同时查询 `/api/services`；
- HTTP 504：应提示用户，避免高频自动重试；
- `success=false`：根据 `message` 判断原因，不要无条件循环调用；
- 重定位接口执行期间不要再次提交重定位请求；
- 底盘桥启用失败时不要自动持续重试，应先检查定位和机器人状态心跳。

## 14. 部署与浏览器调用注意事项

当前 API 服务本身未配置 CORS。

如果调用程序运行在浏览器中，并且页面域名、协议或端口与 API 不一致，浏览器可能阻止跨域请求。部署时应由同源网关或反向代理转发 `/api`，或者由服务维护人员统一配置跨域策略。

调用方不应在代码中使用：

```text
http://0.0.0.0:8000
```

应使用：

```text
http://127.0.0.1:8000
```

或者机器人的实际网络地址。

## 15. 安全注意事项

- 发布目标和开启底盘桥是两个独立动作；发布目标不会自动保证底盘桥已开启。
- `success=true` 表示请求被相应节点接受，不表示机器人已经到达。
- 开启底盘桥前必须确认定位正确、目标安全、场地无人且急停措施可用。
- 调用方应始终提供明确的“停止/禁用底盘桥”操作。
- 网络异常、页面关闭或请求超时不能代替停止机器人，机器人仍由底盘桥自身的安全超时机制保护。
- 不要把具有底盘控制权限的 API Key 写入公开仓库或前端公开配置。
