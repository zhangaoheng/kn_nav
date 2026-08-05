# config_v2 分层配置

该目录用于开发新版工程化配置结构，目前处于草案阶段，尚未接入导航启动文件。

当前实机仍使用：

```text
config/A2/navigation.yaml
```

目录职责：

```text
config_v2/
├── schema/       配置格式和校验规则
├── defaults/     算法默认参数
├── robots/       机械狗尺寸、外参和硬件限制
├── maps/         各套配置的地图路径和地图定位阈值
├── deployments/ 现场常用配置入口
└── generated/    自动生成的完整运行配置
```

`generated` 中的文件由后续配置工具生成，不应手动修改。

## 实机部署参数

现场调试主要修改 `deployments/a2_outdoor.yaml` 中的 `tuning`：

- `localization`：ICP搜索范围、接受阈值和单次位姿变化限制；
- `local_planner`：规划视距、滑动地图大小和更新范围；
- `robot_geometry`：双圆柱模型和障碍物上下膨胀；
- `unitree`：机械狗状态心跳话题；
- `motion`：实机测试速度和加速度限制。

这些字段以《使用.md》的“主要参数调整”为参考。当前仅作为新版配置草案，尚未传入ROS节点。

当前已迁移 `A2`、`local`、`unitree_go2` 和 `unitree_go2w` 四套配置。
