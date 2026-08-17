# KN 导航环境镜像（ARM64）

本目录只用于原生 `linux/arm64`（`aarch64`）机器。与 AMD64 版本的命名区别：

| 架构 | Dockerfile | 推荐镜像名 |
|---|---|---|
| ARM64 | `docker/arm64/Dockerfile.arm64` | `cross-floor-nav:arm64-v1` |
| AMD64 | `docker/amd/Dockerfile` | `cross-floor-nav:amd64-v1` |

在 ARM64 机器上构建：

```bash
cd docker/arm64
docker buildx build \
  --platform linux/arm64 \
  --load \
  -t cross-floor-nav:arm64-v1 \
  -f Dockerfile.arm64 .
```

内存较小时建议添加 `--build-arg BUILD_JOBS=2`。首次构建需要下载并编译 ROS、
Livox、Unitree、PCT 和 KN 导航依赖，耗时较长。

运行：

```bash
docker run --rm -it \
  --name kn-nav-arm64 \
  --network host --ipc host --privileged \
  -v /实际地图目录:/home/nav_map:ro \
  cross-floor-nav:arm64-v1
```

导出与导入：

```bash
docker save -o cross-floor-nav-arm64-v1.tar cross-floor-nav:arm64-v1
docker load -i cross-floor-nav-arm64-v1.tar
```

Dockerfile 会主动检查目标架构，并确认 Unitree SDK2 的 `aarch64` 静态库存在，
防止误构建出 AMD64 镜像。Open3D 使用 ARM64 路径
`/usr/lib/aarch64-linux-gnu/cmake/Open3D`。
