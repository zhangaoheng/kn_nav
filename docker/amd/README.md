# KN 导航环境镜像（AMD64）

这份镜像复现当前容器的核心环境：Ubuntu 22.04、ROS 2 Humble、Python
3.10、Open3D 0.14.1、PCL、OpenCV、Unitree SDK2、Livox SDK2、Livox ROS
Driver 2 和 KN 导航工作区。源码版本已在 `Dockerfile` 中固定。

## 在另一台机器构建

要求：x86_64 Linux、Docker 24+，并能访问 Ubuntu、ROS 和 GitHub。只需复制
本目录，或从 KN 导航仓库取得本目录，不需要复制当前容器的 `build/`、`install/`
或 1.6 GB 的 Open3D 压缩包。

```bash
cd docker/amd
docker buildx build \
  --platform linux/amd64 \
  --load \
  -t cross-floor-nav:amd-v1 \
  -f Dockerfile .
```

网络较慢时可让构建使用宿主机网络：

```bash
docker buildx build --network=host --platform linux/amd64 --load \
  -t cross-floor-nav:amd-v1 -f Dockerfile .
```

内存不足时降低并行编译数：

```bash
docker buildx build --build-arg BUILD_JOBS=2 --platform linux/amd64 \
  --load -t cross-floor-nav:amd-v1 -f Dockerfile .
```

如需构建另一个代码版本，传入完整 commit 或 tag，例如：

```bash
docker buildx build --build-arg KN_NAV_REF=<commit-or-tag> \
  --platform linux/amd64 --load -t cross-floor-nav:amd-v1 -f Dockerfile .
```

## 运行

交互验证：

```bash
docker run --rm -it --network host --ipc host --privileged \
  -v /home/unitree/nav_map:/home/nav_map:ro \
  cross-floor-nav:amd-v1
```

也可使用同目录脚本（第一个参数是镜像名）：

```bash
chmod +x run_container.sh
NAV_MAP_DIR=/实际地图目录 ./run_container.sh cross-floor-nav:amd-v1
```

入口脚本会自动加载 ROS、Livox 和 KN 导航工作区环境。地图、雷达设备和机器人
网卡属于宿主机运行配置，不应固化到镜像中。

## 离线搬运

联网机器构建后：

```bash
docker save -o cross-floor-nav-amd-v1.tar cross-floor-nav:amd-v1
```

目标机器导入：

```bash
docker load -i cross-floor-nav-amd-v1.tar
```

注意：该 Dockerfile 是 `linux/amd64` 版本，不能直接用于 ARM64/Jetson。ARM64
需要对应架构的 ROS 基础镜像以及 Unitree/Open3D 依赖。
