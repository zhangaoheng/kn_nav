# =============================================================================
# param.py — tomography 配置参数
# ConfigROS：ROS 话题与坐标系配置；ConfigMap：导出目录；Config：聚合入口。
# =============================================================================
# ROS 话题与坐标系配置：点云、逐层 G/C 层、tomogram 可视化话题。
class ConfigROS():
    map_frame = "map"

    pointcloud_topic = "/global_points"
    layer_G_topic = "/layer_G_"
    layer_C_topic = "/layer_C_"
    tomogram_topic = "/tomogram"


# 建图结果（tomogram pickle）的导出目录。
class ConfigMap():
    export_dir = "/rsc/tomogram/"


# 顶层配置聚合：ros 与 map 两类配置的统一入口。
class Config():
    ros = ConfigROS()
    map = ConfigMap()