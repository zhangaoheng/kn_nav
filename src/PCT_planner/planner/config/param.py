# 文件：planner/config/param.py
# 用途：定义 PCT 规划器与封装层的默认参数（纯 Python 配置，无 ROS 依赖）。
# 结构：ConfigPlanner —— 规划算法参数；ConfigWrapper —— tomogram 存放目录；
#       Config —— 汇总入口（planner + wrapper）。
# 数据流：planner_wrapper.TomogramPlanner 通过 Config() 读取这些参数。
class ConfigPlanner():
    use_quintic = True
    max_heading_rate = 10
    a_star_cost_threshold = 20.0
    step_cost_weight = 0.50
    optimizer_cost_threshold = 10.0
    safe_cost_margin = optimizer_cost_threshold
    use_clearance_cost = True
    clearance_cost_mode = "relative"
    clearance_cost_weight = 20.0
    clearance_cost_local_radius = 2.0
    clearance_cost_cap = 20.0
    clearance_cost_decay = 1.0


# ConfigWrapper：封装层配置，tomo_dir 为离线 tomogram（pickle）所在相对目录。
class ConfigWrapper():
    tomo_dir = '/rsc/tomogram/'


# Config：配置入口，聚合规划器参数与封装层参数。
class Config():
    planner = ConfigPlanner()
    wrapper = ConfigWrapper()
