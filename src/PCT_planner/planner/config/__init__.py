# 文件：planner/config/__init__.py
# 用途：规划器配置包入口，统一导出参数配置类 Config。
# 结构：仅从 param 模块导出 Config，供 planner_wrapper 等模块加载规划参数。
from .param import Config