# 文件：plan.py
# 用途：ROS 1 命令行示例——按场景（Spiral/Building/Plaza）加载对应 tomogram，
#       规划固定起点到终点的路径并发布到 /pct_path。
# 结构：顶层完成 场景→(tomogram, 起终点) 映射；pct_plan() 执行加载与规划。
# 依赖：utils、planner_wrapper、config；需要 rospy（ROS 1）。
import sys
import argparse
import numpy as np

import rospy
from nav_msgs.msg import Path

from utils import *
from planner_wrapper import TomogramPlanner

sys.path.append('../')
from config import Config

# 场景选择参数（--scene）。
parser = argparse.ArgumentParser()
parser.add_argument('--scene', type=str, default='Spiral', help='Name of the scene. Available: [\'Spiral\', \'Building\', \'Plaza\']')
args = parser.parse_args()

cfg = Config()

# 场景 → (tomogram 文件名, 起终点) 映射；不同场景使用不同的离线地图。
if args.scene == 'Spiral':
    tomo_file = 'spiral0.3_2'
    start_pos = np.array([-16.0, -6.0], dtype=np.float32)
    end_pos = np.array([-26.0, -5.0], dtype=np.float32)
elif args.scene == 'Building':
    tomo_file = 'building2_9'
    start_pos = np.array([5.0, 5.0], dtype=np.float32)
    end_pos = np.array([-6.0, -1.0], dtype=np.float32)
else:
    tomo_file = 'plaza3_10'
    start_pos = np.array([0.0, 0.0], dtype=np.float32)
    end_pos = np.array([23.0, 10.0], dtype=np.float32)

# 路径发布器（latch 常驻，供 RViz 等订阅）。
path_pub = rospy.Publisher("/pct_path", Path, latch=True, queue_size=1)
planner = TomogramPlanner(cfg)

# 加载 tomogram 并规划固定起终点，成功后发布 /pct_path。
def pct_plan():
    planner.loadTomogram(tomo_file)

    traj_3d = planner.plan(start_pos, end_pos)
    if traj_3d is not None:
        path_pub.publish(traj2ros(traj_3d))
        print("Trajectory published")


if __name__ == '__main__':
    rospy.init_node("pct_planner", anonymous=True)

    pct_plan()

    rospy.spin()