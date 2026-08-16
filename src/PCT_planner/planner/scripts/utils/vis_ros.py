# =============================================================================
# vis_ros.py — 轨迹转 ROS 消息工具
# 把 (N,3) 的轨迹点序列转换成 nav_msgs/Path 消息，供 RViz 显示。
# =============================================================================
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


# 逐点构造 PoseStamped 写入 Path（frame 固定为 map），未提供姿态时朝向取默认值。
def traj2ros(traj):
    path_msg = Path()
    path_msg.header.frame_id = "map"

    for waypoint in traj:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = waypoint[0]
        pose.pose.position.y = waypoint[1]
        pose.pose.position.z = waypoint[2]
        pose.pose.orientation.w = 1
        path_msg.poses.append(pose)

    return path_msg