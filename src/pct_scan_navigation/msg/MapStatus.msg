# 地图状态消息：导航管理节点发布当前地图的加载状态（/current_map）。
# 状态：UNLOADED 未加载 / LOADING 加载中 / LOADED 已加载 / FAILED 加载失败。
std_msgs/Header header

uint8 UNLOADED=0
uint8 LOADING=1
uint8 LOADED=2
uint8 FAILED=3
uint8 state

string map_name
string reason
