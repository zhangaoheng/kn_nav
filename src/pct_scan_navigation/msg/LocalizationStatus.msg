# 定位状态消息：全局定位节点向导航管理节点上报定位质量。
# 状态机：UNINITIALIZED -> INITIALIZING -> INIT_SUCCESS -> TRACKING，
# 配准退化时进入 TRACKING_WARN，丢失时 TRACKING_LOST（导航管理据此软停）。
# 字段：state 状态、map_name 当前地图名、fitness 配准分数、reason 补充说明。
std_msgs/Header header

uint8 UNINITIALIZED=0
uint8 INITIALIZING=1
uint8 INIT_SUCCESS=2
uint8 TRACKING=3
uint8 TRACKING_WARN=4
uint8 TRACKING_LOST=5
uint8 MAP_SWITCHING=6
uint8 state

string map_name
float32 fitness
string reason
