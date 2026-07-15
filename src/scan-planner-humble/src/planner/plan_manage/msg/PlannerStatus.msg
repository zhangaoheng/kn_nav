uint8 IDLE=0
uint8 WAIT_MAP=1
uint8 PLANNING=2
uint8 EXECUTING=3
uint8 REPLANNING=4
uint8 BLOCKED=5
uint8 CANCELED=6

std_msgs/Header header
uint64 task_id
uint8 state
string reason
bool odom_ready
bool map_ready
