uint8 DISABLED=0
uint8 WAIT_LOCALIZATION=1
uint8 WAIT_GLOBAL_PATH=2
uint8 VALIDATING_PATH=3
uint8 WAIT_PLANNER=4
uint8 TRACKING=5
uint8 FINAL_ALIGN=6
uint8 GOAL_REACHED=7
uint8 BLOCKED=8
uint8 CANCELED=9

std_msgs/Header header
uint64 task_id
uint8 state
string reason
bool motion_enabled
float64 distance_to_goal
float64 yaw_error
