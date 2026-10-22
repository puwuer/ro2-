# 系统工作模式常量
uint8 PATROL=0
uint8 TRACK=1
uint8 SEARCH=2

std_msgs/Header header

# 当前工作模式: PATROL / TRACK / SEARCH
uint8 mode

# 是否检测到跟踪目标(人)
bool target_visible

# 目标在 map 坐标系下的位置, target_visible=false 时忽略
geometry_msgs/PoseStamped target_pose

# 检测置信度 0~1
float32 confidence

# 当前(或最近)巡检点索引, 非巡检模式时保留最后一次的值
int32 waypoint_index

# 最近一次状态切换原因/说明, 仅用于调试显示
string detail
