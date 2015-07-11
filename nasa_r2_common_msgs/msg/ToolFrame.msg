Header header
string[] refFrame
string[] name
geometry_msgs/Pose[] location
float64[] toolMass
geometry_msgs/Vector3[] toolCog
nasa_r2_common_msgs/Inertia[] toolInertia
string[] momentumChain
uint8 actionType

uint8 RESET = 0
uint8 SET = 1
uint8 ADD = 2
uint8 REMOVE = 3
