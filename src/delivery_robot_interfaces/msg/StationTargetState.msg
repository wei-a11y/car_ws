uint8 STATION_LOCKER=1
uint8 STATION_RECEIVER=2

std_msgs/Header header
uint8 station_type
string station_id
uint8 locker_slot
bool station_detected
bool target_detected
bool door_open
bool pose_valid
geometry_msgs/PoseStamped target_pose
float32 confidence
