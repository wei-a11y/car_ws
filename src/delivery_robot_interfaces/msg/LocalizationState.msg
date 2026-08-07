uint8 WAITING_FOR_SENSORS=0
uint8 INITIALIZING=1
uint8 CONVERGING=2
uint8 LOCALIZED=3
uint8 LOST=4
uint8 FAILED=5

std_msgs/Header header
uint8 state
bool localized
bool initial_pose_sent
float64 xy_variance
float64 yaw_variance
uint16 stable_sample_count
uint16 error_code
string detail
