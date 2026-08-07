uint8 STATUS_PENDING=0
uint8 STATUS_PICKING=1
uint8 STATUS_STORED=2
uint8 STATUS_DELIVERING=3
uint8 STATUS_DELIVERED=4
uint8 STATUS_FAILED=5

std_msgs/Header header
string order_id
string locker_id
uint8 locker_slot
geometry_msgs/PoseStamped delivery_pose
builtin_interfaces/Time deadline
uint8 priority
uint8 status
