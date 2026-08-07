uint16 ACCEPTED=0
uint16 INVALID_TASK=1001
uint16 DUPLICATE_TASK=1002
uint16 QUEUE_FULL=1003
uint16 UNCALIBRATED_SITE=1004

std_msgs/Header header
string task_id
uint32 revision
bool accepted
uint16 reason_code
string message
uint16 queue_position
