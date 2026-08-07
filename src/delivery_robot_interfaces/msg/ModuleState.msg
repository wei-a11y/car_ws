uint8 MODULE_CHASSIS=1
uint8 MODULE_ARM=2
uint8 MODULE_CARGO=3

uint8 STATE_UNCONFIGURED=0
uint8 STATE_READY=1
uint8 STATE_BUSY=2
uint8 STATE_FAULT=3
uint8 STATE_SAFE_STOP=4

std_msgs/Header header
uint8 module
uint8 lifecycle_state
uint8 operation_state
bool ready
bool busy
bool transport_safe
bool emergency_stopped
uint16 fault_code
string detail
