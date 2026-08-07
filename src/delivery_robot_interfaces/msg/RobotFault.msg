uint8 INFO=0
uint8 WARNING=1
uint8 ERROR=2
uint8 FATAL=3

std_msgs/Header header
uint16 code
uint8 severity
string module
string description
bool recoverable
