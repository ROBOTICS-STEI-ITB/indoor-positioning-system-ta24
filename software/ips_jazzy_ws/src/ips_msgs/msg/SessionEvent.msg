# SessionEvent.msg
std_msgs/Header header

uint8 KIND_HELLO    = 0
uint8 KIND_RESET    = 1
uint8 KIND_HEARTBEAT = 2
uint8 KIND_AUTO_DETECTED_RESTART = 3

uint8 kind
uint8 device_id
uint32 session_id
string reason
