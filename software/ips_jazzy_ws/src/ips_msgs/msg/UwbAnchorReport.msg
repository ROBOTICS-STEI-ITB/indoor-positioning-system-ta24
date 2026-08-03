# UwbAnchorReport.msg
std_msgs/Header header

uint8 TYPE_MASTER_CLOCK = 0
uint8 TYPE_SLAVE_MASTER = 1
uint8 TYPE_SLAVE_TAG    = 2

uint8 report_type
uint8 reporter_id
uint32 session_id
uint8 source_id
uint32 seq
string tx_hex
string rx_hex
float64 pc_time_s
string src_ip
