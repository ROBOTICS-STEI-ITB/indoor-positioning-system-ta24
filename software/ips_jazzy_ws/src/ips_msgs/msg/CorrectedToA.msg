# CorrectedToA.msg
std_msgs/Header header

uint8  tag_id
uint8  slave_id
uint32 tag_seq
uint32 sync_seq

float64 toa_corrected_s
float64 toa_li_s
float64 toa_kf_s
float64 prop_delay_s

float64 delta_k
float64 theta_k_s
bool    kf_used
