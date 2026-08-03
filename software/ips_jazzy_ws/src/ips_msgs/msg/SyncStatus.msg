# SyncStatus.msg
std_msgs/Header header

uint32 reset_count
uint32 dropped_packets_on_reset

uint8[]   slave_ids
uint32[]  sync_count
bool[]    li_ready
bool[]    kf_converged
uint32[]  kf_updates
float64[] delta_k
float64[] theta_k_s
float64[] kf_drift
float64[] kf_drift_rate
