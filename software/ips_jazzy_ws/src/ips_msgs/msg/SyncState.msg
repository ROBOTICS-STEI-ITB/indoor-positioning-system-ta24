# SyncState.msg
std_msgs/Header header

bool   converged
uint32 sample_count

uint8[]   anchor_ids
float64[] skew
float64[] offset
float64[] residual_std_ns
