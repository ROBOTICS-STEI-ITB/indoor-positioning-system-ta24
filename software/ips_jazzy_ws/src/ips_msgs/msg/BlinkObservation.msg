# BlinkObservation.msg
std_msgs/Header header

uint8  tag_id
uint32 seq

uint8[]   anchor_ids
float64[] toa_corrected_s
float64[] sync_quality
