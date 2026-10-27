std_msgs/Header header
uint64 sequence
string request_id
string detector_id
deux_task_msgs/BoundingBox[] boxes
# Pixel rectangle enclosing the configured selection workspace projected into this RGB.
bool selection_roi_valid
float64[4] selection_roi_xyxy
