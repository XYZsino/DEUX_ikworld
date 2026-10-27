# RGB acquisition time; position is a visible surface centroid, NOT a bag pose.
std_msgs/Header header
uint64 sequence
string request_id
string target_id
bool bag_visible
bool barcode_visible
string barcode_track_id
bool surface_center_valid
geometry_msgs/Point surface_center
# Registered-depth bounds of the visible surface, not an amodal model box.
bool bbox3d_valid
geometry_msgs/Point bbox3d_min
geometry_msgs/Point bbox3d_max
