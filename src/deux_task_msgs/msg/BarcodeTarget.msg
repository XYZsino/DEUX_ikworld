# Optical surface frame: +Z toward camera, +X projected image horizontal.
# Does not imply decoded bars or full bag orientation.
std_msgs/Header header
uint64 sequence
string request_id
string target_id
geometry_msgs/Pose pose
float64 plane_rmse_m
float64 valid_fraction
bool orientation_valid
# Identity of the currently observed barcode image track.
string barcode_track_id
