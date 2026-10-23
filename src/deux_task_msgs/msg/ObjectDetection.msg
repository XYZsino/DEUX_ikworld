# Pose origin must be the configured bag/model origin, in array.header.frame_id.
string track_id
geometry_msgs/PoseWithCovariance pose
float32 confidence
# True only when roll and pitch are observed as well as yaw.
bool full_orientation
