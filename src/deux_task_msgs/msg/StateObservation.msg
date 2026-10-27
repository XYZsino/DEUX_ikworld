# Lightweight state at aggregation time. No image payloads.
# Object acquisition header/sequence remain inside objects, never relabeled.
std_msgs/Header header
uint64 sequence
bool objects_valid
ObjectDetectionArray objects
# True only while a fresh perception result (including an empty result) exists.
bool perception_fresh
bool camera_transform_valid
geometry_msgs/TransformStamped base_from_camera
bool joints_valid
# Summary only. Actual joint arrays go directly to planner/execution.
builtin_interfaces/Time joints_acquired_at
string joints_reason
bool left_ee_valid
geometry_msgs/TransformStamped base_from_left_ee
bool right_ee_valid
geometry_msgs/TransformStamped base_from_right_ee
