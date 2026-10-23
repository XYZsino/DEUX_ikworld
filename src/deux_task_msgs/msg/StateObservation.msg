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
sensor_msgs/JointState joints
sensor_msgs/JointState arm_state
sensor_msgs/JointState hand_state
bool left_ee_valid
geometry_msgs/TransformStamped base_from_left_ee
bool right_ee_valid
geometry_msgs/TransformStamped base_from_right_ee
