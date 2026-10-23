# Camera observation, not an actuator command or a success verdict.
# header.stamp = RGB acquisition; header.frame_id = actual optical frame.
std_msgs/Header header
uint64 sequence
sensor_msgs/Image rgb
# 32FC1 metres; invalid/nonpositive samples are NaN. Original depth stamp preserved.
sensor_msgs/Image depth
sensor_msgs/CameraInfo rgb_info
sensor_msgs/CameraInfo depth_info
float32 valid_depth_fraction
# Optional transform at RGB time. Never use unless valid is true.
bool camera_transform_valid
geometry_msgs/TransformStamped base_from_camera
# Optional measured state; never filled from commanded positions.
bool joints_valid
sensor_msgs/JointState joints
# Pose results can arrive late; retain their acquisition header and sequence.
bool objects_valid
ObjectDetectionArray objects
# Original measurements retain separate acquisition times; joints.header uses oldest.
sensor_msgs/JointState arm_state
sensor_msgs/JointState hand_state
# Optional calibrated grasp-frame transforms at RGB acquisition time.
bool left_ee_valid
geometry_msgs/TransformStamped base_from_left_ee
bool right_ee_valid
geometry_msgs/TransformStamped base_from_right_ee
