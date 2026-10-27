# RGB acquisition identity; depth and calibration retain their own source stamps.
std_msgs/Header header
uint64 sequence
string detector_id
sensor_msgs/Image depth
sensor_msgs/CameraInfo depth_info
BoundingBox[] boxes
# Request lease that selected this frame for ICP; empty for legacy producers.
string request_id
