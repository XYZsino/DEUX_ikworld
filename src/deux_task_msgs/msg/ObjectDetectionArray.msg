# Original acquisition timestamp, NOT ICP result arrival time.
std_msgs/Header header
uint64 sequence
string source
string model_id
ObjectDetection[] detections

# Empty for legacy standalone producers.
string request_id
