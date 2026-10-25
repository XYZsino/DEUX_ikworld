std_msgs/Header header
string plan_id
string scene_version
string target_id
string stage
string model_id
builtin_interfaces/Time valid_until
geometry_msgs/Pose target_pose
trajectory_msgs/JointTrajectory trajectory
uint32 ik_calls
float64 minimum_clearance
bool synthetic

# Present for AlignBarcode; acquisition-time surface snapshot.
deux_task_msgs/BarcodeTarget barcode_target
