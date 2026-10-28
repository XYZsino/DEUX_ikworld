std_msgs/Header header
string plan_id
string scene_version
string target_id
string stage
# IK engine only. Conveyor timing and full-body path checks are shared.
string ik_backend
string grasp_candidate_id
# Optional absolute handoff and fixed Track/Descend/Close durations for timed search.
builtin_interfaces/Time grasp_start_at
float64[] grasp_phase_durations
string model_id
builtin_interfaces/Time valid_until
geometry_msgs/Pose target_pose
trajectory_msgs/JointTrajectory trajectory
uint32 ik_calls
float64 minimum_clearance
bool synthetic

# Present for AlignBarcode; acquisition-time surface snapshot.
deux_task_msgs/BarcodeTarget barcode_target
# Original ICP acquisition time. Never refreshed when a grasp snapshot is reused.
builtin_interfaces/Time target_acquired_at
# Moving-source forecast at original acquisition; trajectory.header is scheduled start.
bool follows_conveyor
geometry_msgs/Twist target_velocity

# TrackGrasp: cumulative Track, Descend, Close boundaries (seconds).
float64[] phase_end_times
