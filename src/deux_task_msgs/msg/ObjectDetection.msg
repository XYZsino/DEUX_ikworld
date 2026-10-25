# Pose origin must be the configured bag/model origin, in array.header.frame_id.
string track_id
geometry_msgs/PoseWithCovariance pose
float32 confidence
# True only when roll and pitch are observed as well as yaw.
bool full_orientation
# Fitting metrics, not ground-truth accuracy. Existing external producers may omit.
bool fit_metrics_valid
float64 trimmed_rmse_m
float64 target_coverage_10mm
# False for configured uncertainty assumptions used by yaw_icp.
bool covariance_estimated
# True when observation_node applied the optional acquisition-time XY/yaw filter.
bool pose_filtered
