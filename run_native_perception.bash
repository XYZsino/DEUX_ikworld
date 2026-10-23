#!/usr/bin/env bash
set -eo pipefail
root=/home/DEUX003/DEUX_ikworld
logs=/home/DEUX003/zed_setup/native-sdk541/perception
mkdir -p "$logs"
exec 9>"$logs/stack.lock"
flock -n 9 || { echo 'Native perception stack is already running'; exit 1; }
source "$root/setup_env.bash"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
setsid bash /home/DEUX003/zed_ros2_ws/run_zed2i.bash >"$logs/camera.log" 2>&1 &
pids+=("$!")
setsid bash "$root/run_merged_perception.bash" >"$logs/perception.log" 2>&1 &
pids+=("$!")
echo "Native SDK 5.4.1 camera + merged perception + lightweight state. Logs: $logs"
wait -n "${pids[@]}"
