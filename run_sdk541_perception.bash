#!/usr/bin/env bash
set -eo pipefail
root=/home/DEUX003/DEUX_ikworld
logs=/home/DEUX003/zed_setup/live-sdk541
exec 9>"$logs/stack.lock"
flock -n 9 || { echo 'SDK 5.4.1 perception stack is already running'; exit 1; }
source "$root/setup_env.bash"
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
sudo -n docker start zed-sdk541-live >/dev/null
pids=()
cleanup() {
  trap - EXIT INT TERM
  sudo -n docker exec zed-sdk541-live pkill -INT -f '^python3 /work/live_sdk541_bridge.py$' || true
  for pid in "${pids[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
}
trap cleanup EXIT INT TERM
setsid sudo -n docker exec zed-sdk541-live bash -c 'source /opt/ros/jazzy/setup.bash; export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp; exec python3 /work/live_sdk541_bridge.py' >"$logs/camera.log" 2>&1 &
pids+=("$!")
setsid ros2 launch deux_task_perception observation.launch.py config:="$root/observation.local.yaml" >"$logs/observation.log" 2>&1 &
pids+=("$!")
setsid bash "$root/run_perception.bash" >"$logs/perception.log" 2>&1 &
pids+=("$!")
echo "SDK 5.4.1 camera + observation + YOLO/ICP running. Logs: $logs"
wait -n "${pids[@]}"
