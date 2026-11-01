#!/usr/bin/env bash
# Start state + gravity/effort + camera + BT + GUI; keep existing services. No task/motion is started.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source ./setup_env.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
mkdir -p .setup
exec 8>.setup/real-stack-start.lock
flock -n 8 || { echo 'Another stack startup is in progress'; exit 1; }
start_service() {
  local name="$1" pattern="$2" logfile="$3"
  shift 3
  if pgrep -f "$pattern" >/dev/null; then
    echo "$name: already running"
    return
  fi
  # Do not let background processes inherit the startup lock.
  nohup "$@" >"$logfile" 2>&1 < /dev/null 8>&- &
  local pid=$!
  sleep 2
  if ! kill -0 "$pid" 2>/dev/null; then
    echo "$name: startup failed; see $logfile" >&2
    tail -20 "$logfile"
    return 1
  fi
  echo "$name: started PID $pid; log $logfile (initialization may still be running)"
}
state_file="${DEUX_STATE_FILE:-/home/xyz_thor/DEUX/deux_state.env}"
[[ -r "$state_file" ]] || { echo "Missing robot state file: $state_file" >&2; exit 1; }
start_service State '[/]lib/deux_state/state_node|[r]os2 launch deux_state state.launch.py' .setup/state-current.log ros2 launch deux_state state.launch.py "state_file:=$state_file"
start_service Gravity '[r]os2 launch deux_gravity deux_gravity.launch.py|[/]lib/deux_gravity/deux_gravity( |$)' .setup/gravity-current.log ros2 launch deux_gravity deux_gravity.launch.py
# Require a live torque stream before activating its controller.
if ! timeout 15 ros2 topic echo /arm/deux_arm_effort_controller/commands std_msgs/msg/Float64MultiArray --once > .setup/effort-startup-sample.log; then
  echo 'No effort command received; stopping startup.' >&2
  exit 1
fi
bash enable_arm_effort.bash
start_service ZED '[r]os2 launch .*/calibrated_zed.thor.launch.py' .setup/zed-current.log bash run_calibrated_zed.bash
# Reconfirm both hands even when the BT process is reused.
python tools/configure_hand_modes.py
start_service BT '[r]os2 launch .*/real_bt.thor.launch.py' .setup/bt-current.log bash run_real_bt.bash
start_service GUI '[r]os2 launch deux_task_gui control.launch.py|[/]lib/deux_task_gui/task_gui' .setup/gui-current.log ros2 launch deux_task_gui control.launch.py deployment:=real
echo 'GUI: remote localhost:8080; token is in .setup/gui-current.log.'
echo 'On your workstation: ssh -N -L 18080:127.0.0.1:8080 xyz_thor@192.168.20.213'
echo 'Services remain running after this launcher exits. Start motions manually in the GUI.'
