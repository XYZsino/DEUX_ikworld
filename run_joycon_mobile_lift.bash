#!/usr/bin/env bash
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == --help || "${1:-}" == -h ]]; then
  echo 'Usage: ./run_joycon_mobile_lift.bash [check_only:=true] [left_device:=NAME] [right_device:=NAME]'
  echo 'Default ROS_DOMAIN_ID=12. Requires running mobile/lift hardware drivers. Ctrl-C stops.'
  exit 0
fi
source "$root/setup_env.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
mkdir -p "$root/.setup"
exec 9>"$root/.setup/joycon-mobile-lift.lock"
flock -n 9 || { echo 'This launcher is already running.' >&2; exit 1; }
ros2 pkg prefix joy >/dev/null
exec ros2 launch "$root/joycon_mobile_lift.launch.py" "$@"
