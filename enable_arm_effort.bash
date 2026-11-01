#!/usr/bin/env bash
# Enable only the existing arm effort controller; no teleop/trajectory launch.
set -eo pipefail
case "${1:-enable}" in
  enable|status) action="${1:-enable}" ;;
  -h|--help)
    echo "Usage: $0 [enable|status]"
    echo 'Default: enable deux_arm_effort_controller. ROS_DOMAIN_ID defaults to 12.'
    echo 'Existing effort commands take effect on activation; no gravity node is launched.'
    exit 0 ;;
  *) echo "Usage: $0 [enable|status]" >&2; exit 2 ;;
esac
cd "$(dirname "${BASH_SOURCE[0]}")"
source ./setup_env.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
export RCUTILS_COLORIZED_OUTPUT=0
controller=deux_arm_effort_controller
manager=/arm/controller_manager
read_state() {
  local listing
  listing=$(ros2 control list_controllers -c "$manager") || return 1
  awk -v name="$controller" '$1 == name {print $NF; found=1} END {if (!found) exit 1}' <<< "$listing"
}
state=$(read_state) || { echo "Cannot read $controller from $manager" >&2; exit 1; }
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID $controller: $state"
if [[ "$action" == status || "$state" == active ]]; then exit 0; fi
if [[ "$state" != inactive ]]; then
  echo "Expected inactive controller; found $state. No change made." >&2
  exit 1
fi
echo 'Activating effort controller: existing torque commands will take effect.'
ros2 control set_controller_state "$controller" active -c "$manager"
state=$(read_state) || { echo 'Activation state could not be verified.' >&2; exit 1; }
[[ "$state" == active ]] || { echo "Activation failed: $state" >&2; exit 1; }
echo "$controller: active (verified)"
