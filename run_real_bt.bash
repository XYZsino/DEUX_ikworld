#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source setup_env.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
mkdir -p .setup
exec 9>.setup/real-bt.lock
flock -n 9 || { echo 'Real BT stack already running'; exit 1; }
python tools/ensure_bt_state.py
# Confirm hand Leader modes before starting task/execution nodes.
python tools/configure_hand_modes.py
exec ros2 launch "$PWD/real_bt.thor.launch.py"
