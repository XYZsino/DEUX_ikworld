#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source ./setup_env.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
mkdir -p .setup
exec 9>.setup/calibrated-zed.lock
flock -n 9 || { echo "Calibrated ZED stack already running"; exit 1; }
exec ros2 launch "$PWD/calibrated_zed.thor.launch.py"
