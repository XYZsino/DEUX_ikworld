#!/usr/bin/env bash
# GUI-started upper-only fixed-height wait, velocity merge, qualification and stop.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source setup_env.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
export DEUX_MOVING_UPPER_REPEAT=0
# Server startup never requests motion. Existing GUI MoveToReady starts one trial.
exec python tools/upper_wait_trial.py
