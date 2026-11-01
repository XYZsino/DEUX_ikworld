#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source setup_env.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
export DEUX_MOVING_UPPER_ASYNC=1 DEUX_MOVING_UPPER_AUTO=1 DEUX_READY_AUTO=1
# One task request: right open -> ready -> upper -> fresh descent/close -> stop.
# Do not run alongside another /task/move_to_ready server.
exec python tools/descent_trial.py
