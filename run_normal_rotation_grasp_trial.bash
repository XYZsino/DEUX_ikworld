#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source setup_env.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
exec python tools/grid_grasp_trial.py --ros-args \
  -p grid_library_directory:="$PWD/artifacts/right_grasp_grid_rotations" \
  -p use_observed_yaw:=true \
  -p grid_output_directory:="$PWD/.setup/grid-grasp-rotation-test" "$@"
