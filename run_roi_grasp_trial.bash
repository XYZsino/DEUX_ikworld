#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
# Source external message packages in the caller's shell or provide their setup path.
source /opt/ros/jazzy/setup.bash
if [ -n "${DEUX_INTERFACE_SETUP:-}" ]; then source "$DEUX_INTERFACE_SETUP"; fi
if [ -f .venv/bin/activate ]; then source .venv/bin/activate; fi
if [ ! -f install/local_setup.bash ]; then
  echo 'Build/source the Jazzy workspace first: install/local_setup.bash is missing.' >&2
  exit 1
fi
source install/local_setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$PWD/artifacts/roi-grasp-runs/ros-logs}"
exec python3 tools/roi_grasp_trial.py "$@"
