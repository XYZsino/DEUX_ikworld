#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source setup_env.bash
if [ ! -f install_lookup_bt/local_setup.bash ]; then
  echo 'Run ./build_lookup_bt.bash first (new LookupGrasp action).' >&2
  exit 1
fi
source install_lookup_bt/local_setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
exec ros2 launch "$PWD/lookup_bt.thor.launch.py" "$@"
