#!/usr/bin/env bash
# Start the existing hardware lookup stack and submit one repeating RunTask.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
for startup_arg in "$@"; do
  case "$startup_arg" in
    -h|--help|--dry-run) exec python3 tools/start_lookup_stack.py "$@" ;;
  esac
done
if [[ -f /home/xyz_thor/deux_leader/install/local_setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
  source /home/xyz_thor/deux_leader/install/local_setup.bash
fi
source setup_env.bash
[[ -f install_lookup_bt/local_setup.bash ]] || { echo 'Run build_lookup_bt.bash first.' >&2; exit 1; }
source install_lookup_bt/local_setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
mkdir -p .setup/lookup-bt
exec 8>.setup/lookup-bt/start-stack.lock
flock -n 8 || { echo 'Lookup stack startup is already in progress.' >&2; exit 1; }
exec python tools/start_lookup_stack.py "$@"
