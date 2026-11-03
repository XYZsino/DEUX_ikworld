#!/usr/bin/env bash
# Cancel BT, return arms to attention, then stop BT. --no-stow skips arm motion.
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
for arg in "$@"; do
  case "$arg" in
    -h|--help|--dry-run) exec python3 tools/stop_lookup_bt.py "$@" ;;
  esac
done
source setup_env.bash
if [[ -f install_lookup_bt/local_setup.bash ]]; then source install_lookup_bt/local_setup.bash; fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-12}"
mkdir -p .setup/lookup-bt
exec 8>.setup/lookup-bt/start-stack.lock
flock -n 8 || { echo 'Lookup stack startup/shutdown is already in progress.' >&2; exit 1; }
exec python tools/stop_lookup_bt.py "$@"
