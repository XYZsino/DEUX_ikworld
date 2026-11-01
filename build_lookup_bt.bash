#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source setup_env.bash
python -m colcon build --base-paths src --packages-select deux_task_msgs \
  --build-base build_lookup_bt --install-base install_lookup_bt \
  --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python"
