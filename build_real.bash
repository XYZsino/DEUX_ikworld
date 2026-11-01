#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
source ./setup_env.bash
python -m colcon build --base-paths src --packages-skip deux_task_sim --symlink-install --cmake-args -DPython3_EXECUTABLE="$VIRTUAL_ENV/bin/python" -DPYTHON_EXECUTABLE="$VIRTUAL_ENV/bin/python"
