#!/usr/bin/env bash
set -e
source /home/DEUX003/DEUX_ikworld/setup_env.bash
exec ros2 launch /home/DEUX003/DEUX_ikworld/fixed_reference.launch.py "$@"
