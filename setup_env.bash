#!/usr/bin/env bash
_ikworld_root="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONNOUSERSITE=1
source /opt/ros/jazzy/setup.bash
source /home/xyz_thor/DEUX/install/local_setup.bash
source /home/xyz_thor/zed_ws/install/local_setup.bash
source "$_ikworld_root/.venv/bin/activate"
if [ -f "$_ikworld_root/install/local_setup.bash" ]; then
  source "$_ikworld_root/install/local_setup.bash"
fi
export ROS_LOG_DIR="$_ikworld_root/.setup/ros-logs"
export YOLO_CONFIG_DIR="$_ikworld_root/.setup/ultralytics"
export JAX_PLATFORMS=cpu
unset _ikworld_root
