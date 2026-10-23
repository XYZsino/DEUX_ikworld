#!/usr/bin/env bash
set -e
source /home/DEUX003/DEUX_ikworld/setup_env.bash
export ROS_DOMAIN_ID=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec python /home/DEUX003/DEUX_ikworld/tools/pose_rgb_viewer.py
