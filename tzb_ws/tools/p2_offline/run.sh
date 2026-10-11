#!/usr/bin/env bash
set -eo pipefail
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source /opt/ros/humble/setup.bash
source "$WS/install/offline/setup.bash"
set -u
export ROS_DOMAIN_ID=187
export ROS_LOCALHOST_ONLY=1
exec ros2 launch lite_motion_planner p2_offline.launch.py use_rviz:="${P2_OFFLINE_RVIZ:-false}"
