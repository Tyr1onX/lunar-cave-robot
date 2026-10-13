#!/usr/bin/env bash
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$HERE/../.." && pwd)"
if [[ ! -f "$ROOT/Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/cave_simulator.py" ]]; then
  echo "P6 simulator missing; refusing to start" >&2
  exit 1
fi
# Force a dedicated localhost-only DDS graph. Never source a robot-control overlay.
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=217
export ROS_LOCALHOST_ONLY=1
export PYTHONPATH="$ROOT/Nav2_rea_ws/src/p6_autonomous_mission${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$HERE/sim_server.py"
