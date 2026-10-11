#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
NAV_WS="$REPO_ROOT/Nav2_rea_ws"
ARM_WS="$REPO_ROOT/tzb_ws"
DOMAIN="${P6_ROS_DOMAIN_ID:-97}"
SCENARIO="${P6_SCENARIO:-normal}"
LOG_DIR="${P6_LOG_DIR:-/tmp/p6-autonomous-mission}"

if [[ ! "$DOMAIN" =~ ^[0-9]+$ ]] || (( DOMAIN < 0 || DOMAIN > 101 )); then
  echo "P6_ROS_DOMAIN_ID must be an integer from 0 through 101" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required at /opt/ros/humble/setup.bash" >&2
  exit 2
fi
if [[ ! -d "$NAV_WS/src/p6_autonomous_mission" || ! -d "$ARM_WS/src/lite_motion_planner" ]]; then
  echo "Run this script from a complete lunar-cave-robot checkout" >&2
  exit 2
fi

export P6_NAV_WS="$NAV_WS"
export P6_ARM_WS="$ARM_WS"
export P6_DOMAIN="$DOMAIN"
export P6_SCENARIO="$SCENARIO"
export P6_SCENARIO_MONITOR="$SCRIPT_DIR/scenario_monitor.py"
export P6_EVENT_LOG_PATH="${P6_EVENT_LOG_PATH:-$LOG_DIR/mission-events.jsonl}"
export ROS_DOMAIN_ID="$DOMAIN"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
mkdir -p "$LOG_DIR"

if [[ "${P6_SKIP_BUILD:-0}" != "1" ]]; then
  echo "Building Nav2 workspace independently..."
  (
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    cd "$NAV_WS"
    colcon build --base-paths src --packages-up-to p6_autonomous_mission \
      --build-base build/p6 --install-base install/p6 --parallel-workers "${P6_BUILD_JOBS:-4}"
  )
  echo "Building arm workspace independently..."
  (
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    cd "$ARM_WS"
    colcon build --base-paths src --packages-up-to lite_motion_planner \
      --packages-skip orbbec_camera --build-base build/p6 --install-base install/p6 \
      --parallel-workers "${P6_BUILD_JOBS:-4}"
  )
fi

if [[ "${1:-}" == "--build-only" ]]; then
  exit 0
fi
if [[ $# -gt 0 ]]; then
  echo "Unsupported argument: $1" >&2
  exit 2
fi

if [[ ! -f "$NAV_WS/install/p6/setup.bash" || ! -f "$ARM_WS/install/p6/setup.bash" ]]; then
  echo "Both independent offline installs must exist; build them or unset P6_SKIP_BUILD" >&2
  exit 2
fi

nav_pid=""
arm_pid=""
monitor_pid=""
stop_children() {
  set +e
  if [[ -z "$nav_pid$arm_pid$monitor_pid" ]]; then
    return
  fi
  local pid
  for pid in "$nav_pid" "$arm_pid" "$monitor_pid"; do
    [[ -n "$pid" ]] && kill -TERM -- "-$pid" 2>/dev/null
  done
  sleep 2
  for pid in "$nav_pid" "$arm_pid" "$monitor_pid"; do
    if [[ -n "$pid" ]] && kill -0 -- "-$pid" 2>/dev/null; then
      kill -KILL -- "-$pid" 2>/dev/null
    fi
  done
  [[ -n "$nav_pid" ]] && wait "$nav_pid" 2>/dev/null
  [[ -n "$arm_pid" ]] && wait "$arm_pid" 2>/dev/null
  [[ -n "$monitor_pid" ]] && wait "$monitor_pid" 2>/dev/null
  nav_pid=""
  arm_pid=""
  monitor_pid=""
}
trap stop_children EXIT INT TERM

if [[ -n "${P6_EXPECTED_STATE:-}" ]]; then
  export P6_EXPECTED_STATE
  setsid bash -c '
    set -e
    source /opt/ros/humble/setup.bash
    exec /usr/bin/python3 "$P6_SCENARIO_MONITOR"
  ' >"$LOG_DIR/scenario-monitor.log" 2>&1 &
  monitor_pid=$!
fi

setsid bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  source "$P6_ARM_WS/install/p6/setup.bash"
  exec ros2 launch lite_motion_planner p3b_offline.launch.py \
    offline_domain_id:="$P6_DOMAIN" autostart:=false \
    publish_synthetic_target:=true use_rviz:=false p6_staged_tasks:=true
' >"$LOG_DIR/arm.log" 2>&1 &
arm_pid=$!

setsid bash -c '
  set -e
  source /opt/ros/humble/setup.bash
  source "$P6_NAV_WS/install/p6/setup.bash"
  exec ros2 launch p6_autonomous_mission p6_autonomous_mission.launch.py \
    domain_id:="$P6_DOMAIN" scenario:="$P6_SCENARIO" \
    event_log_path:="$P6_EVENT_LOG_PATH"
' >"$LOG_DIR/nav2-mission.log" 2>&1 &
nav_pid=$!

echo "P6 offline simulation started (ROS_DOMAIN_ID=$DOMAIN, localhost only)."
echo "The arm workspace uses P3b synthetic feedback; no serial driver or physical device is launched."
echo "Logs: $LOG_DIR/arm.log and $LOG_DIR/nav2-mission.log"
echo "Mission event record: $P6_EVENT_LOG_PATH"
set +e
if [[ -n "$monitor_pid" ]]; then
  wait -n "$arm_pid" "$nav_pid" "$monitor_pid"
else
  wait -n "$arm_pid" "$nav_pid"
fi
status=$?
set -e
echo "A P6 process exited (status=$status). Stopping both launch groups." >&2
stop_children
exit "$status"
