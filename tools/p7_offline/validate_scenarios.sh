#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
RESULTS="${P7_RESULTS_DIR:-/tmp/p7-end-to-end-results}"
DOMAIN_START="${P7_DOMAIN_START:-50}"
BUILD_JOBS="${P7_BUILD_JOBS:-4}"
NAV_TIMEOUT="${P7_NAV_TIMEOUT_SEC:-90}"
MISSION_TIMEOUT="${P7_MISSION_TIMEOUT_SEC:-300}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required at /opt/ros/humble/setup.bash" >&2
  exit 2
fi
if [[ ! "$DOMAIN_START" =~ ^[0-9]+$ ]] || (( DOMAIN_START < 2 || DOMAIN_START > 101 )); then
  echo "P7_DOMAIN_START must be an integer in 2..101" >&2
  exit 2
fi
mkdir -p "$RESULTS"
RUN_ID="$(date -u +%Y%m%dT%H%M%SZ)-$$"
RUN_RESULTS="$RESULTS/$RUN_ID"
mkdir -p "$RUN_RESULTS"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
unset ROS_DISCOVERY_SERVER || true

if [[ "${P7_SKIP_BUILD:-0}" != "1" ]]; then
  (
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    cd "$REPO_ROOT/Nav2_rea_ws"
    colcon build --base-paths src --packages-up-to p6_autonomous_mission chassis_serial_bridge \
      --build-base build/p7 --install-base install/p7 --parallel-workers "$BUILD_JOBS"
  )
  (
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    cd "$REPO_ROOT/tzb_ws"
    colcon build --base-paths src --packages-up-to lite_motion_planner rm_serial_driver \
      --packages-skip orbbec_camera --build-base build/p7 --install-base install/p7 \
      --parallel-workers "$BUILD_JOBS"
  )
fi

case_index=0
run_case() {
  local name="$1" scenario="$2" expected="$3" fault="${4:-}" nav_timeout="${5:-$NAV_TIMEOUT}"
  if [[ -n "${P7_CASE_FILTER:-}" && ",${P7_CASE_FILTER}," != *",${name},"* ]]; then
    return
  fi
  local domain=$((DOMAIN_START + case_index))
  local case_dir="$RUN_RESULTS/$name"
  case_index=$((case_index + 1))
  if (( domain > 101 )); then
    echo "P7 case domains exceed ROS_DOMAIN_ID range 0..101" >&2
    exit 2
  fi
  mkdir -p "$case_dir"
  echo "=== P7 $name: scenario=$scenario expected=$expected fault=${fault:-none} domain=$domain ==="
  P7_SKIP_BUILD=1 \
  P7_ROS_DOMAIN_ID="$domain" \
  P7_SCENARIO="$scenario" \
  P7_EXPECTED_STATE="$expected" \
  P7_FAULT="$fault" \
  P7_NAV_TIMEOUT_SEC="$nav_timeout" \
  P7_MISSION_TIMEOUT_SEC="$MISSION_TIMEOUT" \
  P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator \
  P7_LOG_DIR="$case_dir" \
    "$REPO_ROOT/tools/p7_offline/run.sh" 2>&1 | tee "$case_dir/run.log"
  grep -q '^P7_ACCEPTANCE_RESULT=PASS$' "$case_dir/run.log"
}

run_case normal normal SUCCEEDED
run_case nav_failure nav_failure ABORTED
run_case target_loss target_loss ABORTED
run_case grasp_failure grasp_failure ABORTED
run_case communication_loss communication_loss ABORTED
run_case duplicate_trigger duplicate_trigger SUCCEEDED
run_case arm_feedback_stop normal ABORTED arm_feedback_stop
run_case arm_disconnect normal ABORTED arm_disconnect
run_case driver_restart normal ABORTED driver_restart
run_case chassis_disconnect normal ABORTED chassis_disconnect 12
run_case chassis_disconnect_return normal ABORTED chassis_disconnect_return
run_case chassis_bridge_stop normal ABORTED chassis_bridge_stop

echo "P7_E2E_SCENARIOS=PASS results=$RESULTS cases=$case_index"
echo "P7_E2E_RUN_ID=$RUN_ID"
