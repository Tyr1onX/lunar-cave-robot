#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"
P6_DOMAIN="$(printenv P4F_P6_DOMAIN_ID || true)"
P4E_DOMAIN="$(printenv P4F_P4E_DOMAIN_ID || true)"
P6_JOBS="$(printenv P6_BUILD_JOBS || true)"
SCENARIO_TIMEOUT="$(printenv P6_SCENARIO_TIMEOUT || true)"
RESULTS="$(printenv P4F_RESULTS_DIR || true)"
[[ -n "$P6_DOMAIN" ]] || P6_DOMAIN=98
[[ -n "$P4E_DOMAIN" ]] || P4E_DOMAIN=99
[[ -n "$P6_JOBS" ]] || P6_JOBS=4
[[ -n "$SCENARIO_TIMEOUT" ]] || SCENARIO_TIMEOUT=360
[[ -n "$RESULTS" ]] || RESULTS=/tmp/p4f-ros2-acceptance

for value in "$P6_DOMAIN" "$P4E_DOMAIN"; do
  if [[ ! "$value" =~ ^[0-9]+$ ]] || (( value < 2 || value > 101 )); then
    echo "P4F ROS domain IDs must be integers in 2..101" >&2
    exit 2
  fi
done
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required at /opt/ros/humble/setup.bash" >&2
  exit 2
fi

mkdir -p "$RESULTS"
export ROS_LOCALHOST_ONLY=1
export ROS2CLI_DISABLE_DAEMON=1
set +u
source /opt/ros/humble/setup.bash
set -u

echo "Building the independent P6 Nav2 and arm workspaces..."
P6_ROS_DOMAIN_ID="$P6_DOMAIN" P6_BUILD_JOBS="$P6_JOBS" \
  "$REPO_ROOT/tools/p6_offline/run.sh" --build-only

echo "Building the P4e PTY driver into its separate install prefix..."
(
  cd "$REPO_ROOT/tzb_ws"
  colcon --log-base log/p4f build --base-paths src --packages-up-to rm_serial_driver \
    --build-base build/p4f --install-base install/p4f --parallel-workers "$P6_JOBS"
)

echo "Running P4e driver fault injection through a generated PTY only..."
(
  set +u
  source "$REPO_ROOT/tzb_ws/install/p4f/setup.bash"
  set -u
  export ROS_DOMAIN_ID="$P4E_DOMAIN"
  /usr/bin/python3 -u "$REPO_ROOT/tools/p4e_offline/test_arm_host_safety_pty.py" \
    2>&1 | tee "$RESULTS/p4e-pty.log"
)

echo "Running P6 mission logic and static contract tests..."
(
  cd "$REPO_ROOT"
  ./tools/p6_offline/run_unit_tests.sh 2>&1 | tee "$RESULTS/p6-unit-tests.log"
)

echo "Running a real planner-result/cancellation race before the six mission scenarios..."
RACE_DIR="$RESULTS/cancel-plan-race"
mkdir -p "$RACE_DIR"
(
  cd "$REPO_ROOT"
  export ROS_DOMAIN_ID="$P6_DOMAIN"
  P6_ROS_DOMAIN_ID="$P6_DOMAIN" P6_SCENARIO=grasp_failure P6_EXPECTED_STATE=ABORTED \
    P6_TEST_CANCEL_DELAY_SEC=0.36 P6_SKIP_BUILD=1 P6_SCENARIO_TIMEOUT=240 \
    P6_LOG_DIR="$RACE_DIR" P6_EVENT_LOG_PATH="$RACE_DIR/mission-events.jsonl" \
    ./tools/p6_offline/run.sh 2>&1 | tee "$RACE_DIR/run.log"
  /usr/bin/python3 ./tools/p4f_offline/verify_cancel_race.py \
    "$RACE_DIR/arm.log" "$RACE_DIR/mission-events.jsonl" "$RACE_DIR/scenario-monitor.log" \
    2>&1 | tee "$RACE_DIR/assertions.log"
)

echo "Running all six P6 scenarios on a localhost-only ROS domain..."
(
  cd "$REPO_ROOT"
  export ROS_DOMAIN_ID="$P6_DOMAIN"
  P6_ROS_DOMAIN_ID="$P6_DOMAIN" P6_SKIP_BUILD=1 \
    P6_RESULTS_DIR="$RESULTS/p6-scenarios" \
    P6_SCENARIO_TIMEOUT="$SCENARIO_TIMEOUT" \
    ./tools/p6_offline/validate_scenarios.sh 2>&1 | tee "$RESULTS/p6-scenarios.log"
)

echo "P4f ROS 2 offline acceptance completed. Results: $RESULTS"
