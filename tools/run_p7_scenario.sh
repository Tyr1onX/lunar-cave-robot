#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 1 ]]; then
  echo "usage: $0 <scenario-name>" >&2
  exit 2
fi
scenario=$1
case "$scenario" in
  normal|nav_failure|target_loss|grasp_failure|communication_loss|duplicate_trigger|arm_feedback_stop|arm_disconnect|driver_restart|chassis_disconnect|chassis_disconnect_return|chassis_bridge_stop) ;;
  *) echo "unknown P7 scenario: $scenario" >&2; exit 2 ;;
esac

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
state_home=${XDG_STATE_HOME:-}
if [[ -z "$state_home" ]]; then
  if [[ "$(basename -- "$repo_root")" == code ]]; then
    state_home="$(dirname -- "$repo_root")/tests/state"
  else
    state_home="$HOME/.local/state"
  fi
fi
state_dir="$state_home/lunar-cave-robot"
run_id=$(date -u +%Y%m%dT%H%M%SZ)
results_dir="${P7_RESULTS_DIR:-$state_dir/runs/p7-scenario-$scenario-$run_id}"
mkdir -p "$results_dir"

set +e
P7_CASE_FILTER="$scenario" P7_RESULTS_DIR="$results_dir" \
  P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator \
  "$repo_root/tools/p7_offline/validate_scenarios.sh"
status=$?
set -e
if [[ $status -eq 0 ]]; then outcome=PASS; else outcome=FAIL; fi
"$repo_root/tools/record_local_status.sh" p7-scenario "$outcome" \
  p7_offline_validate_scenarios "scenario=$scenario" "$results_dir"
exit "$status"
