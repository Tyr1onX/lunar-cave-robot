#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/../.." && pwd)"

run_case() {
  local scenario="$1"
  local expected="$2"
  local case_log_dir="${P6_RESULTS_DIR:-/tmp/p6-autonomous-mission-scenarios}/$scenario"
  echo "Running P6 scenario: $scenario (expected $expected)"
  P6_SCENARIO="$scenario" P6_EXPECTED_STATE="$expected" \
    P6_LOG_DIR="$case_log_dir" P6_EVENT_LOG_PATH="$case_log_dir/mission-events.jsonl" \
    P6_SCENARIO_TIMEOUT="${P6_SCENARIO_TIMEOUT:-360}" \
    P6_SKIP_BUILD=1 "$REPO_ROOT/tools/p6_offline/run.sh"
}

if [[ "${P6_SKIP_BUILD:-0}" != "1" ]]; then
  P6_SKIP_BUILD=0 "$REPO_ROOT/tools/p6_offline/run.sh" --build-only
fi

run_case normal SUCCEEDED
run_case nav_failure ABORTED
run_case target_loss ABORTED
run_case grasp_failure ABORTED
run_case communication_loss ABORTED
run_case duplicate_trigger SUCCEEDED
