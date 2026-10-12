#!/usr/bin/env bash
set -uo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/lunar-cave-robot"
timestamp=$(date -u +%Y-%m-%dT%H:%M:%SZ)
run_id=$(date -u +%Y%m%dT%H%M%SZ)
results_dir="$state_dir/runs/p7-$run_id"
mkdir -p "$state_dir"

sha=$(git -C "$repo_root" rev-parse HEAD 2>/dev/null || printf unknown)
branch=$(git -C "$repo_root" branch --show-current 2>/dev/null || printf detached)
echo "Running isolated P7/P8 PTY scenarios; results: $results_dir"
P7_RESULTS_DIR="$results_dir" P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator \
  "$repo_root/tools/p7_offline/validate_scenarios.sh"
result=$?
if [[ $result -eq 0 ]]; then outcome=PASS; else outcome=FAIL; fi
temporary_file=$(mktemp "$state_dir/.latest-test.XXXXXX") || exit 2
{
  printf 'timestamp: %s\n' "$timestamp"
  printf 'result: %s\n' "$outcome"
  printf 'sha: %s\n' "$sha"
  printf 'branch: %s\n' "$branch"
  printf 'test: p7_offline_validate_scenarios\n'
  printf 'results: %s\n' "$results_dir"
} > "$temporary_file"
mv -f "$temporary_file" "$state_dir/latest-test.yaml"
exit "$result"
