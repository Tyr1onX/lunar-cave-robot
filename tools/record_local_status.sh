#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 4 || $# -gt 5 ]]; then
  echo "usage: $0 <status-key> <PASS|FAIL|NOT_RUN> <test-name> <scope> [artifact]" >&2
  exit 2
fi

key=$1
result=$2
test_name=$3
scope=$4
artifact=${5:-}
if [[ ! "$key" =~ ^[a-z0-9][a-z0-9_-]*$ ]]; then
  echo "invalid status key: $key" >&2
  exit 2
fi
case "$result" in PASS|FAIL|NOT_RUN) ;; *) echo "invalid result: $result" >&2; exit 2 ;; esac

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
state_home=${XDG_STATE_HOME:-}
if [[ -z "$state_home" ]]; then
  if [[ "$(basename -- "$repo_root")" == code ]]; then
    state_home="$(dirname -- "$repo_root")/tests/state"
  else
    state_home="$HOME/.local/state"
  fi
fi
state_dir="$state_home/lunar-cave-robot/tests"
timestamp=$(date -u +%Y-%m-%dT%H:%M:%SZ)
sha=$(git -C "$repo_root" rev-parse HEAD 2>/dev/null || printf unknown)
branch=$(git -C "$repo_root" branch --show-current 2>/dev/null || printf detached)
mkdir -p "$state_dir"
temporary_file=$(mktemp "$state_dir/.${key}.XXXXXX")
{
  printf 'timestamp: %s\n' "$timestamp"
  printf 'result: %s\n' "$result"
  printf 'sha: %s\n' "$sha"
  printf 'branch: %s\n' "$branch"
  printf 'test: %s\n' "$test_name"
  printf 'scope: %s\n' "$scope"
  if [[ -n "$artifact" ]]; then printf 'artifact: %s\n' "$artifact"; fi
} > "$temporary_file"
mv -f "$temporary_file" "$state_dir/$key.yaml"
