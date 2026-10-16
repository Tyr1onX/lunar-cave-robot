#!/usr/bin/env bash
set -euo pipefail

repo_root="${LUNAR_CONSOLE_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
required_repo=$(realpath -m -- "$HOME/lunar-robot/code")
resolved_repo=$(realpath -m -- "$repo_root")
if [[ "$resolved_repo" != "$required_repo" ]]; then
  printf 'Refusing to run Console module tests outside %s (requested %s).\n' "$required_repo" "$resolved_repo" >&2
  exit 2
fi
ros_setup="/opt/ros/humble/setup.bash"
set +u
if [[ -f "$ros_setup" ]]; then
  if ! source "$ros_setup"; then
    printf 'ROS Humble setup could not be sourced; the environment check will report this.\n' >&2
  fi
fi
overlay="$repo_root/tzb_ws/install/offline/setup.bash"
if [[ -f "$overlay" ]]; then
  if ! source "$overlay"; then
    printf 'The offline ROS overlay could not be sourced; dependent tests will remain unavailable.\n' >&2
  fi
fi
set -u

export LUNAR_CONSOLE_REPO="$repo_root"
export LUNAR_CONSOLE_TEST_BIND=127.0.0.1
export ROS_DOMAIN_ID=187
export ROS_LOCALHOST_ONLY=1
if [[ "$(basename -- "$repo_root")" == code ]]; then
  project_root=$(dirname -- "$repo_root")
  export XDG_STATE_HOME="${XDG_STATE_HOME:-$project_root/tests/state}"
  export LUNAR_CONSOLE_FIELD_LAB_ROOT="${LUNAR_CONSOLE_FIELD_LAB_ROOT:-$project_root/lab/vision}"
fi
model_config="$HOME/.config/lunar-console/yolo-model-dir"
if [[ -z "${LUNAR_CONSOLE_YOLO_MODEL_DIR:-}" && -f "$model_config" ]]; then
  IFS= read -r model_path < "$model_config" || true
  if [[ -n "${model_path:-}" ]]; then
    export LUNAR_CONSOLE_YOLO_MODEL_DIR="$model_path"
  fi
fi
cd "$repo_root/tools/console-agent"
exec python3 -m console_agent.module_tests
