#!/usr/bin/env bash
set -euo pipefail

repo_root="${LUNAR_CONSOLE_REPO:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)}"
ros_setup="/opt/ros/humble/setup.bash"
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

export LUNAR_CONSOLE_REPO="$repo_root"
export LUNAR_CONSOLE_TEST_BIND=127.0.0.1
export ROS_DOMAIN_ID=187
export ROS_LOCALHOST_ONLY=1
model_config="$HOME/.config/lunar-console/yolo-model-dir"
if [[ -f "$model_config" ]]; then
  IFS= read -r model_path < "$model_config" || true
  if [[ -n "${model_path:-}" ]]; then
    export LUNAR_CONSOLE_YOLO_MODEL_DIR="$model_path"
  fi
fi
exec python3 -m console_agent.module_tests
