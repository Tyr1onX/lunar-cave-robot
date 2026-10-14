#!/usr/bin/env bash
set -eo pipefail

repo_root="${LUNAR_CONSOLE_REPO:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)}"
field_lab="${LUNAR_CONSOLE_FIELD_LAB_ROOT:-$HOME/lunar-field-lab-p9i}"
model_dir="${LUNAR_CONSOLE_YOLO_MODEL_DIR:-}"
if [[ -z "$model_dir" && -f "$HOME/.config/lunar-console/yolo-model-dir" ]]; then
  IFS= read -r model_dir < "$HOME/.config/lunar-console/yolo-model-dir" || true
fi
if [[ -z "$model_dir" ]]; then
  model_dir="$field_lab/models/blue_block"
fi

set +u
source /opt/ros/humble/setup.bash
source "$field_lab/build/tzb/install/setup.bash"
set -u

export ROS_DOMAIN_ID=187
export ROS_LOCALHOST_ONLY=1
export YOLO_AUTOINSTALL=false
exec "$field_lab/venv/bin/python" \
  "$repo_root/tools/perception/offline_yolo_ros_smoke.py" \
  --model "$model_dir" \
  --timeout 90
