#!/usr/bin/env bash
set -euo pipefail

repo_root="${LUNAR_CONSOLE_REPO:-$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)}"
if [[ "$#" -ne 2 || "$1" != "--device" ]]; then
  echo "usage: run_ros_readonly_check.sh --device {camera|mid360|chassis|arm}" >&2
  exit 64
fi
device="$2"
case "$device" in
  camera|mid360|chassis|arm) ;;
  *) echo "unsupported read-only device profile" >&2; exit 64 ;;
esac

set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
exec python3 "$repo_root/tools/module-tests/scripts/observe_ros_device.py" --device "$device"
