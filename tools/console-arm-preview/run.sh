#!/usr/bin/env bash
set -eo pipefail

HERE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
ROOT=$(cd -- "$HERE/../.." && pwd)
ROS_SETUP=${ROS_SETUP:-/opt/ros/humble/setup.bash}

if [[ ! -r "$ROS_SETUP" ]]; then
  echo "ROS 2 Humble setup missing; refusing to start arm preview" >&2
  exit 2
fi
source "$ROS_SETUP"
set -u

# Optional ament overlay, useful for temporary/offline package extraction.
if [[ -n "${FOXGLOVE_OVERLAY:-}" ]]; then
  if [[ ! -r "$FOXGLOVE_OVERLAY/local_setup.bash" ]]; then
    echo "FOXGLOVE_OVERLAY has no local_setup.bash" >&2
    exit 2
  fi
  source "$FOXGLOVE_OVERLAY/local_setup.bash"
fi

# This is a separate, loopback-only DDS domain. No robot-control overlay is sourced.
export ROS_DOMAIN_ID=218
export ROS_LOCALHOST_ONLY=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

for required in colcon ros2; do
  command -v "$required" >/dev/null || { echo "$required is required" >&2; exit 2; }
done

scratch=$(mktemp -d "${TMPDIR:-/tmp}/lunar-arm-preview.XXXXXX")
trap 'rm -rf "$scratch"' EXIT INT TERM

# Build only the existing description package to a disposable prefix so Foxglove
# can resolve its package:// STL URLs. No robot model files are generated here.
colcon --log-base "$scratch/log" build \
  --build-base "$scratch/build" \
  --install-base "$scratch/install" \
  --base-paths "$ROOT/tzb_ws/src/tzb_description" \
  --packages-select tzb_description --event-handlers console_direct+
set +u
source "$scratch/install/setup.bash"
set -u

for package in robot_state_publisher foxglove_bridge; do
  ros2 pkg prefix "$package" >/dev/null 2>&1 || {
    echo "ROS package $package is required; install it in the ROS 2 Humble environment" >&2
    exit 2
  }
done

ros2 launch "$HERE/launch/arm_preview.launch.py"
