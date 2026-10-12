#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
jobs=${ROS_BUILD_JOBS:-4}
if [[ ! "$jobs" =~ ^[1-9][0-9]*$ ]]; then
  echo "ROS_BUILD_JOBS must be a positive integer" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required at /opt/ros/humble/setup.bash" >&2
  exit 2
fi

build_one() {
  local key=$1 workspace=$2 packages=$3 skip=${4:-}
  local package_list
  read -r -a package_list <<< "$packages"
  local command_status=0
  echo "Building $workspace ($packages) in an isolated install prefix..."
  set +e
  (
    set +u
    source /opt/ros/humble/setup.bash
    set -u
    cd "$repo_root/$workspace"
    if [[ -n "$skip" ]]; then
      colcon build --base-paths src --packages-up-to "${package_list[@]}" --packages-skip "$skip" \
        --build-base build/p9b --install-base install/p9b --parallel-workers "$jobs"
    else
      colcon build --base-paths src --packages-up-to "${package_list[@]}" \
        --build-base build/p9b --install-base install/p9b --parallel-workers "$jobs"
    fi
  )
  command_status=$?
  set -e
  if [[ $command_status -eq 0 ]]; then outcome=PASS; else outcome=FAIL; fi
  "$repo_root/tools/record_local_status.sh" "$key" "$outcome" colcon_build \
    "${workspace}_${packages// /_}" "$repo_root/$workspace/log/latest_build"
  return "$command_status"
}

overall_status=0
build_one ros-nav-build Nav2_rea_ws "p6_autonomous_mission chassis_serial_bridge" || overall_status=1
build_one ros-arm-build tzb_ws "lite_motion_planner rm_serial_driver" orbbec_camera || overall_status=1
exit "$overall_status"
