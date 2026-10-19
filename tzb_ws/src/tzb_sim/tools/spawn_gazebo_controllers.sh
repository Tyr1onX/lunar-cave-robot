#!/usr/bin/env bash
set -euo pipefail
for controller in joint_state_broadcaster arm_position_controller gripper_effort_controller; do
  ros2 run controller_manager spawner "$controller" \
    --controller-manager /controller_manager \
    --controller-manager-timeout 120
done
