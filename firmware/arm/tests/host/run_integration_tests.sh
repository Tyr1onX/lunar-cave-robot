#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)
build_dir=${1:-"${TMPDIR:-/tmp}/lunar-cave-arm-integration-tests"}
mkdir -p "$build_dir"

cc=${CC:-cc}
cxx=${CXX:-c++}
stub_dir="$repo_root/firmware/arm/tests/host/stubs"
arm_inc="$repo_root/firmware/arm/others/inc"

common=(-Wall -Wextra -Werror -pedantic -I"$stub_dir" -I"$arm_inc/app" -I"$arm_inc/bsp" -I"$arm_inc/obj" -I"$arm_inc/modules" -I"$arm_inc/thread")

"$cc" -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$arm_inc/app" \
  -c "$repo_root/firmware/arm/others/src/app/arm_safety_state.c" \
  -o "$build_dir/arm_safety_state.o"

"$cxx" -std=c++17 "${common[@]}" \
  "$repo_root/firmware/arm/tests/host/test_arm_safety_integration.cpp" \
  "$repo_root/firmware/arm/others/src/app/Host.cpp" \
  "$repo_root/firmware/arm/others/src/app/arm_output_gate.cpp" \
  "$repo_root/firmware/arm/others/src/thread/task_error.cpp" \
  "$repo_root/firmware/arm/others/src/obj/motor_dm.cpp" \
  "$repo_root/firmware/arm/others/src/bsp/bsp_can.cpp" \
  "$repo_root/firmware/arm/others/src/callcallback/can_callback.cpp" \
  "$build_dir/arm_safety_state.o" \
  -o "$build_dir/test_arm_safety_integration"

"$build_dir/test_arm_safety_integration"

real_common=(-Wall -Wextra -Werror -pedantic -I"$arm_inc/obj" -I"$arm_inc/algorithm" -I"$arm_inc/app" -I"$arm_inc/bsp" -I"$arm_inc/modules" -I"$stub_dir")

"$cc" -DREMOTE_C_HOST_TEST -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$arm_inc/app" -I"$stub_dir" \
  -c "$repo_root/firmware/arm/others/src/app/Remote.c" \
  -o "$build_dir/remote_production.o"

"$cxx" -std=c++17 "${real_common[@]}" \
  "$repo_root/firmware/arm/tests/host/test_joint_output_gate.cpp" \
  "$repo_root/firmware/arm/others/src/obj/roboarm.cpp" \
  "$repo_root/firmware/arm/others/src/obj/motor_dm.cpp" \
  "$repo_root/firmware/arm/others/src/bsp/bsp_can.cpp" \
  "$repo_root/firmware/arm/others/src/app/arm_output_gate.cpp" \
  "$build_dir/remote_production.o" \
  -o "$build_dir/test_joint_output_gate"

"$build_dir/test_joint_output_gate"

"$cc" -DREMOTE_C_HOST_TEST -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$arm_inc/app" -I"$stub_dir" \
  "$repo_root/firmware/arm/tests/host/test_remote_mode_generation.c" \
  "$build_dir/remote_production.o" \
  -o "$build_dir/test_remote_mode_generation"

"$build_dir/test_remote_mode_generation"
