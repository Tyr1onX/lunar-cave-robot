#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)
build_dir=${1:-"${TMPDIR:-/tmp}/lunar-cave-arm-safety-host-tests"}
mkdir -p "$build_dir"

cc=${CC:-cc}
"$cc" -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$repo_root/firmware/arm/others/inc/app" \
  "$repo_root/firmware/arm/tests/host/test_arm_safety_state.c" \
  "$repo_root/firmware/arm/others/src/app/arm_safety_state.c" \
  -o "$build_dir/test_arm_safety_state"
"$build_dir/test_arm_safety_state"
"$repo_root/tools/record_local_status.sh" firmware-arm-safety-state-test PASS \
  arm_safety_state host-compiled-firmware-policy "$build_dir"
