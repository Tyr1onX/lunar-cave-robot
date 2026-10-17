#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../../.." && pwd)
build_dir=${1:-"${TMPDIR:-/tmp}/lunar-cave-chassis-host-tests"}
mkdir -p "$build_dir"

cc=${CC:-cc}
compile_status=0
"$cc" -std=c11 -Wall -Wextra -Werror -pedantic \
  -I"$repo_root/firmware/chassis/tests/host/stubs" \
  -I"$repo_root/firmware/chassis/User/APP" \
  "$repo_root/firmware/chassis/tests/host/test_chassis_protocol.c" \
  "$repo_root/firmware/chassis/User/APP/Chaiss_Communication.c" \
  "$repo_root/firmware/chassis/User/APP/chassis_command.c" \
  -lm -o "$build_dir/test_chassis_protocol" || compile_status=$?

run_status=$compile_status
if [[ $compile_status -eq 0 ]]; then
  "$build_dir/test_chassis_protocol" || run_status=$?
fi

if [[ $run_status -eq 0 ]]; then
  "$cc" -std=c11 -Wall -Wextra -Werror -pedantic \
    -I"$repo_root/firmware/chassis/User/Bsp" \
    -I"$repo_root/firmware/chassis/User/APP" \
    "$repo_root/firmware/chassis/tests/host/test_chassis_safety.c" \
    "$repo_root/firmware/chassis/User/APP/chassis_safety.c" \
    -o "$build_dir/test_chassis_safety" || run_status=$?
  if [[ $run_status -eq 0 ]]; then
    "$build_dir/test_chassis_safety" || run_status=$?
  fi
fi

if [[ $run_status -eq 0 ]]; then outcome=PASS; else outcome=FAIL; fi
"$repo_root/tools/record_local_status.sh" firmware-chassis-protocol-test "$outcome" \
  chassis_protocol_parser protocol-parser-fault-injection "$build_dir"
exit "$run_status"
