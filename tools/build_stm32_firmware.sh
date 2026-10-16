#!/usr/bin/env bash
set -uo pipefail

if [[ $# -ne 1 || ( "$1" != arm && "$1" != chassis ) ]]; then
  echo "usage: $0 <arm|chassis>" >&2
  exit 2
fi

module=$1
repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
if [[ "$(basename -- "$repo_root")" == code ]]; then
  project_root=$(dirname -- "$repo_root")
  default_build_root="$project_root/tests/cache/lunar-cave-robot/stm32"
  build_root=${STM32_BUILD_ROOT:-$default_build_root}
  resolved_project_root=$(realpath -m -- "$project_root")
  resolved_build_root=$(realpath -m -- "$build_root")
  case "$resolved_build_root" in
    "$resolved_project_root"/tests/*) build_root=$resolved_build_root ;;
    *)
      echo "拒绝在 lunar-robot/code 外构建新版 STM32 固件：$build_root" >&2
      exit 2
      ;;
  esac
else
  build_root=${STM32_BUILD_ROOT:-"${XDG_CACHE_HOME:-$HOME/.cache}/lunar-cave-robot/stm32"}
fi
build_dir="$build_root/$module"
log_file="$build_root/logs/$module-build.log"
key="firmware-$module-build"
generator=${STM32_CMAKE_GENERATOR:-"Unix Makefiles"}

if [[ -n ${STM32CUBECLT_PATH:-} ]]; then
  export PATH="$STM32CUBECLT_PATH/bin:$PATH"
fi

mkdir -p "$build_root/logs"
cmake_status=0
cmake -S "$repo_root/firmware/$module" -B "$build_dir" -G "$generator" \
  -DCMAKE_TOOLCHAIN_FILE="$repo_root/firmware/$module/cmake/gcc-arm-none-eabi.cmake" \
  -DCMAKE_BUILD_TYPE=Debug >"$log_file" 2>&1 || cmake_status=$?

build_status=0
if [[ $cmake_status -eq 0 ]]; then
  cmake --build "$build_dir" --parallel "${STM32_BUILD_JOBS:-4}" >>"$log_file" 2>&1 || build_status=$?
else
  build_status=$cmake_status
fi

if [[ $build_status -eq 0 ]]; then outcome=PASS; else outcome=FAIL; fi
"$repo_root/tools/record_local_status.sh" "$key" "$outcome" \
  cmake_build "firmware/$module" "$log_file"
cat "$log_file"
if [[ $build_status -ne 0 ]]; then exit "$build_status"; fi
printf 'ELF: %s\n' "$build_dir"
