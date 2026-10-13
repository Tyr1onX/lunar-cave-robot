#!/usr/bin/env bash
set -uo pipefail
export GIT_OPTIONAL_LOCKS=0

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
remote_url=$(git -C "$repo_root" remote get-url origin 2>/dev/null || true)
github_sha=""
if [[ -n "$remote_url" ]]; then
  github_sha=$(git ls-remote "$remote_url" refs/heads/main 2>/dev/null | awk 'NR == 1 { print $1 }')
fi

printf 'GitHub main:\n'
if [[ -n "$github_sha" ]]; then
  printf '  SHA: %s\n  状态: 可读取\n' "$github_sha"
else
  printf '  SHA: unknown\n  状态: 无法读取远程 main\n'
fi

device_status() {
  local label=$1 host=$2 device_repo=$3 output
  printf '%s:\n' "$label"
  if ! output=$(ssh -o BatchMode=yes -o ConnectTimeout=8 -o ForwardAgent=no -o ClearAllForwardings=yes -o RequestTTY=no "$host" bash -s -- "$device_repo" "$github_sha" <<'REMOTE'
set -u
export GIT_OPTIONAL_LOCKS=0
device_repo=$1
if [[ "$device_repo" == __HOME__ ]]; then device_repo="$HOME/lunar-cave-robot"; fi
github_sha=$2
if [[ ! -d "$device_repo/.git" ]]; then
  printf '  SHA: unknown\n  分支: unknown\n  未提交修改: unknown\n  与 GitHub 差异: 统一工作目录不存在\n  P7/P8 全套: 未记录\n  P7 单场景: 未记录\n  ROS Nav2 工作区构建: 未记录\n  ROS 机械臂工作区构建: 未记录\n  底盘协议主机测试: 未记录\n  底盘 STM32 构建: 未记录\n  机械臂 STM32 构建: 未记录\n  当前部署版本: 未记录\n'
  exit 0
fi
if ! git -C "$device_repo" rev-parse --show-toplevel >/dev/null 2>&1; then
  printf '  SHA: unknown\n  分支: unknown\n  未提交修改: unknown\n  与 GitHub 差异: 路径不是 Git 工作区\n  P7/P8 全套: 未记录\n  P7 单场景: 未记录\n  ROS Nav2 工作区构建: 未记录\n  ROS 机械臂工作区构建: 未记录\n  底盘协议主机测试: 未记录\n  底盘 STM32 构建: 未记录\n  机械臂 STM32 构建: 未记录\n  当前部署版本: 未记录\n'
  exit 0
fi
head_sha=$(git -C "$device_repo" rev-parse HEAD)
branch=$(git -C "$device_repo" branch --show-current)
dirty_count=$(git -C "$device_repo" status --porcelain=v1 | wc -l | tr -d ' ')
if [[ "$dirty_count" == 0 ]]; then dirty='0（干净）'; else dirty="${dirty_count} 个文件"; fi
if [[ -n "$github_sha" ]] && git -C "$device_repo" cat-file -e "${github_sha}^{commit}" 2>/dev/null; then
  counts=$(git -C "$device_repo" rev-list --left-right --count "${github_sha}...${head_sha}" 2>/dev/null || true)
  read -r behind ahead <<<"$counts"
  if [[ ${behind:-x} == 0 && ${ahead:-x} == 0 ]]; then
    diff='与 GitHub main 同一提交'
  elif [[ ${ahead:-x} == 0 && ${behind:-x} =~ ^[0-9]+$ ]]; then
    diff="落后 ${behind} 个提交"
  elif [[ ${behind:-x} == 0 && ${ahead:-x} =~ ^[0-9]+$ ]]; then
    diff="领先 ${ahead} 个本地提交"
  else
    diff="分叉：远程独有 ${behind:-?} 个，本地独有 ${ahead:-?} 个提交"
  fi
else
  if [[ -n "$github_sha" ]]; then
    diff='本地 SHA 与 GitHub main 不同；需取回远程提交以统计提交数'
  else
    diff='GitHub main 不可读取，差异未知'
  fi
fi
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/lunar-cave-robot"
tests_dir="$state_dir/tests"
read_test_record() {
  local key=$1 file="$tests_dir/$1.yaml"
  if [[ ! -f "$file" ]]; then printf '未记录'; return; fi
  local result timestamp sha scope
  result=$(sed -n 's/^result: //p' "$file" | head -1)
  timestamp=$(sed -n 's/^timestamp: //p' "$file" | head -1)
  sha=$(sed -n 's/^sha: //p' "$file" | head -1)
  scope=$(sed -n 's/^scope: //p' "$file" | head -1)
  printf '%s %s @ %s (%s)' "${result:-unknown}" "${timestamp:-}" "${sha:-unknown}" "${scope:-范围未知}"
}
suite_info=$(read_test_record ros-offline-suite)
if [[ "$suite_info" == 未记录 && -f "$state_dir/latest-test.yaml" ]]; then
  test_time=$(sed -n 's/^timestamp: //p' "$state_dir/latest-test.yaml" | head -1)
  test_result=$(sed -n 's/^result: //p' "$state_dir/latest-test.yaml" | head -1)
  test_sha=$(sed -n 's/^sha: //p' "$state_dir/latest-test.yaml" | head -1)
  suite_info="${test_result:-unknown} ${test_time:-} @ ${test_sha:-unknown} (历史记录范围未知)"
fi
scenario_info=$(read_test_record p7-scenario)
nav_build_info=$(read_test_record ros-nav-build)
arm_build_info=$(read_test_record ros-arm-build)
chassis_protocol_test_info=$(read_test_record firmware-chassis-protocol-test)
chassis_build_info=$(read_test_record firmware-chassis-build)
arm_firmware_build_info=$(read_test_record firmware-arm-build)
if [[ -f "$state_dir/deployed-sha" ]]; then
  deployed_sha=$(head -1 "$state_dir/deployed-sha" | tr -cd '[:alnum:]')
  if [[ -n "$deployed_sha" ]]; then deployed_info=$deployed_sha; else deployed_info='未记录'; fi
else
  deployed_info='未记录'
fi
printf '  SHA: %s\n  分支: %s\n  未提交修改: %s\n  与 GitHub 差异: %s\n  P7/P8 全套: %s\n  P7 单场景: %s\n  ROS Nav2 工作区构建: %s\n  ROS 机械臂工作区构建: %s\n  底盘协议主机测试: %s\n  底盘 STM32 构建: %s\n  机械臂 STM32 构建: %s\n  当前部署版本: %s\n' \
  "$head_sha" "${branch:-detached}" "$dirty" "$diff" "$suite_info" "$scenario_info" \
  "$nav_build_info" "$arm_build_info" "$chassis_protocol_test_info" "$chassis_build_info" \
  "$arm_firmware_build_info" "$deployed_info"
REMOTE
  ); then
    printf '  SHA: unknown\n  分支: unknown\n  未提交修改: unknown\n  与 GitHub 差异: SSH 状态查询失败\n  P7/P8 全套: 未知\n  P7 单场景: 未知\n  ROS Nav2 工作区构建: 未知\n  ROS 机械臂工作区构建: 未知\n  底盘协议主机测试: 未知\n  底盘 STM32 构建: 未知\n  机械臂 STM32 构建: 未知\n  当前部署版本: 未知\n'
    return
  fi
  local device_sha counts behind ahead diff
  device_sha=$(sed -n 's/^  SHA: //p' <<<"$output" | head -1)
  if [[ "$device_sha" =~ ^[0-9a-f]{40}$ ]] \
    && git -C "$repo_root" cat-file -e "${github_sha}^{commit}" 2>/dev/null \
    && git -C "$repo_root" cat-file -e "${device_sha}^{commit}" 2>/dev/null; then
    counts=$(git -C "$repo_root" rev-list --left-right --count "${github_sha}...${device_sha}" 2>/dev/null || true)
    read -r behind ahead <<<"$counts"
    if [[ ${behind:-x} == 0 && ${ahead:-x} == 0 ]]; then
      diff='与 GitHub main 同一提交'
    elif [[ ${ahead:-x} == 0 && ${behind:-x} =~ ^[0-9]+$ ]]; then
      diff="落后 ${behind} 个提交"
    elif [[ ${behind:-x} == 0 && ${ahead:-x} =~ ^[0-9]+$ ]]; then
      diff="领先 ${ahead} 个本地提交"
    elif [[ ${behind:-x} =~ ^[0-9]+$ && ${ahead:-x} =~ ^[0-9]+$ ]]; then
      diff="分叉：远程独有 ${behind} 个，本地独有 ${ahead} 个提交"
    else
      diff='提交差异未知'
    fi
    output=$(sed "s/本地 SHA 与 GitHub main 不同；需取回远程提交以统计提交数/${diff}/" <<<"$output")
  fi
  printf '%s\n' "$output"
}

device_status ThinkBook "${THINKBOOK_ALIAS:-thinkbook}" "${THINKBOOK_REPO:-__HOME__}"
device_status NUC "${NUC_ALIAS:-nuc}" "${NUC_REPO:-__HOME__}"
