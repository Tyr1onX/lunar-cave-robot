#!/usr/bin/env bash
set -euo pipefail

repo_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
remote_url=$(git -C "$repo_root" remote get-url origin)
expected_sha=$(git ls-remote "$remote_url" refs/heads/main | awk 'NR == 1 { print $1 }')
if [[ ! "$expected_sha" =~ ^[0-9a-f]{40}$ ]]; then
  echo '无法读取 GitHub main SHA；没有同步任何设备。' >&2
  exit 2
fi

sync_device() {
  local label=$1 host=$2 device_repo=$3
  printf '%s: 检查并同步到 %s\n' "$label" "$expected_sha"
  ssh -o BatchMode=yes -o ConnectTimeout=8 "$host" bash -s -- "$remote_url" "$device_repo" "$expected_sha" <<'REMOTE'
set -euo pipefail
remote_url=$1
device_repo=$2
if [[ "$device_repo" == __HOME__ ]]; then device_repo="$HOME/lunar-cave-robot"; fi
expected_sha=$3

normalize_repo() {
  local value=$1
  value=${value#git@github.com:}
  value=${value#ssh://git@github.com/}
  value=${value#https://github.com/}
  value=${value#http://github.com/}
  value=${value%.git}
  printf '%s' "$value" | tr '[:upper:]' '[:lower:]'
}

if [[ -L "$device_repo" ]]; then
  echo "拒绝符号链接目标：$device_repo" >&2
  exit 10
fi
if [[ ! -e "$device_repo" ]]; then
  mkdir -p "$(dirname -- "$device_repo")"
  git clone --single-branch --branch main "$remote_url" "$device_repo"
fi
if [[ ! -d "$device_repo/.git" ]]; then
  echo "目标已存在但不是普通 Git 工作区；未改动：$device_repo" >&2
  exit 11
fi
if [[ "$(normalize_repo "$(git -C "$device_repo" config --get remote.origin.url || true)")" != 'tyr1onx/lunar-cave-robot' ]]; then
  echo "目标 origin 不是 Tyr1onX/lunar-cave-robot；未改动。" >&2
  exit 12
fi
if [[ "$(git -C "$device_repo" branch --show-current)" != main ]]; then
  echo "目标当前不在 main；未改动。" >&2
  exit 13
fi
if [[ -n "$(git -C "$device_repo" status --porcelain=v1)" ]]; then
  echo "目标含未提交或未跟踪修改；未覆盖任何内容。" >&2
  exit 14
fi

git -C "$device_repo" fetch origin main
remote_ref=$(git -C "$device_repo" rev-parse refs/remotes/origin/main)
if [[ "$remote_ref" != "$expected_sha" ]]; then
  echo "同步期间 GitHub main 已变化；重新查询后再运行。" >&2
  exit 15
fi
current_sha=$(git -C "$device_repo" rev-parse HEAD)
if [[ "$current_sha" != "$expected_sha" ]]; then
  if ! git -C "$device_repo" merge-base --is-ancestor HEAD refs/remotes/origin/main; then
    echo "本地 main 与远程分叉或含本地提交；拒绝覆盖。" >&2
    exit 16
  fi
  git -C "$device_repo" merge --ff-only refs/remotes/origin/main
fi
if [[ "$(git -C "$device_repo" rev-parse HEAD)" != "$expected_sha" ]] || [[ -n "$(git -C "$device_repo" status --porcelain=v1)" ]]; then
  echo '同步后 SHA 或干净状态校验失败。' >&2
  exit 17
fi
printf '同步完成：%s，main，工作区干净。\n' "$expected_sha"
REMOTE
}

sync_device ThinkBook "${THINKBOOK_ALIAS:-thinkbook}" "${THINKBOOK_REPO:-__HOME__}"
sync_device NUC "${NUC_ALIAS:-nuc}" "${NUC_REPO:-__HOME__}"
