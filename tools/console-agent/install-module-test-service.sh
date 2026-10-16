#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git -C "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)" rev-parse --show-toplevel)"
service_dir="$HOME/.config/systemd/user"
service_file="$service_dir/lunar-console-module-tests.service"
required_repo=$(realpath -m -- "$HOME/lunar-robot/code")
resolved_repo=$(realpath -m -- "$repo_root")
if [[ "$resolved_repo" != "$required_repo" ]]; then
  echo "Console 模块测试服务只允许安装到 $required_repo；拒绝 $resolved_repo" >&2
  exit 2
fi
mkdir -p "$service_dir"

python3 - "$service_file" "$repo_root" <<'PY'
from pathlib import Path
import os
import sys

def quoted(value: str) -> str:
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'

service_file, repo_root = Path(sys.argv[1]), sys.argv[2]
agent_dir = f"{repo_root}/tools/console-agent"
repo_path = Path(repo_root)
if repo_path.name == "code":
    project_root = repo_path.parent
    state_home = Path(os.environ.get("LUNAR_CONSOLE_STATE_HOME", project_root / "tests" / "state")).expanduser()
    field_lab_root = Path(os.environ.get("LUNAR_CONSOLE_FIELD_LAB_ROOT", project_root / "lab" / "vision")).expanduser()
else:
    state_home = Path(os.environ.get("LUNAR_CONSOLE_STATE_HOME", Path.home() / ".local" / "state")).expanduser()
    field_lab_root = Path(os.environ.get("LUNAR_CONSOLE_FIELD_LAB_ROOT", Path.home() / "lunar-field-lab-p9i")).expanduser()
body = "\n".join([
    "[Unit]",
    "Description=Lunar Robot Console restricted module test runner",
    "After=network.target",
    "",
    "[Service]",
    "Type=simple",
    f"WorkingDirectory={agent_dir}",
    f"Environment=LUNAR_CONSOLE_REPO={quoted(repo_root)}",
    f"Environment=XDG_STATE_HOME={quoted(str(state_home))}",
    f"Environment=LUNAR_CONSOLE_FIELD_LAB_ROOT={quoted(str(field_lab_root))}",
    "Environment=LUNAR_CONSOLE_TEST_BIND=127.0.0.1",
    "Environment=ROS_DOMAIN_ID=187",
    "Environment=ROS_LOCALHOST_ONLY=1",
    "Environment=P6_BUILD_JOBS=2",
    "Environment=P7_BUILD_JOBS=2",
    "Environment=CMAKE_BUILD_PARALLEL_LEVEL=2",
    "Environment=MAKEFLAGS=-j2",
    f"ExecStart=/bin/bash {quoted(f'{agent_dir}/run-module-test-service.sh')}",
    "Restart=on-failure",
    "RestartSec=3",
    "TimeoutStopSec=20",
    # Let the runner terminate its own process groups before systemd applies
    # the final cgroup kill when this service is restarted or stopped.
    "KillMode=mixed",
    "CPUWeight=20",
    "CPUQuota=300%",
    "MemoryHigh=8G",
    "MemoryMax=10G",
    "TasksMax=1024",
    "Nice=10",
    "NoNewPrivileges=true",
    "PrivateTmp=true",
    "ProtectSystem=full",
    "LockPersonality=true",
    # Fast DDS uses getifaddrs()/NETLINK to enumerate local interfaces even
    # when ROS_LOCALHOST_ONLY is set. Keep packet sockets unavailable.
    "RestrictAddressFamilies=AF_UNIX AF_INET AF_INET6 AF_NETLINK",
    "",
    "[Install]",
    "WantedBy=default.target",
    "",
])
service_file.write_text(body, encoding="utf-8")
PY

systemctl --user daemon-reload
systemctl --user enable lunar-console-module-tests.service
if systemctl --user is-active --quiet lunar-console-module-tests.service; then
  systemctl --user restart lunar-console-module-tests.service
else
  systemctl --user start lunar-console-module-tests.service
fi
printf 'Installed and started %s\n' "$service_file"
printf 'Optional YOLO model path file: %s/.config/lunar-console/yolo-model-dir\n' "$HOME"
