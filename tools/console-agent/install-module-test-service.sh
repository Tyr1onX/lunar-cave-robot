#!/usr/bin/env bash
set -euo pipefail

repo_root="$(git -C "$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)" rev-parse --show-toplevel)"
service_dir="$HOME/.config/systemd/user"
service_file="$service_dir/lunar-console-module-tests.service"
mkdir -p "$service_dir"

python3 - "$service_file" "$repo_root" <<'PY'
from pathlib import Path
import sys

def quoted(value: str) -> str:
    return '"' + value.replace('\\', '\\\\').replace('"', '\\"').replace('%', '%%') + '"'

service_file, repo_root = Path(sys.argv[1]), sys.argv[2]
agent_dir = f"{repo_root}/tools/console-agent"
body = "\n".join([
    "[Unit]",
    "Description=Lunar Robot Console restricted module test runner",
    "After=network.target",
    "",
    "[Service]",
    "Type=simple",
    f"WorkingDirectory={agent_dir}",
    f"Environment=LUNAR_CONSOLE_REPO={quoted(repo_root)}",
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
    "KillMode=control-group",
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
systemctl --user enable --now lunar-console-module-tests.service
printf 'Installed and started %s\n' "$service_file"
printf 'Optional YOLO model path file: %s/.config/lunar-console/yolo-model-dir\n' "$HOME"
