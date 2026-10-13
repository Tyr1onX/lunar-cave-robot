#!/bin/bash
set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
app_root=$(cd -- "$script_dir/.." && pwd)
repo_root=$(cd -- "$app_root/../.." && pwd)
sdk_path=${SDKROOT:-$(xcrun --sdk macosx --show-sdk-path)}
swiftc_path=$(xcrun --find swiftc)

"$swiftc_path" \
  -typecheck \
  -sdk "$sdk_path" \
  -target arm64-apple-macosx14.0 \
  -swift-version 5 \
  "$app_root"/Sources/LunarRobotConsole/*.swift
PYTHONPATH="$repo_root/tools/console-agent${PYTHONPATH:+:$PYTHONPATH}" \
  python3 -m unittest discover -s "$repo_root/tools/console-agent/tests" -v
