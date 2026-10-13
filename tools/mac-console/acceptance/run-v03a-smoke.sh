#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK="${SDKROOT:-/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk}"
SOURCES=()
for f in "$ROOT"/Sources/LunarRobotConsole/*.swift; do
  [[ "$f" == *"/LunarRobotConsoleApp.swift" ]] || SOURCES+=("$f")
done
mkdir -p "$ROOT/.build"
"$(xcrun --find swiftc)" -sdk "$SDK" -target arm64-apple-macosx14.0 -swift-version 5 \
  -o "$ROOT/.build/v03a-smoke" "${SOURCES[@]}" "$ROOT/acceptance/v0.3a-simulation-smoke.swift"
exec "$ROOT/.build/v03a-smoke"
