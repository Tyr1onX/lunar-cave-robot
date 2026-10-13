# Lunar Robot Console for macOS

Native SwiftUI read-only console for Apple Silicon. The first release exposes only Overview, System Diagnostics, and History. It does not install ROS 2 on the Mac and does not contain motion, service-call, deployment, or shell-command controls.

## Connection contract

The app starts one fixed SSH local forward using the existing `nuc` SSH alias:

```text
Mac 127.0.0.1:18765 → NUC 127.0.0.1:8765
```

It invokes `/usr/bin/ssh -N -T` with fixed arguments. No remote command is sent. The NUC agent must already be running. The app polls only `GET /health`, `GET /v1/snapshot`, and `GET /v1/logs`; a failed read clears the current snapshot so stale values are not presented as live.

GitHub `main` is read separately by a fixed `git ls-remote` request over the Mac's existing SSH authentication. NUC worktree `HEAD` and the deployment owner's `deployed-sha` record are separate fields. Missing deployment evidence remains “未记录”.

## Build on Mac

Requirements: Apple Silicon Mac, Swift 5.9 or newer, macOS 14 or newer, and a working `nuc` SSH alias with host-key verification configured. ROS 2 is not required on the Mac.

```sh
cd tools/mac-console
./Scripts/validate.sh
./Scripts/build-app.sh
open '.build/Lunar Robot Console.app'
```

On this development Mac, use the installed 26.5 SDK explicitly because the standalone Swift compiler is older than the latest SDK:

```sh
SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk ./Scripts/build-app.sh
```

The Swift package test target can be run with `swift test` on a matching SwiftPM toolchain. This Mac's current SwiftPM binary has a system-library mismatch, so local verification uses direct `swiftc` compilation plus the in-app live NUC contract check.

`Scripts/build-app.sh` creates a local unsigned `.app` bundle. For ad-hoc local development it applies a local ad-hoc signature; it is not a notarized distribution build. On hosts where the selected Swift toolchain and newest installed SDK differ, set `SDKROOT` to a compatible installed macOS SDK before running the scripts. `validate.sh` directly type-checks the native app and runs the read-only agent's HTTP contract tests without ROS 2. A matching Swift Package Manager toolchain can additionally run `swift test`.

## Screens and state meaning

- **总览:** live CPU, memory, root disk, available thermal sensors, three version sources, ROS graph counts, safety evidence and recent ROS log lines.
- **系统诊断:** runtime-discovered ROS nodes/topics and the origin/status of each system metric.
- **历史记录:** existing YAML records under the NUC user's `$XDG_STATE_HOME/lunar-cave-robot/tests/` (default `~/.local/state/lunar-cave-robot/tests/`). The app never creates or edits records.

“未知” is used when a source is absent, unreadable, or unverified. ROS graph presence is not reported as device health; physical E-stop status remains unknown without a verified input. `HOST_WRITE_OK` is not used as controller feedback.
