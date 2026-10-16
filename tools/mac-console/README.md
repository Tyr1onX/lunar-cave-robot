# Lunar Robot Console for macOS

Native SwiftUI console for Apple Silicon. The default screen is the read-only Test Cockpit; there is also a **separate SIMULATION-only** driving page, System Diagnostics, and History. Mac does not need ROS 2. Physical robot motion, arbitrary ROS Topic/service writes, device control, deployment and shell-command controls remain absent.

## Connection contract

The app starts a fixed **read-only production Agent** SSH local forward using the existing `nuc` SSH alias:

```text
Mac 127.0.0.1:18765 → NUC 127.0.0.1:8765
```

It invokes `/usr/bin/ssh -N -T` with fixed arguments. No remote command is sent. **Only on the SIMULATION page**, a second fixed SSH forward is used, independently of the production read-only Agent:

```text
Mac 127.0.0.1:18766 → NUC 127.0.0.1:8766 (isolated P6 simulator)
```

The **模块测试** page adds a third fixed forward:

```text
Mac 127.0.0.1:18769 → NUC 127.0.0.1:8769 (restricted module test runner)
```

The module test runner is a separate, standard-library Python service. It binds only to NUC loopback and accepts only a registered test ID, with no command or argument field. It reuses fixed ROS, FK, YOLO, P5, P6, P7 and read-only observer scripts. Install it once from the repository root on NUC with `bash tools/console-agent/install-module-test-service.sh`. The installer creates and starts a user `systemd` service, which starts on later user logins. P6/P7 are capped at 300% CPU, 8 GiB `MemoryHigh`, 10 GiB `MemoryMax`, and two build workers. On stop or restart, systemd first signals the runner; it terminates test process groups and persists them as `INTERRUPTED`, with cgroup cleanup as the timeout fallback. Reports and logs survive restart. It does not start the read-only Agent, any physical robot process, camera driver, base, or arm.

YOLO remains locked until `metadata.yaml`, `yoloe-26l-seg.xml`, and `yoloe-26l-seg.bin` exist in a local model directory and the NUC ROS/Python overlay has `yolo_node`, `rclpy`, OpenCV, Ultralytics, and NumPy. To unlock it, place the model directory path on one line in `~/.config/lunar-console/yolo-model-dir`, then restart `lunar-console-module-tests.service`. The actual offline test uses the existing synthetic blue-block ROS smoke test; it does not read a camera frame and does not establish scene accuracy.

Every run is automatically recorded under `$XDG_STATE_HOME/lunar-cave-robot/module-tests/` (on the migrated NUC, `~/lunar-robot/tests/state/lunar-cave-robot/module-tests/`) as JSON plus a text log. The report includes UTC start/end, NUC repository SHA and dirty state, YOLO model bundle SHA-256 when applicable, fixed input source and parameters, duration, result, captured output and P6/P7 resource snapshots. The app streams new log lines, can cancel the process group, browses recent reports, reloads a report after a connection/service interruption, and exports JSON to Mac. Per-run output is capped at 1 MiB; each test has a fixed timeout. Only allowlisted script IDs can run.

The simulator must be explicitly started on NUC as described in [`../console-sim/README.md`](../console-sim/README.md). It runs with ROS Domain 217 + localhost-only, and accepts only bounded simulated motion requests addressed to `/console_sim/cmd_vel`. Entering the GUI simulation page **does not start a robot or a ROS process**. The NUC agent must already be running. The app polls only `GET /health`, `GET /v1/snapshot`, and `GET /v1/logs`; a failed read clears the current snapshot so stale values are not presented as live.

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

The Swift package test target can be run with `swift test` on a matching SwiftPM toolchain. This Mac's current SwiftPM binary has a system-library mismatch, so local verification uses direct `swiftc` compilation plus the runner's loopback HTTP contract tests.

`Scripts/build-app.sh` creates a local unsigned `.app` bundle. For ad-hoc local development it applies a local ad-hoc signature; it is not a notarized distribution build. On hosts where the selected Swift toolchain and newest installed SDK differ, set `SDKROOT` to a compatible installed macOS SDK before running the scripts. `validate.sh` directly type-checks the native app and runs both NUC service HTTP contract suites without ROS 2. A matching Swift Package Manager toolchain can additionally run `swift test`.

## Screens and state meaning

- **测试驾驶舱:** RGB, registered depth, and YOLO annotated-image views backed by fixed NUC ROS Image topics. NUC receive FPS and freshness come from Agent callbacks; the GUI separately reports its own accepted image-update FPS (current polling ceiling about 2 FPS) and HTTP round trip. Neither value is a promise of screen hardware refresh rate or physical camera FPS. Synthetic acceptance frames are explicitly marked from their `lunar-v0.2a-synthetic/*` ROS frame IDs; all other unverified sources remain ROS stream observations, not assertions of hardware health. A stale source clears the image and shows a disconnect state. YOLO image freshness proves the output stream is alive, not that detection accuracy has been validated. Depth display is normalized per frame for viewing and is not a measurement scale.
- **模块测试:** categorized one-click checks reuse the ROS environment check, six-axis URDF FK, existing OpenVINO YOLO synthetic-image smoke test, P5 chassis protocol/source checks, P6 unit/simulation scenarios, and P7 PTY scenarios. On the migrated NUC, YOLO uses the copied environment at `~/lunar-robot/lab/vision` and the model under `models/blue_block`; the moved virtual environment and overlay are rebuilt in that new location before use. Four LIVE read-only checks report current camera, MID-360, chassis, and arm ROS topics, publisher nodes, sampled low-bandwidth status rates, status text, and matching diagnostics. They do not launch drivers, subscribe to image or point-cloud streams, or send control commands. Selecting a test and clicking **一键执行** refreshes prerequisites, streams logs, supports stop/timeout, and saves/exports a JSON report. Hardware motion tests remain locked; OFFLINE/SIMULATION results and LIVE snapshots are not physical acceptance.
- **模拟驾驶:** dedicated SIMULATION-only environment. Click the map to capture WASD key events (W/S forward/reverse, A/D left/right, including combinations). Shows the P6 cave simulator's `/console_sim/map` occupancy grid, `/console_sim/odom` position/yaw/velocities and bounded trajectory. No duplicate Mac-side kinematic integration. Key release sends zero; window focus loss, disconnect and leaving the page revoke the simulation session. The P6 0.25-second command watchdog remains active even if the SSH connection disappears. This mathematical differential-drive model is *not* verified physical six-wheel dynamics.
- **系统诊断:** the V0.1 CPU, memory, disk, temperature, Git/deployment version, ROS graph, safety evidence, log, and test-record details. Log entries label the containing file's modification time, not an event time.
- **历史记录:** existing YAML records are kept in the migrated NUC state tree at `~/lunar-robot/tests/state/lunar-cave-robot/tests/`. The app never creates or edits these YAML records.

The mode labels have separate meanings: **SIMULATION** is a simulated subsystem, **OFFLINE** is software-only input and processing, and **LIVE** is an observation of the current ROS/device state. The module-test LIVE checks are strictly read-only snapshots, not physical acceptance. Any later physical acceptance needs its own calibrated evidence and cannot be derived from a SIMULATION or OFFLINE pass.

“未知” is used when a source is absent, unreadable, or unverified. ROS graph presence is not reported as device health; physical E-stop status remains unknown without a verified input. `HOST_WRITE_OK` is not used as controller feedback.

With the isolated P6 simulation service explicitly running on NUC, repeat the full Mac→SSH→NUC protocol/motion smoke test using:

```sh
bash tools/mac-console/acceptance/run-v03a-smoke.sh
```

This standalone test builds the production Swift model (excluding only the app entry point), tests WASD axis combinations, map validation, simulated driving, zero on release, and focus-loss stop. The non-ROS safety tests run with `python3 -m unittest discover -s tools/console-sim/tests -v` from the repo root. Neither test launches a physical controller.

The live-camera audit and current runtime limitations are recorded in [`acceptance/v0.2-camera-audit.md`](acceptance/v0.2-camera-audit.md). A repository topic declaration is not treated as proof that the camera is connected or publishing.

## System health events

The 系统诊断 health view and 测试驾驶舱 summary use one event evaluator over the existing read-only Agent snapshot and fixed vision streams. Each event displays its current state, module, stable rule ID, severity, source-code location, observation, Topic or node/endpoint, source data time, observation time, source, observed and expected ROS Domain, run mode, persistence, and suggested inspection location. A finding begins as a single snapshot; only distinct abnormal observations mark it as sustained. The visible time window shows the first and latest observations. A normal, not-running, or unknown observation clears the active abnormal episode.

Events use four states: normal, abnormal, not running, and unknown. Summary state prioritizes abnormal and unknown evidence; an unknown finding cannot be presented as green normal. Agent loss is reported only after a successful Agent connection has been observed; an initial unavailable connection remains unknown, and a requested reconnect is not counted as loss. ROS graph freshness uses the Agent's monotonic graph_age_ms with a five-second threshold. RGB and depth time out at 1.5 seconds, matching the Agent's fixed frame policy. Missing image publishers are shown as not running unless there is evidence of stale frames; absence alone is not a fault.

The Agent's LUNAR_CONSOLE_EXPECTED_ROS_DOMAIN_ID and LUNAR_CONSOLE_EXPECTED_ROS_NODES values are optional. Domain mismatch and unexpected node disappearance are evaluated only when the corresponding expectation is configured. A Domain mismatch, invalid expected Domain configuration, or stale/unconfirmed graph suppresses downstream node and vision findings to unknown, preventing unverified scope from producing cascading faults. No expected node set means the console reports the observed graph without treating unlisted modules as missing. Invalid expectations remain unknown. Optional telemetry fields preserve decoding compatibility with older Agents; where freshness cannot be established, the affected findings remain unknown.

YOLO output can be suppressed by the node's confidence publish gate. Missing, waiting, or stale YOLO frames remain unknown and are not counted as faults. An incompatible published message type is direct ROS evidence and is reported separately. Synthetic frame identifiers remain labeled as synthetic. A healthy ROS transport or image stream is not evidence that navigation, arm, chassis, or physical sensor hardware is healthy.
