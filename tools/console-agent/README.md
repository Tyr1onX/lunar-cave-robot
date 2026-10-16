# NUC read-only agent

Python 3.10 agent for Ubuntu 22.04 / ROS 2 Humble. It exposes read-only HTTP `GET` routes and binds strictly to `127.0.0.1:8765`. Do not add a LAN bind option, write route, shell-command endpoint, ROS publisher, service/action client, or device-control operation.

The V0.3c module test runner is a separate service process in `console_agent.module_tests`, with its own loopback listener at `127.0.0.1:8769`. It is not an Agent write route. Install and start it using [`install-module-test-service.sh`](install-module-test-service.sh); its API accepts only fixed test IDs in the registry and never accepts a command or free-form arguments. The test runner runs as the current user and does not start robot hardware.

The registry reuses the ROS environment check, URDF FK, OpenVINO YOLO synthetic-image smoke test, P5 chassis protocol/source checks, P6 unit and six-scenario suites, and P7 twelve-scenario PTY suite. In the standard layout, YOLO uses `~/lunar-robot/lab/vision` for its venv, built ROS overlay and `models/blue_block` bundle; it does not install packages during a test. P6/P7 simulations build only inside `~/lunar-robot/code` when required and use synthetic inputs/PTYs. Reports and scenario logs are saved under `~/lunar-robot/tests/state/`. Four LIVE entries inspect the existing local ROS graph and low-bandwidth status topics only; they never launch drivers, subscribe to camera images or lidar point clouds, or send commands. LIVE read-only observations are status snapshots, not hardware acceptance.

The Mac app forwards its local port 18765 to this loopback endpoint through the configured SSH alias `nuc`. HTTP is not exposed to Tailscale, LAN, or the public internet. The app will not start this process remotely; start and stop the agent on the NUC through the documented operator procedure.

## Start for a development session

Run in a NUC terminal with the ROS Humble environment sourced and the repository at `~/lunar-robot/code`:

```sh
cd ~/lunar-robot/code/tools/console-agent
source /opt/ros/humble/setup.bash
python3 -m console_agent
```

Python must be 3.10 and `rclpy` must come from the NUC's existing ROS 2 Humble installation. This agent is a separate process; it does not source either robot workspace overlay. The ROS observer is a single node named `lunar_console_graph_probe`, with parameter services and rosout disabled. It immediately destroys rclpy Humble's implicit parameter-event publisher because this agent never changes parameters. It creates only the three fixed image subscriptions documented below; it creates no publisher, service/action client, or device connection. Its own graph node is omitted from the reported node list.

The HTTP listener is fixed to `127.0.0.1:8765`. `LUNAR_CONSOLE_REPO` and `XDG_STATE_HOME` may point to the NUC's existing repository and existing state directory. They do not enable writes. The agent reads the current Git `HEAD`/branch/dirty state, the deployment owner's `deployed-sha` marker, existing test YAML records, Linux proc/sysfs metrics, and available ROS log files.

The optional `LUNAR_CONSOLE_EXPECTED_ROS_DOMAIN_ID` and `LUNAR_CONSOLE_EXPECTED_ROS_NODES` environment variables declare a runtime expectation for read-only diagnostics. The Domain value must be from 0 through 232. The node value is a comma-separated list of fully qualified or bare ROS node names (for example, `/camera/camera_node,/yolo_detector`). When these variables are unset, the agent reports observed values without treating an unconfigured Domain or absent node as a failure. Configure these only for modules confirmed to be part of the active run mode. The ROS snapshot also includes `observed_at` and monotonic `graph_age_ms`, which describe the last successful graph scan rather than the HTTP response time.

The separate restricted module-test runner accepts only a registered `test_id`; it does not accept arbitrary shell, path, topic, or arguments. Reports include UTC times, mode/category, fixed parameters, input source, repository SHA/dirty state, optional model hash, elapsed time, result, resource snapshots for P6/P7, and captured logs. The runner stores them under the user's module-test state directory, persists an initial report before spawning a child, and restores completed/interrupted history after a service restart. `GET /v1/tests/reports?limit=20` lists recent report summaries; each report and its full persisted log remain readable by run ID.

The installer limits this dedicated service cgroup to 300% CPU, 8 GiB `MemoryHigh`, 10 GiB `MemoryMax`, 1024 tasks, and low scheduling priority. P6/P7 colcon concurrency and nested Make/CMake workers are limited to two. Each P6/P7 run gets its own persistent scenario log directory below the module-test state directory. With `KillMode=mixed`, systemd signals the runner first; the runner cleans test process groups and records an active run as `INTERRUPTED`. systemd applies its cgroup kill only if the bounded stop timeout expires. These limits apply only to the console test runner, not to the robot's existing system services.

The observer also subscribes read-only to three fixed `sensor_msgs/msg/Image` topics: RGB `/camera/color/image_raw`, depth `/camera/depth/image_rect_raw`, and YOLO annotated output `/yolo/annotated_image`. It never starts those publishers. The YOLO node in this repository draws/publishes annotated frames only when a subscriber exists, so running the Agent adds its existing annotation work to the NUC load; it does not change inference thresholds or publish controls. Each subscription uses the ROS sensor-data QoS profile and keeps only the latest frame in memory. The image viewer requests PNG frames over the same loopback-only SSH forward; frames are not written to disk.

## API

- `GET /health` returns service liveness and schema version.
- `GET /v1/snapshot` returns a versioned JSON snapshot with source, UTC collection time, metric status/reason, Git and deployment sources, runtime ROS graph, existing test records, and explicit unknown safety evidence. The optional `workspaces` field contains a read-only inventory of the new and two original ROS workspaces, current-UID process evidence, explicit per-process Domain/mode values, and a conservative relation to the Agent's existing graph. Older Console builds ignore this additional field; newer builds continue to decode snapshots from older Agents that omit it.
- `GET /v1/logs?limit=100` returns the latest available ROS log lines, capped at 200. Each entry's `file_modified_at` is the containing file's modification time; it is not the event time of that line.
- `GET /v1/vision` reports the live publisher/type state, latest frame metadata, receive FPS, frame freshness and receive-gap events for the three fixed image streams. Missing publishers/frames have an explicit status and null metrics.
- `GET /v1/vision/{rgb|depth|yolo}/frame.png` returns the latest image as PNG only while its frame age is at most 1.5 seconds. Missing, stale or unsupported frames return a JSON error; arbitrary topic names are not accepted.
- Every other path is `404`; POST/PUT/PATCH/DELETE/OPTIONS return `405` with `Allow: GET`.

CPU is sampled from `/proc/stat`; memory from `/proc/meminfo`; disk from the root filesystem; temperature only from readable Linux thermal sysfs sensors. Missing data is `unknown` with a reason. An empty ROS graph is a valid empty list. No source-code topic declarations are reported as live topics.

Workspace inventory reads package manifest names and launch/config paths under the fixed project roots, plus process working directory/executable/cmdline references and only the ROS/domain/mode environment keys it needs. It does not read arbitrary process arguments into the API, execute shell commands, follow symlinked source trees, or open any ROS workspace. Process scan scope is the Agent UID. A sourced shell is not counted as a ROS module process. If a process references multiple workspaces and cannot be assigned uniquely, it is withheld from per-workspace counts and the scan is marked partial. `runtime_status=not_running` means no associated process was observed in that scan scope; it is not a fault. Module `run_expectation` remains `unconfigured`, and module status stays unknown until an active run expectation is confirmed.

`graph_relationship` compares explicit process `ROS_DOMAIN_ID` values with the existing Agent graph only when the graph is fresh. A different/unset Domain, partial process scan, or stale graph yields unknown visibility and cannot create an abnormal event. The Agent does not start additional ROS participants to inspect other Domains. `interface_topics` are source-level candidates, not observed Topics or runtime expectations. The inventory never subscribes to command Topics such as `/cmd_vel` and never publishes ROS data.

For image streams, ROS `header.stamp` remains the source event time. `received_at` is the NUC Agent's wall-clock receipt time, and `frame_age_ms` uses the NUC's monotonic clock. `header_to_agent_ms` is only a clock-dependent comparison; the NUC and sensor clock synchronization is not verified. `frame_rate_hz` is the receive rate observed by this Agent, not a publisher-side promise. `gap_events` counts receive gaps longer than 1.5 seconds; it is not an exact DDS packet-loss count. The depth PNG is stretched to the nonzero min/max of its sampled frame pixels for visibility; its grayscale is not a stable distance scale. Large 16-bit inputs are sampled into a preview of at most 1024×1024 pixels to limit CPU and conversion allocations. `frame_id` carries the ROS Header frame identifier for provenance; the opt-in V0.2a test publisher uses `lunar-v0.2a-synthetic/*` and must never be confused with a physical camera.

YOLO annotated output is confidence-gated by the existing node. A closed confidence gate can legitimately produce no detection or annotated-image messages; absence of this output is not evidence of node or camera failure. The console reports that state as unknown unless an incompatible ROS message type is directly observed.

## Tests

```sh
python3 -m unittest discover -s tests -v
```

Tests do not need ROS 2 and do not connect to robot hardware.
