# NUC read-only agent

Python 3.10 agent for Ubuntu 22.04 / ROS 2 Humble. It exposes read-only HTTP `GET` routes and binds strictly to `127.0.0.1:8765`. Do not add a LAN bind option, write route, shell-command endpoint, ROS publisher, service/action client, or device-control operation.

The Mac app forwards its local port 18765 to this loopback endpoint through the configured SSH alias `nuc`. HTTP is not exposed to Tailscale, LAN, or the public internet. The app will not start this process remotely; start and stop the agent on the NUC through the documented operator procedure.

## Start for a development session

Run in a NUC terminal with the ROS Humble environment sourced and the repository at `~/lunar-cave-robot`:

```sh
cd ~/lunar-cave-robot/tools/console-agent
source /opt/ros/humble/setup.bash
python3 -m console_agent
```

Python must be 3.10 and `rclpy` must come from the NUC's existing ROS 2 Humble installation. This agent is a separate process; it does not source either robot workspace overlay. The ROS observer is a single node named `lunar_console_graph_probe`, with parameter services and rosout disabled. It immediately destroys rclpy Humble's implicit parameter-event publisher because this agent never changes parameters. It creates only the three fixed image subscriptions documented below; it creates no publisher, service/action client, or device connection. Its own graph node is omitted from the reported node list.

The HTTP listener is fixed to `127.0.0.1:8765`. `LUNAR_CONSOLE_REPO` and `XDG_STATE_HOME` may point to the NUC's existing repository and existing state directory. They do not enable writes. The agent reads the current Git `HEAD`/branch/dirty state, the deployment owner's `deployed-sha` marker, existing test YAML records, Linux proc/sysfs metrics, and available ROS log files.

The observer also subscribes read-only to three fixed `sensor_msgs/msg/Image` topics: RGB `/camera/color/image_raw`, depth `/camera/depth/image_rect_raw`, and YOLO annotated output `/yolo/annotated_image`. It never starts those publishers. The YOLO node in this repository draws/publishes annotated frames only when a subscriber exists, so running the Agent adds its existing annotation work to the NUC load; it does not change inference thresholds or publish controls. Each subscription uses the ROS sensor-data QoS profile and keeps only the latest frame in memory. The image viewer requests PNG frames over the same loopback-only SSH forward; frames are not written to disk.

## API

- `GET /health` returns service liveness and schema version.
- `GET /v1/snapshot` returns a versioned JSON snapshot with source, UTC collection time, metric status/reason, Git and deployment sources, runtime ROS graph, existing test records, and explicit unknown safety evidence.
- `GET /v1/logs?limit=100` returns the latest available ROS log lines, capped at 200. Each entry's `file_modified_at` is the containing file's modification time; it is not the event time of that line.
- `GET /v1/vision` reports the live publisher/type state, latest frame metadata, receive FPS, frame freshness and receive-gap events for the three fixed image streams. Missing publishers/frames have an explicit status and null metrics.
- `GET /v1/vision/{rgb|depth|yolo}/frame.png` returns the latest image as PNG only while its frame age is at most 1.5 seconds. Missing, stale or unsupported frames return a JSON error; arbitrary topic names are not accepted.
- Every other path is `404`; POST/PUT/PATCH/DELETE/OPTIONS return `405` with `Allow: GET`.

CPU is sampled from `/proc/stat`; memory from `/proc/meminfo`; disk from the root filesystem; temperature only from readable Linux thermal sysfs sensors. Missing data is `unknown` with a reason. An empty ROS graph is a valid empty list. No source-code topic declarations are reported as live topics.

For image streams, ROS `header.stamp` remains the source event time. `received_at` is the NUC Agent's wall-clock receipt time, and `frame_age_ms` uses the NUC's monotonic clock. `header_to_agent_ms` is only a clock-dependent comparison; the NUC and sensor clock synchronization is not verified. `frame_rate_hz` is the receive rate observed by this Agent, not a publisher-side promise. `gap_events` counts receive gaps longer than 1.5 seconds; it is not an exact DDS packet-loss count. The depth PNG is stretched to the nonzero min/max of each frame for visibility; its grayscale is not a stable distance scale.

## Tests

```sh
python3 -m unittest discover -s tests -v
```

Tests do not need ROS 2 and do not connect to robot hardware.
