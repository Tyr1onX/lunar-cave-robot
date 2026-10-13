# V0.1 acceptance evidence

Captured 2026-10-13 from the native Mac dashboard connected to the NUC's temporary read-only Agent over the app's fixed SSH local forward. The PNG is a real application-window capture; the NUC host identifier, Git SHAs, and username-bearing ROS log path are obscured. Metric readings and connection indicators remain as captured.

`snapshot-v1-redacted.json` is the body returned by `GET /v1/snapshot` (HTTP 200) during the same live session. Its structure and measurements come from the running Agent. Host identity, commit IDs, user-home paths, and thermal sysfs paths are replaced with non-identifying values; the file keeps the API's schema and field types.

## Reproduction and observed result

- Mac: Apple Silicon, macOS 26.6; the app was built locally with the installed compatible SDK. The Mac does not need ROS 2.
- NUC: Ubuntu 22.04.5 LTS, Python 3.10.12, ROS 2 Humble environment sourced. The Agent ran from a temporary copy under `/tmp`; no service was installed or enabled.
- Started the GUI with the Agent stopped: the app reported that the Agent was unavailable and showed no live snapshot.
- Started the Agent: the app reported the fixed SSH forward and Agent online, then displayed fresh CPU, memory, root-disk, thermal, and ROS graph values from the NUC.
- Stopped the Agent while leaving the SSH forward in place: after the next failed poll the GUI marked the Agent disconnected and cleared the snapshot and logs.
- Restarted the Agent: the GUI returned online and displayed a new snapshot timestamp and fresh readings.
- The observed ROS graph was empty; the deployed SHA and ROS log directory were unavailable and stayed explicitly unknown. Physical E-stop and hardware health stayed unknown; the control interface remained read-only.

## Verification

- Console Agent unit and HTTP contract suite: 8/8 passed locally and on the NUC.
- Native Mac sources type-checked and the `.app` bundle built successfully.
- SwiftPM/XCTest could not run on this Mac because its SwiftPM binary crashes on a system `BuildServerProtocol` mismatch and the installed Command Line Tools do not provide XCTest to direct `swiftc`. The committed state-regression tests were instead run with the direct Swift regression runner documented in the PR.
