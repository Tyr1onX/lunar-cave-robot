# Lunar Cave Robot

ROS 2 Humble robot project recovered from the NUC and validated on the ThinkBook. This repository keeps **two separate workspaces**: `tzb_ws/` (arm/perception) and `Nav2_rea_ws/` (navigation). Never source both overlays in the same terminal: both define `rm_serial_driver` / `rm_serial_msgs`.

## Architecture and team onboarding

New contributors: start with the [system architecture and module handoff guide](docs/ARCHITECTURE.md) (中文). It maps the two ROS 2 workspaces, main launch files, ROS topics/TF, navigation-arm interfaces, verified scope, and open integration gaps. The diagrams describe source-level design; they are **not** evidence of physical-robot acceptance.

## Validated boundary

ThinkBook (Ubuntu 22.04 / ROS 2 Humble): original P0 builds completed for 14 arm packages and 18 navigation packages; P1 arm planning and collision checks passed with synthetic feedback; P2 only covered offline software/model checks; P2b provides real-bag capture/replay audit scripts but no real RGB-D bag or hand-eye-calibration acceptance is recorded. **No physical arm, base, camera, lidar or serial-device behavior is verified.**

The publishable snapshot intentionally omits original build/install/log directories, Python environments, captured maps/bags, trained model weights and bundled SDK binary libraries. As a result, a full hardware-driver/model build **requires separately installing authorized upstream dependencies and models**; the isolated offline build below does not cover those components.

## P3–P4b archived offline results

The integration history adds P3 timestamp-safe vision TF conversion, P3b URDF/TF synthetic integration, P4a arm serial framing/parsing, P4b six-joint angle mapping, P4c safety audit evidence, P5a chassis source audit, and the P5b one-way ROS 2 chassis command bridge. See [P3–P4b test commands and measured outcomes](docs/P3_P4B_VALIDATION.md) for source commit SHAs, reproducible tests, and limitations. On ThinkBook, the isolated integrated snapshot built successfully and reported **23 passing colcon tests**; P3b dynamic TF and the synthetic vision-to-place sequence also passed.

**Offline accepted ≠ hardware accepted.** No real STM32 command/feedback, arm motion, gripper, camera hand-eye calibration, navigation-to-arm interaction, or physical safety test is validated. The full STM32 firmware project is not in Git pending separate authorization and license review. Do not launch hardware-facing bringup to reproduce these tests.

## Offline builds (no device access)

Prerequisites: Ubuntu 22.04, ROS 2 Humble, `colcon`, and system dependencies (MoveIt 2, Nav2, slam_toolbox, Eigen/PCL, OMPL, Ruckig, Pinocchio, serial dependencies). Use `rosdep` to inspect missing dependencies; known legacy/external keys `ament_python`, `gz_sim_ros`, `rm_armor_msgs` and `tzb_state_test` cannot be assumed resolvable. In particular, `nav2arm` still needs the unavailable `tzb_state_test` interface at runtime.

From the repository root, in **separate terminals**:

```bash
source /opt/ros/humble/setup.bash
cd tzb_ws
colcon build --base-paths src --packages-skip orbbec_camera \
  --build-base build/offline --install-base install/offline \
  --parallel-workers 4
```

```bash
source /opt/ros/humble/setup.bash
cd Nav2_rea_ws
colcon build --base-paths src --packages-skip livox_ros_driver2 fast_lio point_lio pointcloud_tools \
  --build-base build/offline --install-base install/offline \
  --parallel-workers 4
```

Optional camera/laser driver builds need the original SDK libraries obtained from the appropriate licensed upstream distribution; do not copy libraries from NUC or another local system into Git.

## Safe offline arm test

After completing the arm offline build, from `tzb_ws/`:

```bash
./tools/p2_offline/run.sh
```

The launcher enforces domain 187, localhost-only ROS, synthetic arm feedback, and `/p2_offline/*` command topics; it never starts robot hardware drivers. Ctrl-C stops it. Only use this offline launcher, **not** the default robot bringup files, when no robot operator is present.

P2 model sample inference requires a separately authorized OpenVINO `.bin` matching the versioned XML and an explicitly supplied RGB sample; `./tools/p2_offline/run_yolo_rgb_sample.sh IMAGE MODEL_DIR` does not activate hardware. P2b bag auditing and replay require independently captured and approved bag files: see `tzb_ws/tools/p2b/README.md`.

## Source provenance and distribution

`docs/RELEASE_AUDIT.md` documents copied components, excluded assets and outstanding release blockers. Individual upstream `LICENSE` and `NOTICE` files are preserved; the entire mixed-origin repository is **not** assigned a blanket open-source license. This is an internal private snapshot; any public release still requires project-owner authorization and another license and data audit.


## Unified engineering archive with P5b

Integration branch: archive/unified-engineering-assets-p5b. It starts directly from the latest P5b branch head eefd91e5d4e378e15b34dc37872413f097131132 and carries the reviewed archive documentation forward. The branch retains all P5b code, offline test scripts, the P4c audit and fault-injection records, and the P3–P4b baseline. It does not change main or either existing archive branch.

P4c: the arm safety audit and fault-injection report/test are present. Safety remediation remains open. The original short SHA 2964e87 is still not traceable to a source repository or full commit.

P5a: the six-wheel chassis firmware source audit is complete. Firmware source itself remains outside Git pending redistribution authorization.

P5b: the ROS 2 one-way chassis command path passed the reported offline build and PTY integration validation. This does not verify a real chassis or closed-loop feedback. See docs/p5b/chassis-ros2-integration.md and docs/p5b/run-offline-validation.md.

The arm and chassis STM32 projects remain withheld because project ownership and internal redistribution permission have not been confirmed. The private repository setting does not grant third-party redistribution rights. firmware/arm/ and firmware/chassis/ contain audit notes only.

To clone this integration branch:

~~~bash
git clone --single-branch --branch archive/unified-engineering-assets-p5b https://github.com/Tyr1onX/lunar-cave-robot.git
cd lunar-cave-robot
python3 tools/p5a_offline/test_chassis_control_path.py
~~~

For ROS 2 Humble recovery, build and source tzb_ws and Nav2_rea_ws in separate terminals. Install python3-yaml before running tools/p5b_validate_configuration.py. Build chassis_serial_bridge and run the PTY integration steps in docs/p5b/run-offline-validation.md. No firmware is deployed or flashed by these instructions.
