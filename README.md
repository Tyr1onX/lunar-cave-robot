# Lunar Cave Robot

ROS 2 Humble robot project recovered from the NUC and validated on the ThinkBook. This repository keeps **two separate workspaces**: `tzb_ws/` (arm/perception) and `Nav2_rea_ws/` (navigation). Never source both overlays in the same terminal: both define `rm_serial_driver` / `rm_serial_msgs`.

## Architecture and team onboarding

New contributors: start with the [system architecture and module handoff guide](docs/ARCHITECTURE.md) (中文). It maps the two ROS 2 workspaces, main launch files, ROS topics/TF, navigation-arm interfaces, verified scope, and open integration gaps. The diagrams describe source-level design; they are **not** evidence of physical-robot acceptance.

## Validated boundary

ThinkBook (Ubuntu 22.04 / ROS 2 Humble): original P0 builds completed for 14 arm packages and 18 navigation packages; P1 arm planning and collision checks passed with synthetic feedback; P2 only covered offline software/model checks; P2b provides real-bag capture/replay audit scripts but no real RGB-D bag or hand-eye-calibration acceptance is recorded. **No physical arm, base, camera, lidar or serial-device behavior is verified.**

The publishable snapshot intentionally omits original build/install/log directories, Python environments, captured maps/bags, trained model weights and bundled SDK binary libraries. As a result, a full hardware-driver/model build **requires separately installing authorized upstream dependencies and models**; the isolated offline build below does not cover those components.

## P3–P4b archived offline results

The independent integration branch adds P3 timestamp-safe vision TF conversion, P3b URDF/TF synthetic integration, P4a arm serial framing/parsing, and P4b six-joint angle mapping. See [P3–P4b test commands and measured outcomes](docs/P3_P4B_VALIDATION.md) for source commit SHAs, reproducible tests, and limitations. On ThinkBook, the isolated integrated snapshot built successfully and reported **23 passing colcon tests**; P3b dynamic TF and the synthetic vision-to-place sequence also passed.

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


## Unified engineering archive

The branch archive/unified-engineering-assets is based on archive/p3-p4b-offline-validation. It preserves the P0–P4b baseline and adds the reviewed P5a chassis source audit and its 13 standard-library model tests.

The two STM32 source archives were inspected but are not stored in this branch. The arm archive has no project license declaration and contains an upstream repository with no GitHub license metadata; the chassis archive also has no project-level license. Internal redistribution permission is not documented. See docs/RELEASE_AUDIT.md and docs/PROJECT_STATUS.md. The firmware/arm/ and firmware/chassis/ directories contain audit notes only and are not buildable firmware.

P4c is recorded as safety audit complete with remediation pending. Its reported short SHA 2964e87 does not resolve in this repository, and no P4c branch is present. P5b remains under development because the ROS 2 chassis interface has blocking mismatches.

To clone this archive branch:

    git clone --single-branch --branch archive/unified-engineering-assets https://github.com/Tyr1onX/lunar-cave-robot.git
    cd lunar-cave-robot
    python3 tools/p5a_offline/test_chassis_control_path.py

ROS 2 recovery and isolated workspace build instructions remain in this README and docs/ARCHITECTURE.md. Use Ubuntu 22.04 with ROS 2 Humble, and build/source tzb_ws and Nav2_rea_ws in separate terminals. No firmware source is currently available from this branch.
