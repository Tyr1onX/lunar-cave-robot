# Four-stage end-to-end robot flow audit

## Scope and evidence policy

The accompanying diagram is used as an information-architecture reference only. Its algorithm names, controller behavior, hardware feedback and stage boundaries are not accepted as implementation evidence. The relationships below were checked against the repository's ROS 2 Launch files, node publishers/subscribers, configuration, serial protocol and recorded offline validation. No raw ROS workspace was changed or launched for this audit.

Evidence labels used by the Console:

- **Source / Launch defined**: a repository source, parameter or selected Launch connects the interfaces. This does not establish runtime delivery or hardware behavior.
- **Conditional source path**: both ends exist, but the path is disabled by the default launch/configuration or depends on an option.
- **Offline software validation**: an isolated software run exercised the path; it does not establish physical integration.
- **Integration unconfirmed**: code or a protocol describes an interface, but the deployed robot startup or receiving endpoint has not been verified.

ROS Graph topic-name visibility is not treated as proof that publishers and subscribers are connected. Host-side serial write status is not treated as controller acknowledgement. The existing Agent and topology resolver remain read-only and are the only source of runtime health status.

## System flow

```text
PERCEPTION AND TARGET RECOGNITION
  RGB camera ──> YOLO detections ─┐
  depth cloud + CameraInfo ────────┴─> target pose / TF ──────────────┐
  MID-360 point cloud ─────────────────> Point-LIO ─> terrain cloud ──┼──> Nav2 route planning
  MID-360 IMU ─> IMU transform ─> complementary filter ─> Point-LIO  │
  terrain cloud ─> virtual LaserScan ─> SLAM Toolbox ─> /map ─────────┘
                                                        map→odom_init is a separate static TF

PLANNING AND DECISION
  target pose ─> arm planner ─> arm joint command ─┐
  /map + terrain cloud + localization TF ─> Nav2 ──┼─> host communication bridges
  P6 coordinator ── NavigateToPose Action ─ Nav2   │
                 └─ optional/offline arm task ─────┘

MOTION CONTROL
  arm planner ─> rm_serial_driver ─> arm serial frame
  Nav2 ─> velocity smoother ─> /cmd_vel ─> chassis_serial_bridge ─> 14-byte USART1 frame

EXECUTION AND FEEDBACK
  arm STM32 response frame ─> rm_serial_driver ─> /serial/arm_joint_state ─> arm planner
  chassis STM32: no acknowledgement is defined by the current 14-byte protocol
  host bridge status ─> software status topics ─> (offline/conditional) P6 coordinator
  physical E-stop ─> no verified ROS/Agent interface; kept as an unconnected boundary
```

The diagram deliberately does not present the sequence as an entity-validated autonomous mission. In particular, P6's launch uses `ROS_LOCALHOST_ONLY=1`, defaults to an isolated simulator and a synthetic arm backend. Its runtime report is offline software evidence. The entity robot's main launch integration remains unconfirmed.

## Interface and evidence matrix

| Stage / lane | Nodes and interface relationship | Evidence and limits |
|---|---|---|
| Perception: image | camera → YOLO on `/camera/color/image_raw`; YOLO and pose estimator both consume `/yolo/detections` | `perception_bringup.launch.py`, YOLO launch and pose-estimator parameters define the path. The selected camera config defaults to Gemini 336L but can select D435i. YOLO's confidence/hold-time publish gate means an empty detection topic is not by itself a failure. |
| Perception: target position | camera depth points + CameraInfo and YOLO detections → pose estimator → `/pose_estimator/object_pose` and TF | Source/configuration defines the interfaces. The Agent does not read TF frame connectivity; runtime transform correctness is not established by Graph topic names. |
| Perception: optional visual suggestion | YOLO → `arm2nav` → arm planner | Conditional. `bring_launch.launch.py` defaults `use_arm2nav=false`; planner `sentry_mode` defaults false. Do not show as a default dependency. |
| Localization: lidar and IMU | MID-360 `/livox/lidar` → Point-LIO; `/livox/imu` → `imu_transform` → `/livox/imu_aft_tf` → complementary filter → `/imu/data` → Point-LIO | The selected single-lidar Launch and Point-LIO config define these inputs. This is not proof that the physical sensor is powered or publishing valid measurements. |
| Localization: terrain, scan and map | Point-LIO `/cloud_registered` and `/odometry` → terrain analysis → `/terrain_map`; `/terrain_map` → pointcloud-to-LaserScan → `/scan` → SLAM Toolbox → `/map` | Selected `mapping_navigation_single_lidar.launch.py` and configs define this chain. `/terrain_map` separately feeds Nav2; `/scan` is used by SLAM Toolbox in this Launch. |
| Localization TF | Point-LIO publishes `odom_init → base_frame`; a separate static node publishes `map → odom_init` | The selected Launch/config defines the two TF sources. Agent Graph does not collect the live frame tree, so runtime TF connectivity remains unknown. SLAM Toolbox has transform publishing disabled in this mode. |
| Planning: navigation | `/map`, `/terrain_map`, localization TF → Nav2 → smoothed `/cmd_vel` | Selected Nav2 config/Launch defines the path. Prior PTY testing exercised host software only; it is not physical chassis validation. |
| Planning: arm | target pose → `lite_motion_planner` / `tzb_catch` → `/tzb_catch/arm_joint_command` | Source/configuration defines this path. The planner source uses WDLS with adaptive SDLS fallback, OMPL RRTConnect and Ruckig time parameterization; these details belong in technical details, not the beginner diagram. |
| Planning: task coordination | P6 coordinator ↔ `/navigate_to_pose`; coordinator `/p6/arm_task/request` and cancel ↔ staged arm task interface | The P6 launch and report provide offline software validation. P6 staged arm mode defaults off in the legacy planner config; physical main-launch integration is not verified. |
| Control: arm | arm joint command → `rm_serial_driver` → protocol frame; controller response → driver → joint-state topic | Repository source and `docs/STM32_PROTOCOL_SPEC.md` define host/firmware frames. This does not establish current serial attachment, firmware deployment or physical joint feedback. |
| Control: chassis | Nav2 `/cmd_vel` and P6 stop request → `chassis_serial_bridge` → USART1 14-byte frame | Source/protocol evidence. The frame has no STM32 acknowledgement; `HOST_WRITE_OK` is local write status only. No physical movement claim is made. |
| Feedback and safety | arm joint state and software status topics; chassis host link status; physical E-stop boundary | P6 subscribes to several software status interfaces, exercised only in isolated software validation. The Agent observes some topic names but not message values. No verified physical E-stop interface exists. |

## UI mapping requirements

The beginner home is organized in the four stages above, with separate vision and lidar lanes in stage 1, navigation / arm planning / task coordination in stage 2, host bridges in stage 3, and physical endpoint boundaries plus returned software evidence in stage 4. Inter-stage connectors must reference the catalog's existing edge IDs and evidence grades. Conditional, offline-only and unconfirmed links must remain visually distinct from default source-defined paths.

Stages expand in place. Each expanded step provides plain-language input, processing and output; the existing node evidence panel remains the path to ROS names, TF limitations, workspace/process evidence, source paths and live telemetry. The advanced engineering topology and legacy capability pages remain available. Unknown states are suppressed on the default home; direct abnormal or stale evidence may be highlighted without inferring a root cause from downstream dependency.

## Source references

- `tzb_ws/src/pose_estimator/launch/perception_bringup.launch.py`
- `tzb_ws/src/pose_estimator/launch/camera_switch.launch.py`
- `tzb_ws/src/pose_estimator/config/params.yaml`
- `tzb_ws/src/yolo_node/yolo_node/yolo_detector.py`
- `tzb_ws/src/lite_motion_planner/launch/bring_launch.launch.py`
- `tzb_ws/src/lite_motion_planner/config/tzb_planner.yaml`
- `tzb_ws/src/lite_motion_planner/src/planning_node.cpp`
- `tzb_ws/src/lite_motion_planner/src/motion_planner.cpp`
- `tzb_ws/src/lite_motion_planner/src/trajectory_generator.cpp`
- `tzb_ws/src/serial/driver/src/node.cpp`
- `Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py`
- `Nav2_rea_ws/src/bringup/config/slam_toolbox_single_lidar.yaml`
- `Nav2_rea_ws/src/bringup/config/nav2_params.yaml`
- `Nav2_rea_ws/src/p6_autonomous_mission/launch/p6_autonomous_mission.launch.py`
- `Nav2_rea_ws/src/p6_autonomous_mission/p6_autonomous_mission/mission_coordinator.py`
- `firmware/chassis/底盘串口通信协议.md`
- `docs/STM32_PROTOCOL_SPEC.md`
- `docs/p6/ros2-runtime-acceptance.md`
- `docs/p7/ros2-runtime-acceptance.md`
