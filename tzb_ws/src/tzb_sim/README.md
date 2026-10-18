# tzb_sim offline visual grasp

`tzb_sim` packages the 2026-10-16 synthetic scene and adds only simulation-side launch/configuration. The normal `yolo_node`, `pose_estimator`, `tzb_catch`, and `lite_motion_planner` implementations perform inference, pose estimation, IK, and path planning.

The demo publishes a synthetic RGB image, camera intrinsics, an organized camera-frame point cloud, and a ground-truth blue-cube marker. `publish_gt_detections` is forced off by the demo. The target detections must come from the OpenVINO YOLO node. A virtual controller follows the planner's six-joint commands with velocity limits and returns joint feedback; it also visualizes the gripper's open/close command. It never opens a serial device.

## Build on ThinkBook

From `tzb_ws` after sourcing ROS 2 Humble and the selected dependency underlay:

```bash
source /opt/ros/humble/setup.bash
source install/p7/setup.bash
OPTIONAL_CAMERA_INSTALL="$HOME/tzb-delivery-check-ascii/tzb-repro-v2-2026-10-16/tzb_ws/install"
export AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH:$OPTIONAL_CAMERA_INSTALL/orbbec_camera:$OPTIONAL_CAMERA_INSTALL/orbbec_camera_msgs"
export COLCON_PREFIX_PATH="$COLCON_PREFIX_PATH:$OPTIONAL_CAMERA_INSTALL"
colcon --log-base log/issue48 build --base-paths src \
  --packages-select orbbec_camera_msgs yolo_node pose_estimator tzb_sim \
  --packages-ignore arm2nav_msgs lite_motion_msgs rm_serial_msgs \
    tzb_description rm_serial_driver lite_motion_planner orbbec_camera \
  --allow-overriding yolo_node orbbec_camera_msgs \
  --build-base build/issue48 --install-base install/issue48
```

The package dependencies are declared in `package.xml`. The model XML/BIN pair and prompt sidecar are extracted from the supplied `ska_1.zip` into the preserved `ska_1_reference/best_openvino_model/blue_block` evidence directory; the large `.bin` and `.npz` are intentionally not committed. OpenVINO/Ultralytics and Open3D are provided by the existing perception Python environment on ThinkBook. `pose_estimator` currently declares `orbbec_camera` as an unconditional runtime dependency even when `launch_camera=false`; this ThinkBook already has its package index in the preserved 2026-10-16 install. The start script appends only those optional camera package prefixes after the current branch prefixes and never launches the camera driver. The Orbbec SDK binary is absent from the GitHub worktree, so building the hardware camera driver from this checkout is intentionally not part of this offline build.

## Start and stop

From any directory in the source checkout:

```bash
./tzb_ws/src/tzb_sim/tools/start_offline_demo.sh
./tzb_ws/src/tzb_sim/tools/stop_offline_demo.sh
```

The start script sources the P7 dependency underlay and the isolated `install/issue48` overlay, sets `ROS_LOCALHOST_ONLY=1`, and defaults to ROS domain 214. It launches `demo.launch.py` with the real serial-driver launch argument fixed to `false` and camera launch fixed to `false`. Logs and PID location can be changed with `TZB_SIM_LOG_DIR` and `TZB_SIM_PID_FILE`. `YOLO_NODE_PYTHON`, `POSE_ESTIMATOR_PYTHON`, `TZB_OPENVINO_MODEL`, and `TZB_SIM_ROS_DOMAIN_ID` can be set before launch.

A direct launch is also available from the workspace root:

```bash
source /opt/ros/humble/setup.bash
source install/p7/setup.bash
source install/issue48/local_setup.bash
OPTIONAL_CAMERA_INSTALL="$HOME/tzb-delivery-check-ascii/tzb-repro-v2-2026-10-16/tzb_ws/install"
export AMENT_PREFIX_PATH="$AMENT_PREFIX_PATH:$OPTIONAL_CAMERA_INSTALL/orbbec_camera:$OPTIONAL_CAMERA_INSTALL/orbbec_camera_msgs"
ROS_DOMAIN_ID=214 ROS_LOCALHOST_ONLY=1 \
YOLO_NODE_PYTHON="$HOME/robot_p0/.venv/bin/python" \
POSE_ESTIMATOR_PYTHON="$HOME/robot_p0/.venv/bin/python" \
TZB_OPENVINO_MODEL="$HOME/tzb-study-output/issue48-p0-p1-2026-10-18/ska_1_reference/best_openvino_model/blue_block" \
ros2 launch tzb_sim demo.launch.py autostart:=true use_rviz:=true
```

The RViz run shows a planned TCP path and feedback-driven joint motion. The synthetic marker and resulting path are software visualization; this package does not simulate physical contact or validate real hardware.
