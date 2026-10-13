# V0.3b · SIMULATION only arm pose preview

This small ROS 2 graph republishes the existing `tzb_description` URDF/STL model and the existing `tzb_description.srdf` joint groups. The Mac Console provides bounded joint sliders. Foxglove renders the model from `robot_state_publisher` TF; this repository does not add a URDF, regenerate meshes, or implement a 3D renderer.

## Start the isolated viewer on the NUC

Foxglove Bridge must be available in ROS 2 Humble. Install the official ROS package once if it is absent:

```sh
sudo apt install ros-humble-foxglove-bridge
```

From the synchronized repository, start only the offline preview:

```sh
cd ~/lunar-cave-robot
bash tools/console-arm-preview/run.sh
```

The runner sources only `/opt/ros/humble`, builds only the existing `tzb_description` package into a temporary directory so the Foxglove Bridge can resolve the package mesh paths, and sets `ROS_DOMAIN_ID=218`, `ROS_LOCALHOST_ONLY=1`, and `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. It starts `robot_state_publisher`, a small `JointState` preview node, and Foxglove Bridge. It does not launch MoveIt, the serial driver, a controller, a sensor, a chassis node, or firmware. Stopping the runner removes its temporary build prefix.

Foxglove Bridge listens only on NUC loopback port `8768`. It exposes `/tf`, `/tf_static`, `/arm_preview/joint_states`, the robot description parameter, and the existing description package's mesh assets. Service calls and client-published topics are disabled. The Mac app opens fixed SSH loopback forwards for `8767` (bounded slider API) and `8768` (Foxglove WebSocket); it never starts ROS remotely.

## View and move the simulated pose

1. Start `run.sh` on the NUC and leave it running.
2. In Lunar Robot Console, select **机械臂预览**. The page is marked **SIMULATION** and reads joint names and bounds from the server's existing URDF/SRDF.
3. Click **打开 Foxglove 3D**. Foxglove opens with a connection to `ws://127.0.0.1:18768`. Add a 3D panel if the layout is empty. The ROS `robot_description` parameter supplies the URDF; the panel displays the fixed camera, TCP and six-axis arm from TF. Use the Console sliders to update the simulated `/arm_preview/joint_states` stream and watch the pose update.
4. Use the **夹爪模拟开合** slider to move the two existing jaw joints symmetrically within their URDF limits. This direction is a visualization convention; the real open/close polarity is not verified.

The API binds `127.0.0.1:8767` only and accepts exactly one full six-joint pose plus a normalized gripper slider value. Every joint is checked against the URDF limits; unknown joints, partial updates, non-finite values, stale sequence numbers, and out-of-range values are rejected. The viewer graph receives only the `sensor_msgs/msg/JointState` data and computes transforms through the existing `robot_state_publisher`.

## Model facts and calibration limits

| Item | Repository model | Verification limit |
| --- | --- | --- |
| Arm group | SRDF chain `base_link` → `J6_link`, six revolute joints in URDF order | Geometry and frame chain checked from repository assets; no mechanical zero/reference pose measured |
| Gripper group | SRDF `zhua`: `Claw_left_joint`, `claw_right_joint` | Two sliders map to mirrored URDF ranges for visualization; physical polarity and travel not verified |
| J3 camera | Fixed `camera_joint`, parent `J3_link`, xyz `(-0.0300, -0.1553, 0.2745) m`, rpy `(-0.5236, 0, 0) rad` | These are URDF export values; camera mounting and optical frame have not been measured/calibrated on the actual arm |
| TCP | URDF fixed `J6_link` → `tcp_link`, xyz `(-0.015, 0.001, 0.110) m`, zero rpy | Exported geometry only; tool-center point and hand-eye calibration not physically verified. SRDF `hand` still names `J6_link` as its parent link |
| Chassis | URDF `base_link` visual mesh | Not a complete six-wheel chassis digital twin; no wheel dynamics/collision claims |

The six arm limits are loaded from URDF at runtime and shown in radians and degrees in the UI. They are software model bounds, not measured safe physical limits. The preview uses the all-zero model pose only as a convenient starting point.

## Offline checks

Run the tests from any host with Python 3.10+; they parse the committed URDF, SRDF, meshes, camera/TCP offsets and reject unsafe preview inputs. They do not start ROS or any device:

```sh
python3 -m unittest discover -s tools/console-arm-preview/tests -v
python3 tzb_ws/src/serial/driver/test/test_tzb_urdf_joint_fk.py
```

For the Foxglove 3D panel, the current Foxglove app supports a URDF custom layer controlled by ROS transforms. The model and mesh files are served through the loopback Foxglove Bridge. Foxglove's web app may require its own sign-in; the Console does not submit account information. ROS data reaches the viewer through the local SSH forward. This preview exposes no camera image stream or robot-control topics.
