# Issue #48 P2: isolated Gazebo Fortress physics simulation

This package adds an isolated Gazebo Fortress (Ignition Gazebo 6) physics overlay for Ubuntu 22.04 / ROS 2 Humble. The P2 worktree layers the preserved P7 and P1 installs, then the P2 overlay. It uses a simulation-only robot description generated from the production URDF, ROS/Gazebo bridges, a Gazebo effort/position controller adapter, the existing OpenVINO YOLO, `pose_estimator`, `tzb_catch`, and `lite_motion_planner`. It never starts a serial driver or the base controller.

## Hardware isolation

Each P2 launch sets `ROS_DOMAIN_ID=216`, `ROS_LOCALHOST_ONLY=1`, and Gazebo Transport partition `tzb-issue48-p2-216`; the Gazebo transport IP is loopback. The catch launch explicitly passes `use_serial_driver:=false`. Arm commands go only to Gazebo controller topics. Stop with the script below. No real serial device, motor, or base command is used.

The simulation URDF is generated at run time into the evidence directory. It reuses the production mesh and joint limits. Only the generated copy gets Gazebo control interfaces, camera frames, and independent effort interfaces for both jaws (Fortress does not enforce the source URDF mimic relation). Joint feedback comes from Gazebo `/joint_states`; gripper contact is read from the Gazebo contact sensor and world state is queried through Gazebo Transport. The block stays a dynamic body under gravity and collision; there is no attach/weld plugin or object-pose command in the grasp path.

The world uses lunar gravity (1.62 m/s²), a static ground/table/place pad, RGB-D camera, and a dynamic 5 cm blue block. Contact/friction and controller gains are simulation assumptions, not experimentally calibrated robot parameters.

## One-command start and stop

On ThinkBook:

```bash
TZB_GZ_AUTOSTART=true TZB_GZ_USE_RVIZ=true \
  /home/qibai/lunar-robot/worktrees/issue-48-p2-gazebo/tzb_ws/src/tzb_sim/tools/start_gazebo_sim.sh
/home/qibai/lunar-robot/worktrees/issue-48-p2-gazebo/tzb_ws/src/tzb_sim/tools/stop_gazebo_sim.sh
```

The start script checks the preserved underlays, OpenVINO XML/BIN model pair, isolated overlay, and conflicting P2 process before launch. If the P2 overlay is missing, it builds `tzb_sim` and the branch-local `lite_motion_planner` copy into that isolated prefix. Set `TZB_GZ_AUTOSTART=false` to leave the sequence idle until a deliberate trigger; set `TZB_GZ_GRASP_PIPELINE=false` for the controller/interface test without the planner sequence.

## Repeat the accepted full-grasp run

The accepted run used a passive probe so it could observe the entire run before the start trigger. Terminal A:

```bash
TZB_GZ_AUTOSTART=false TZB_GZ_LISTEN_NAV_STATUS=true \
TZB_GZ_USE_RVIZ=true TZB_GZ_GRASP_PIPELINE=true \
  /home/qibai/lunar-robot/worktrees/issue-48-p2-gazebo/tzb_ws/src/tzb_sim/tools/start_gazebo_sim.sh
```

Terminal B, start the read-only probe:

```bash
source /opt/ros/humble/setup.bash
source /home/qibai/lunar-robot/worktrees/issue-48-p0-p1/tzb_ws/install/p7/setup.bash
source /home/qibai/lunar-robot/worktrees/issue-48-p0-p1/tzb_ws/install/issue48/local_setup.bash
source /home/qibai/lunar-robot/worktrees/issue-48-p2-gazebo/tzb_ws/install/issue48-p2/local_setup.bash
export ROS_DOMAIN_ID=216 ROS_LOCALHOST_ONLY=1
/home/qibai/robot_p0/.venv/bin/python \
  /home/qibai/lunar-robot/worktrees/issue-48-p2-gazebo/tzb_ws/src/tzb_sim/tools/test_full_grasp.py 240
```

Terminal C, trigger the existing catch sequence after the probe reports ready:

```bash
source /opt/ros/humble/setup.bash
source /home/qibai/lunar-robot/worktrees/issue-48-p0-p1/tzb_ws/install/p7/setup.bash
source /home/qibai/lunar-robot/worktrees/issue-48-p0-p1/tzb_ws/install/issue48/local_setup.bash
source /home/qibai/lunar-robot/worktrees/issue-48-p2-gazebo/tzb_ws/install/issue48-p2/local_setup.bash
export ROS_DOMAIN_ID=216 ROS_LOCALHOST_ONLY=1
ros2 topic pub --once /tzb_catch/start std_msgs/msg/Int8 "{data: 1}"
```

The probe sends no motion commands. It requires real OpenVINO detections, an optical-frame `PoseStamped`, Gazebo bilateral finger contact, a measured block lift and relocation, planner success, and measured return/open states. `test_gazebo_interfaces.py` separately checks the arm controller and two-jaw feedback without executing the pick-and-place sequence.

## ThinkBook acceptance (2026-10-19)

Build and unit test commands:

```bash
cd /home/qibai/lunar-robot/worktrees/issue-48-p2-gazebo/tzb_ws
source /opt/ros/humble/setup.bash
source /home/qibai/lunar-robot/worktrees/issue-48-p0-p1/tzb_ws/install/p7/setup.bash
source /home/qibai/lunar-robot/worktrees/issue-48-p0-p1/tzb_ws/install/issue48/local_setup.bash
colcon build --base-paths src --packages-select tzb_sim lite_motion_planner \
  --build-base build/issue48-p2 --install-base install/issue48-p2
python3 -m unittest discover -s src/tzb_sim/tests -v
```

Result: build succeeded; 13 unit tests passed. The physical-grasp probe passed all ten acceptance checks:

- OpenVINO blue-block detection: 48 detections, best confidence 0.634.
- Pose estimation: 3 poses in `sim_camera_optical_frame`.
- Gazebo bilateral finger contact and physical lift: 3,745 contact frames; measured block maximum Z 0.4441 m from 0.244 m.
- The block was carried to the place area before release; at the open command it was at (0.0134, -0.5016, 0.4441) m.
- After release and gravity/contact settling, the block rested at (0.0924, -0.5558, 0.0350) m on `place_pad`; Gazebo contact surfaces included `place_pad::place_pad_link::place_pad_collision`.
- The planner returned to the observation pose within 0.0034 rad; Gazebo gripper feedback showed open.

Evidence logs on ThinkBook:

- `/home/qibai/tzb-study-output/issue48-p2-gazebo/logs/gazebo_20261019_064545.log`
- `/home/qibai/tzb-study-output/issue48-p2-gazebo/logs/full_grasp_probe_20261019_064545.log`
- `/home/qibai/tzb-study-output/issue48-p2-gazebo/logs/full_grasp_trace_20261019_064545.csv`
- `/home/qibai/tzb-study-output/issue48-p2-gazebo/logs/generated/tzb_gazebo_robot.urdf`

This is Gazebo physics evidence only. It is not physical contact validation on the robot, hardware safety acceptance, or a calibrated lunar-rover digital twin. P3/Nav2-to-Gazebo integration has not been started.
