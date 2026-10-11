# P2b real RGB-D data audit (requires approved external data)

No real camera RGB-D bag, calibration or end-to-end grasp was accepted in the original verification. Captures must be authorized and performed by an operator; no command here launches physical robot drivers.

With ROS 2 Humble and an independently running camera/joint state source, run `./tools/p2b/record_rgbd_bag.sh --check-only`. Recording is opt-in: `--confirm-recording`. Bags are written outside Git and must never be uploaded.

For an approved local bag, after building `tzb_ws`, use:

```bash
source /opt/ros/humble/setup.bash
source install/offline/setup.bash
./tools/p2b/audit_rgbd_bag.py --stage capture /path/to/approved/bag
./tools/p2b/replay_perception_bag.sh /path/to/approved/bag
```

Replay requires locally installed OpenVINO weights and Python dependencies. It isolates domain 187 and uses `ROS_LOCALHOST_ONLY=1`, and must not start a camera, serial, navigation or arm control driver. Bag audit checks RGB/depth/pointcloud timestamps, camera intrinsics, joints and TF to `base_link`; do not substitute synthetic results for real-world calibration evidence.
