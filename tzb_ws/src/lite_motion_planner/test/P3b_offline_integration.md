# P3b offline URDF/TF integration

This procedure uses the default tzb_description.urdf, robot_state_publisher, the existing offline planner and synthetic feedback only. The test optical edge is a fixture: it uses Orbbec's source RPY convention (-pi/2, 0, -pi/2) and zero translation, so it does not encode or claim a measured camera extrinsic.

## Build a clean offline overlay

From the cloned repository root (no physical hardware needed):

    source /opt/ros/humble/setup.bash
    cd tzb_ws
    colcon build --base-paths src --packages-up-to lite_motion_planner tzb_description \
      --build-base build/validation --install-base install/validation \
      --parallel-workers 4 --cmake-args -DBUILD_TESTING=ON
    source install/validation/setup.bash

## Dynamic URDF/TF regression

In terminal 1:

    export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 ROS2CLI_DISABLE_DAEMON=1
    ros2 launch lite_motion_planner p3b_offline.launch.py \
      offline_domain_id:=187 autostart:=false publish_synthetic_target:=false use_rviz:=false

In terminal 2, source the same overlays and environment, then run:

    ros2 run lite_motion_planner p3b_urdf_tf_test.py

It publishes timestamped synthetic joint states, checks exact historical transforms, tests J1-J3 camera motion and J4-J6 invariance, transforms a probe target, and requires a missing timestamp to fail.

## Full simulated vision-to-place path

With the same overlays and isolated domain:

    ros2 launch lite_motion_planner p3b_offline.launch.py \
      offline_domain_id:=187 autostart:=true publish_synthetic_target:=true use_rviz:=false

The synthetic vision node creates optical-frame observations by transforming a fixed test target through the current URDF/RSP TF. The planner and feedback topics are confined to /p2_offline/*. No camera, serial driver, base, lidar, or hardware control node is started.
