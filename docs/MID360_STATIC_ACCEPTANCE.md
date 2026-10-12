# MID-360 static point-cloud acceptance on the NUC

This entry point starts only `livox_ros_driver2_node`. It does not start RViz,
Nav2, the chassis, the arm, or any other robot hardware driver. Do not run it
until the sensor is powered and the cable, actual lidar IP, host IP, route, and
firewall have been checked at the vehicle.

## Isolated build on the NUC

The driver source, build, install, logs, and transferred Git bundle live under
`~/mid360_driver_dev`; the existing ROS workspaces are not part of this build.

```bash
source /opt/ros/humble/setup.bash
cd ~/mid360_driver_dev/repo
colcon build \
  --base-paths Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2 \
  --packages-select livox_ros_driver2 \
  --build-base ~/mid360_driver_dev/build \
  --install-base ~/mid360_driver_dev/install \
  --log-base ~/mid360_driver_dev/log
source ~/mid360_driver_dev/install/setup.bash
```

Confirm the package and executable without starting the node:

```bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg executables livox_ros_driver2
ros2 launch --show-args livox_ros_driver2 mid360_static.launch.py
ldd ~/mid360_driver_dev/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node
```

`ldd` must show the Livox SDK2 shared library and no `not found` entries.

## Before connecting/powering the sensor

Record the actual MID-360 IP from its label or approved configuration. Preserve
the two historical NUC JSON files; do not overwrite them. The checked-in `.119`
configuration is only a default. If the actual sensor is `.147`, pass the
preserved `.147` JSON path explicitly after confirming its host address is still
the NUC's wired address.

After the cable is connected, but before launching the driver, read-only checks:

```bash
ip -br address
ip route
ip route get <ACTUAL_LIDAR_IP>
```

The route to the lidar must select the wired Ethernet interface and the host
source address configured in the selected JSON. Do not disable Wi-Fi or
Tailscale. The NUC has previously had Wi-Fi and Ethernet in the same `/24`,
which can route the sensor subnet over Wi-Fi while Ethernet is disconnected.
If the route selects Wi-Fi or the wrong source address, stop and agree on a
narrow route correction before changing network settings.

UFW was observed active during audit, but its rules were not inspected with
administrator privileges. Have an authorized operator inspect the existing
rules before test day; do not change them as part of this procedure. MID-360
configuration uses host UDP receive ports 56101, 56201, 56301, 56401, and
56501. The corresponding lidar-side service/data ports are 56100 through
56500 as specified in the JSON. Verify the SDK2/driver packet directions and
firewall policy against the selected configuration before permitting traffic.

## First live static capture

In terminal 1, after network and firewall checks pass, select the JSON matching
the confirmed lidar IP and wired host IP, then start only the driver:

```bash
source /opt/ros/humble/setup.bash
source ~/mid360_driver_dev/install/setup.bash
ros2 launch livox_ros_driver2 mid360_static.launch.py \
  config_file:=/home/tzb/mid360_driver_dev/repo/Nav2_rea_ws/src/drivers/livox_ros_driver2/livox_ros_driver2/config/MID360_config.json
```

For the preserved `.147` configuration, substitute its confirmed absolute
path. The launch publishes standard `sensor_msgs/PointCloud2` on
`/livox/lidar` and `sensor_msgs/Imu` on `/livox/imu` when packets arrive.

In terminal 2, observe both rates and record a short bag (stop with Ctrl-C):

```bash
source /opt/ros/humble/setup.bash
source ~/mid360_driver_dev/install/setup.bash
ros2 topic list -t
ros2 topic hz /livox/lidar
ros2 topic hz /livox/imu
ros2 bag record -o ~/mid360_static_$(date +%Y%m%d_%H%M%S) /livox/lidar /livox/imu
```

Acceptance requires the expected message types, nonzero and stable rates on
both topics, advancing timestamps, and a bag that can be listed and replayed.
Capture `ros2 topic echo --once /livox/imu` if message-field inspection is
needed. Do not enable any motion-capable stack during this test.
