#!/usr/bin/env python3
"""Static cross-checks for the P5b launch, topics, limits, and package ownership."""
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


ROOT = Path(__file__).resolve().parents[1]


def read(path):
    return (ROOT / path).read_text(encoding="utf-8")


def package_name(path):
    return ET.fromstring(read(path)).findtext("name")


launches = [
    "Nav2_rea_ws/src/bringup/launch/bringup.launch.py",
    "Nav2_rea_ws/src/bringup/launch/mapping.launch.py",
    "Nav2_rea_ws/src/bringup/launch/mapping_navigation_single_lidar.launch.py",
]
for path in launches:
    source = read(path)
    assert source.count("serial.launch.py") == 1, f"{path}: expected exactly one chassis bridge"
    assert "chassis_device" in source, f"{path}: explicit serial device argument missing"
    assert "fake_vel_transform" not in source, f"{path}: legacy velocity transform remains active"
print("PASS each Nav2 bringup starts one explicit chassis bridge and no fake velocity transform")

navigation_launch = read("Nav2_rea_ws/src/navigation/rm_navigation/launch/navigation_launch.py")
assert "('cmd_vel', 'cmd_vel_nav')" in navigation_launch
assert "('cmd_vel_smoothed', 'cmd_vel')" in navigation_launch
print("PASS Nav2 output topic remaps through velocity_smoother to /cmd_vel")

config = yaml.safe_load(read("Nav2_rea_ws/src/bringup/config/nav2_params.yaml"))
controller = config["controller_server"]["ros__parameters"]["FollowPath"]
smoother = config["velocity_smoother"]["ros__parameters"]
assert controller["motion_model"] == "DiffDrive"
assert controller["vx_max"] == 0.6 and controller["vx_min"] == -0.6
assert controller["vy_max"] == 0.0 and controller["vy_min"] == 0.0
assert controller["wz_max"] == 1.5
assert smoother["max_velocity"] == [0.6, 0.0, 1.5]
assert smoother["min_velocity"] == [-0.6, 0.0, -1.5]
assert smoother["velocity_timeout"] == 0.1
assert smoother["max_accel"] == [2.0, 0.0, 3.0]
assert smoother["max_decel"] == [-2.0, 0.0, -3.0]
assert "nav2_params_fake.yaml" not in read(launches[1])
assert "robot_base_frame: base_frame_fake" not in read("Nav2_rea_ws/src/bringup/config/nav2_params.yaml")
print("PASS differential-drive limits, smoother freshness, frame, and mapping config")

bridge_params = yaml.safe_load(read(
    "Nav2_rea_ws/src/drivers/serial/driver/config/serial_params.yaml"
))["chassis_serial_bridge"]["ros__parameters"]
assert bridge_params["baud_rate"] == 115200
assert bridge_params["command_topic"] == "/cmd_vel"
assert bridge_params["tx_period_ms"] == 20
assert bridge_params["command_timeout_ms"] == 100
bridge_source = read(
    "Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp"
)
assert "constexpr std::size_t kFrameSize = 14" in bridge_source
assert "constexpr std::uint8_t kFrameHeader = 0xA5" in bridge_source
assert "constexpr std::uint8_t kFrameTail = 0x4A" in bridge_source
assert "writeFloatLe(frame, 5, 0.0F)" not in bridge_source
assert "const Frame frame = encodeFrame(vx, 0.0F, wz);" in bridge_source
assert package_name("Nav2_rea_ws/src/drivers/serial/driver/package.xml") == "chassis_serial_bridge"
assert package_name("tzb_ws/src/serial/driver/package.xml") == "rm_serial_driver"
print("PASS 115200 8N1 bridge config, 14-byte framing, Vy=0, and unique package names")

fake_source = read(
    "Nav2_rea_ws/src/navigation/fake_vel_transform/src/fake_vel_transform.cpp"
)
assert "aft_tf_vel.angular.z = msg->angular.z;" in fake_source
arm_source = read("tzb_ws/src/serial/driver/src/node.cpp")
assert '"/cmd_vel", rclcpp::SensorDataQoS()' not in arm_source
assert "tzb.legacy_chassis_command_topic" in arm_source
print("PASS fake transform forwards angular.z and arm bridge has no /cmd_vel ownership")

print("RESULT=PASS_P5B_STATIC_CONFIGURATION")
