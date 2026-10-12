#!/usr/bin/env python3
"""Offline ROS 2 -> velocity_smoother -> chassis bridge -> PTY receiver test.

Run after building/sourcing the Nav2 workspace and installing pyserial is not
required. No physical serial device is opened; the bridge target is a PTY symlink.
"""
import errno
import math
import os
import pty
import select
import signal
import struct
import subprocess
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


FRAME_SIZE = 14
HEADER = 0xA5
TAIL = 0x4A


class FrameCollector:
    def __init__(self, fd):
        self.fd = fd
        self.buffer = bytearray()

    def read(self, timeout=0.02):
        frames = []
        ready, _, _ = select.select([self.fd], [], [], timeout)
        if ready:
            try:
                self.buffer.extend(os.read(self.fd, 4096))
            except BlockingIOError:
                pass
            except OSError as error:
                # Linux PTY master reports EIO until the bridge opens the new slave.
                if error.errno != errno.EIO:
                    raise
            while True:
                try:
                    start = self.buffer.index(HEADER)
                except ValueError:
                    self.buffer.clear()
                    break
                if start:
                    del self.buffer[:start]
                if len(self.buffer) < FRAME_SIZE:
                    break
                if self.buffer[13] != TAIL:
                    del self.buffer[0]
                    continue
                packet = bytes(self.buffer[:FRAME_SIZE])
                del self.buffer[:FRAME_SIZE]
                values = struct.unpack("<fff", packet[1:13])
                frames.append((time.monotonic(), packet, values))
        return frames


def check(condition, message):
    if not condition:
        raise AssertionError(message)


def make_twist(vx=0.0, vy=0.0, wz=0.0):
    msg = Twist()
    msg.linear.x = float(vx)
    msg.linear.y = float(vy)
    msg.angular.z = float(wz)
    return msg


def replace_link(link, target):
    candidate = str(link) + ".next"
    try:
        os.unlink(candidate)
    except FileNotFoundError:
        pass
    os.symlink(target, candidate)
    os.replace(candidate, link)


def run_for(node, publisher, collector, duration, command=None, publish_period=0.03):
    end = time.monotonic() + duration
    next_publish = 0.0
    captured = []
    while time.monotonic() < end:
        now = time.monotonic()
        if node is not None:
            rclpy.spin_once(node, timeout_sec=0.0)
        if publisher is not None and command is not None and now >= next_publish:
            publisher.publish(command)
            rclpy.spin_once(node, timeout_sec=0.0)
            next_publish = now + publish_period
        captured.extend(collector.read(0.01))
    return captured


def assert_frame_set(frames, label):
    check(frames, f"{label}: no valid 14-byte frames received")
    for _, packet, values in frames:
        check(len(packet) == FRAME_SIZE, f"{label}: unexpected frame length")
        check(packet[0] == HEADER and packet[-1] == TAIL, f"{label}: invalid framing")
        check(all(math.isfinite(value) for value in values), f"{label}: non-finite wire value")
        check(abs(values[1]) < 1.0e-7, f"{label}: lateral wire velocity was nonzero")


def assert_steady(frames, vx, wz, label, tolerance=0.08):
    assert_frame_set(frames, label)
    tail = frames[-5:]
    check(len(tail) >= 3, f"{label}: fewer than three settled frames")
    mean_vx = sum(item[2][0] for item in tail) / len(tail)
    mean_wz = sum(item[2][2] for item in tail) / len(tail)
    check(abs(mean_vx - vx) <= tolerance, f"{label}: vx {mean_vx:.3f} != {vx:.3f}")
    check(abs(mean_wz - wz) <= tolerance, f"{label}: wz {mean_wz:.3f} != {wz:.3f}")


def stop_process(process, log_path):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=5)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        process.kill()
        process.wait(timeout=3)
    if process.returncode not in (0, -signal.SIGINT, 130):
        try:
            detail = Path(log_path).read_text(errors="replace")[-4000:]
        except Exception:
            detail = ""
        raise RuntimeError(f"child process exited {process.returncode}: {detail}")


def main():
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    # Use a dedicated domain for every test run to avoid any ambient robot session.
    os.environ["ROS_DOMAIN_ID"] = os.environ.get("P5B_ROS_DOMAIN_ID", "53")
    test_input_topic = "/p5b_pty_test/cmd_vel_nav"
    test_output_topic = "/p5b_pty_test/cmd_vel"

    root = Path(tempfile.mkdtemp(prefix="p5b-pty-"))
    bridge_log = root / "bridge.log"
    smoother_log = root / "smoother.log"
    smoother_params = root / "smoother.yaml"
    smoother_params.write_text(
        """velocity_smoother:
  ros__parameters:
    use_sim_time: false
    smoothing_frequency: 20.0
    scale_velocities: false
    feedback: OPEN_LOOP
    max_velocity: [0.6, 0.0, 1.5]
    min_velocity: [-0.6, 0.0, -1.5]
    deadband_velocity: [0.0, 0.0, 0.0]
    velocity_timeout: 0.1
    max_accel: [2.0, 0.0, 3.0]
    max_decel: [-2.0, 0.0, -3.0]
    odom_topic: /odometry
    odom_duration: 0.1
""",
        encoding="utf-8",
    )

    master1, slave1 = pty.openpty()
    os.set_blocking(master1, False)
    slave_path1 = os.ttyname(slave1)
    os.close(slave1)
    serial_link = root / "chassis-serial"
    os.symlink(slave_path1, serial_link)
    collector = FrameCollector(master1)
    env = os.environ.copy()

    rclpy.init(args=None)
    node = rclpy.create_node("p5b_pty_ros2_integration_test")
    link_status = []
    node.create_subscription(
        String, "/chassis/serial_link_status",
        lambda message: link_status.append((time.monotonic(), message.data.strip())), 10)
    bridge_out = open(bridge_log, "w")
    smoother_out = open(smoother_log, "w")
    bridge = subprocess.Popen(
        [
            "ros2", "launch", "chassis_serial_bridge", "serial.launch.py",
            f"device_name:={serial_link}",
            f"command_topic:={test_output_topic}",
        ],
        env=env,
        stdout=bridge_out,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    smoother = None
    master2 = None
    try:
        time.sleep(1.0)
        check(bridge.poll() is None, f"bridge launch exited: {bridge_log.read_text(errors='replace')}")
        start_frames = run_for(None, None, collector, 0.25)
        status_deadline = time.monotonic() + 2.0
        while time.monotonic() < status_deadline and not any(
                status == "HOST_WRITE_OK" for _, status in link_status):
            rclpy.spin_once(node, timeout_sec=0.05)
        check(any(status == "HOST_WRITE_OK" for _, status in link_status),
              "bridge did not publish host-side serial write health")
        ambient = node.get_publishers_info_by_topic(test_output_topic)
        check(not ambient, f"ambient bridge-topic publishers in isolated domain: {[p.node_name for p in ambient]}")
        assert_frame_set(start_frames, "startup safe zeros")
        check(all(max(abs(v) for v in item[2]) < 1e-7 for item in start_frames),
              f"bridge emitted motion before receiving a fresh command: {[item[2] for item in start_frames[-5:]]}")

        smoother = subprocess.Popen(
            [
                "ros2", "run", "nav2_velocity_smoother", "velocity_smoother",
                "--ros-args", "--params-file", str(smoother_params),
                "-r", f"cmd_vel:={test_input_topic}",
                "-r", f"cmd_vel_smoothed:={test_output_topic}",
            ],
            env=env,
            stdout=smoother_out,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        configured = False
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            if smoother.poll() is not None:
                raise RuntimeError(f"velocity_smoother exited: {smoother_log.read_text(errors='replace')}")
            result = subprocess.run(
                ["ros2", "lifecycle", "get", "/velocity_smoother"],
                env=env, capture_output=True, text=True, timeout=2,
            )
            if result.returncode == 0 and "unconfigured" in result.stdout.lower():
                configured = True
                break
            time.sleep(0.2)
        check(configured, "velocity_smoother lifecycle node did not become available")
        for transition in ("configure", "activate"):
            result = subprocess.run(
                ["ros2", "lifecycle", "set", "/velocity_smoother", transition],
                env=env, capture_output=True, text=True, timeout=10,
            )
            check(result.returncode == 0, f"velocity_smoother {transition} failed: {result.stderr}")

        nav_pub = node.create_publisher(Twist, test_input_topic, 10)
        deadline = time.monotonic() + 10
        while nav_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        check(nav_pub.get_subscription_count() > 0, "velocity_smoother did not subscribe to isolated Nav2 input topic")

        cases = [
            ("Nav2 forward", make_twist(0.30, 0.0, 0.0), 0.30, 0.0),
            ("Nav2 reverse", make_twist(-0.25, 0.0, 0.0), -0.25, 0.0),
            ("Nav2 left turn", make_twist(0.0, 0.0, 0.45), 0.0, 0.45),
            ("Nav2 right turn", make_twist(0.0, 0.0, -0.45), 0.0, -0.45),
            ("Nav2 stop", make_twist(), 0.0, 0.0),
        ]
        for label, command, vx, wz in cases:
            frames = run_for(node, nav_pub, collector, 0.55, command)
            assert_steady(frames, vx, wz, label)
            print(f"PASS {label}: /cmd_vel_nav -> velocity_smoother -> /cmd_vel -> 14B PTY")

        lateral = run_for(node, nav_pub, collector, 0.4, make_twist(0.15, 0.3, 0.0))
        assert_frame_set(lateral, "differential model lateral constraint")
        check(all(abs(item[2][1]) < 1e-7 for item in lateral),
              "diff-drive smoother emitted a lateral wire command")
        print("PASS differential-drive lateral constraint: wire Vy remains zero")

        # Stop the smoother so direct command validation is isolated at the bridge input.
        stop_process(smoother, smoother_log)
        smoother = None
        time.sleep(0.2)
        direct_pub = node.create_publisher(Twist, test_output_topic, 10)
        deadline = time.monotonic() + 5
        while direct_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        check(direct_pub.get_subscription_count() > 0, "bridge did not subscribe to /cmd_vel")

        direct_frames = run_for(node, direct_pub, collector, 0.25, make_twist(0.2, 0.0, 0.0))
        assert_steady(direct_frames, 0.2, 0.0, "fresh direct command")

        invalid_cases = [
            ("NaN", make_twist(float("nan"), 0.0, 0.0)),
            ("Inf", make_twist(0.0, 0.0, float("inf"))),
            ("vx over limit", make_twist(0.61, 0.0, 0.0)),
            ("wz over limit", make_twist(0.0, 0.0, 1.51)),
            ("unsupported lateral", make_twist(0.2, 0.1, 0.0)),
            ("unsupported axis", make_twist(0.0, 0.0, 0.0)),
        ]
        invalid_cases[-1][1].angular.x = 0.1
        for label, command in invalid_cases:
            frames = run_for(node, direct_pub, collector, 0.16, command)
            assert_frame_set(frames, label)
            check(all(max(abs(v) for v in item[2]) < 1e-7 for item in frames[-3:]),
                  f"{label}: bridge did not fail to zero")
            print(f"PASS invalid input {label}: zero frame")

        small_lateral = run_for(node, direct_pub, collector, 0.18, make_twist(0.1, 0.005, 0.2))
        assert_steady(small_lateral, 0.1, 0.2, "within-tolerance lateral")
        print("PASS within-tolerance lateral input is encoded as Vy=0")

        run_for(node, direct_pub, collector, 0.08, make_twist(0.3, 0.0, 0.0))
        stale = run_for(node, None, collector, 0.2)
        assert_frame_set(stale, "command freshness timeout")
        late_frames = [item for item in stale if item[0] >= stale[0][0] + 0.13]
        check(late_frames and all(max(abs(v) for v in item[2]) < 1e-7 for item in late_frames),
              "bridge did not stop after 100ms command timeout")
        print("PASS stale command: zero frames after 100ms freshness window")

        # Move the PTY symlink to a new simulated receiver while closing the old peer.
        run_for(node, direct_pub, collector, 0.08, make_twist(0.22, 0.0, 0.0))
        master2, slave2 = pty.openpty()
        os.set_blocking(master2, False)
        slave_path2 = os.ttyname(slave2)
        os.close(slave2)
        replace_link(serial_link, slave_path2)
        disconnect_started = time.monotonic()
        status_mark = len(link_status)
        os.close(master1)
        master1 = None
        collector2 = FrameCollector(master2)
        reconnect_frames = run_for(node, None, collector2, 3.5)
        reconnect_statuses = link_status[status_mark:]
        failed_statuses = [stamp for stamp, status in reconnect_statuses if status == "DISCONNECTED"]
        recovered_statuses = [stamp for stamp, status in reconnect_statuses if status == "HOST_WRITE_OK"]
        check(failed_statuses, "bridge did not publish DISCONNECTED after PTY peer loss")
        check(failed_statuses[0] - disconnect_started < 1.0,
              "bridge did not expose host-side PTY loss promptly")
        check(recovered_statuses and recovered_statuses[0] > failed_statuses[0],
              "bridge did not publish recovery after opening the replacement PTY")
        print("PASS chassis link health: PTY loss detected under 1s; reconnect reports host write recovery")
        assert_frame_set(reconnect_frames, "PTY reconnect")
        check(all(max(abs(v) for v in item[2]) < 1e-7 for item in reconnect_frames),
              "bridge replayed a pre-disconnect nonzero command after reconnect")
        print("PASS serial disconnect/reconnect: reconnect starts with zero only")

        resumed = run_for(node, direct_pub, collector2, 0.22, make_twist(-0.15, 0.0, 0.0))
        assert_steady(resumed, -0.15, 0.0, "fresh command after reconnect")
        print("PASS fresh post-reconnect command resumes output")

        run_for(node, direct_pub, collector2, 0.08, make_twist(0.15, 0.0, 0.0))
        stop_process(bridge, bridge_log)
        bridge = None
        shutdown_frames = collector2.read(0.05)
        check(shutdown_frames, "graceful shutdown produced no final zero frame")
        check(max(abs(v) for v in shutdown_frames[-1][2]) < 1e-7,
              "graceful shutdown did not end with a zero frame")
        print("PASS graceful shutdown: final PTY frame is zero")

        print("RESULT=PASS_ROS2_PTY_INTEGRATION")
    finally:
        if smoother is not None:
            try:
                stop_process(smoother, smoother_log)
            except Exception:
                pass
        if bridge is not None:
            try:
                stop_process(bridge, bridge_log)
            except Exception:
                pass
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        for fd in (master1, master2):
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
        bridge_out.close()
        smoother_out.close()


if __name__ == "__main__":
    main()
