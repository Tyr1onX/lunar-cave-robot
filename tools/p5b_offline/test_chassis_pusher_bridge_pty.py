#!/usr/bin/env python3
"""Exercise the ROS chassis bridge against a private pseudo-terminal only."""

from __future__ import annotations

import argparse
import os
import pty
import select
import signal
import struct
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


FRAME_SIZE = 14
HEADER = 0xA5
TAIL = 0x4A


def expected_frame(vx: float = 0.0, vy: bytes = b"\x00" * 4, wz: float = 0.0) -> bytes:
    return bytes([HEADER]) + struct.pack("<f", vx) + vy + struct.pack("<f", wz) + bytes([TAIL])


class RosProbe(Node):
    def __init__(self) -> None:
        super().__init__("chassis_pusher_pty_probe")
        self.motion_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.pusher_pub = self.create_publisher(String, "/chassis/pusher_command", 10)
        self.status: list[str] = []
        self.link_status: list[str] = []
        self.create_subscription(
            String, "/chassis/pusher_status", lambda msg: self.status.append(msg.data), 10)
        self.create_subscription(
            String, "/chassis/serial_link_status", lambda msg: self.link_status.append(msg.data), 10)

    def pump(self, seconds: float = 0.02) -> None:
        rclpy.spin_once(self, timeout_sec=seconds)


def create_pty_link(link: Path) -> int:
    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    os.close(slave)
    try:
        link.unlink()
    except FileNotFoundError:
        pass
    link.symlink_to(slave_path)
    os.set_blocking(master, False)
    return master


def collect_frames(probe: RosProbe, master: int, duration: float) -> list[bytes]:
    deadline = time.monotonic() + duration
    pending = bytearray()
    frames: list[bytes] = []
    while time.monotonic() < deadline:
        probe.pump(0.005)
        try:
            ready, _, _ = select.select([master], [], [], 0.005)
            if ready:
                pending.extend(os.read(master, 4096))
        except OSError:
            break
        while pending:
            try:
                start = pending.index(HEADER)
            except ValueError:
                pending.clear()
                break
            if start:
                del pending[:start]
            if len(pending) < FRAME_SIZE:
                break
            if pending[FRAME_SIZE - 1] != TAIL:
                del pending[0]
                continue
            frames.append(bytes(pending[:FRAME_SIZE]))
            del pending[:FRAME_SIZE]
    return frames


def wait_for_frame(
    probe: RosProbe,
    master: int,
    expected: bytes,
    timeout: float = 1.5,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        frames = collect_frames(probe, master, 0.04)
        if expected in frames:
            return
    raise AssertionError(f"frame not observed: {expected.hex(' ').upper()}")


def wait_for_status(
    probe: RosProbe,
    field: str,
    expected: str,
    timeout: float = 1.5,
) -> None:
    values = getattr(probe, field)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe.pump(0.02)
        if expected in values:
            return
    raise AssertionError(f"status {expected!r} not observed in {field}: {values}")


def wait_for_publishers(probe: RosProbe, timeout: float = 3.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        probe.pump(0.02)
        if probe.motion_pub.get_subscription_count() and probe.pusher_pub.get_subscription_count():
            return
    raise AssertionError("bridge ROS subscriptions were not discovered")


def publish_motion(probe: RosProbe, vx: float, wz: float) -> None:
    message = Twist()
    message.linear.x = vx
    message.angular.z = wz
    probe.motion_pub.publish(message)


def publish_unsupported_lateral_motion(probe: RosProbe) -> None:
    message = Twist()
    message.linear.y = 0.05
    probe.motion_pub.publish(message)


def publish_pusher(probe: RosProbe, command: str) -> None:
    probe.pusher_pub.publish(String(data=command))


def verify_disabled_by_default(executable: str) -> None:
    with tempfile.TemporaryDirectory(prefix="lunar-chassis-pusher-disabled-") as directory:
        serial_link = Path(directory) / "serial-device"
        master = create_pty_link(serial_link)
        command = [
            executable,
            "--ros-args",
            "-p", f"device_name:={serial_link}",
            "-p", "tx_period_ms:=20",
        ]
        process = subprocess.Popen(
            command, env=os.environ.copy(), stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True)
        probe = RosProbe()
        try:
            wait_for_publishers(probe)
            wait_for_frame(probe, master, expected_frame())
            publish_pusher(probe, "EXTEND")
            wait_for_status(probe, "status", "REJECTED_DISABLED")
            wait_for_frame(probe, master, expected_frame())
            print("PASS: pusher command is disabled unless explicitly enabled")
        finally:
            probe.destroy_node()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    process.wait(timeout=2)
            if master >= 0:
                os.close(master)


def run(executable: str) -> None:
    if not Path(executable).is_file():
        raise FileNotFoundError(f"bridge executable does not exist: {executable}")

    with tempfile.TemporaryDirectory(prefix="lunar-chassis-pty-") as directory:
        runtime = Path(directory)
        serial_link = runtime / "serial-device"
        master = create_pty_link(serial_link)
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "231"
        env["ROS_LOCALHOST_ONLY"] = "1"
        command = [
            executable,
            "--ros-args",
            "-p", f"device_name:={serial_link}",
            "-p", "pusher_commands_enabled:=true",
            "-p", "pusher_command_timeout_ms:=300",
            "-p", "command_timeout_ms:=100",
            "-p", "tx_period_ms:=20",
        ]
        process = subprocess.Popen(
            command, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        probe = RosProbe()
        replacement_master: int | None = None
        try:
            wait_for_publishers(probe)
            zero = expected_frame()
            wait_for_frame(probe, master, zero)

            motion = expected_frame(0.05, b"\x00" * 4, 0.25)
            publish_motion(probe, 0.05, 0.25)
            wait_for_frame(probe, master, motion)
            wait_for_frame(probe, master, zero, timeout=1.0)

            publish_motion(probe, 0.0, 0.0)
            wait_for_frame(probe, master, zero)

            extend = expected_frame(vy=b"\xEE" * 4)
            publish_pusher(probe, "EXTEND")
            wait_for_status(probe, "status", "HOST_COMMAND_ACCEPTED_EXTEND")
            wait_for_frame(probe, master, extend)

            # A live nonzero chassis command is rejected while the actuator
            # lease is active; the transmitted frame remains stationary.
            publish_motion(probe, 0.05, 0.25)
            wait_for_frame(probe, master, extend)
            if any(frame[1:5] != b"\x00" * 4 or frame[9:13] != b"\x00" * 4
                   for frame in collect_frames(probe, master, 0.08)):
                raise AssertionError("chassis motion overlapped the pusher command")

            wait_for_status(probe, "status", "AUTO_STOPPED_TIMEOUT", timeout=1.0)
            wait_for_frame(probe, master, zero)

            retract = expected_frame(vy=b"\xFF" * 4)
            publish_pusher(probe, "RETRACT")
            wait_for_status(probe, "status", "HOST_COMMAND_ACCEPTED_RETRACT")
            wait_for_frame(probe, master, retract)
            publish_pusher(probe, "EXTEND")
            wait_for_status(probe, "status", "REJECTED_STOP_INTERLOCK")
            wait_for_frame(probe, master, zero)
            publish_pusher(probe, "EXTEND")
            wait_for_status(probe, "status", "HOST_COMMAND_ACCEPTED_EXTEND")
            wait_for_frame(probe, master, extend)
            publish_pusher(probe, "STOP")
            wait_for_status(probe, "status", "STOPPED_HOST_COMMAND")
            wait_for_frame(probe, master, zero)

            publish_pusher(probe, "EXTEND ")
            wait_for_status(probe, "status", "REJECTED_INVALID_SAFE_STOP")
            wait_for_frame(probe, master, zero)

            publish_motion(probe, 0.05, 0.0)
            wait_for_frame(probe, master, expected_frame(0.05))
            publish_motion(probe, float("nan"), 0.0)
            wait_for_frame(probe, master, zero)
            publish_motion(probe, 0.7, 0.0)
            if expected_frame(0.7) in collect_frames(probe, master, 0.12):
                raise AssertionError("out-of-range /cmd_vel was transmitted")
            publish_motion(probe, 0.05, 0.0)
            wait_for_frame(probe, master, expected_frame(0.05))
            publish_unsupported_lateral_motion(probe)
            if expected_frame(0.05) in collect_frames(probe, master, 0.12):
                raise AssertionError("unsupported lateral /cmd_vel was transmitted")
            wait_for_frame(probe, master, zero)

            publish_pusher(probe, "EXTEND")
            wait_for_status(probe, "status", "HOST_COMMAND_ACCEPTED_EXTEND")
            wait_for_frame(probe, master, extend)
            os.close(master)
            master = -1
            wait_for_status(probe, "link_status", "DISCONNECTED", timeout=2.0)

            replacement_master = create_pty_link(serial_link)
            wait_for_status(probe, "link_status", "HOST_WRITE_OK", timeout=3.0)
            wait_for_frame(probe, replacement_master, zero, timeout=2.0)
            recent = collect_frames(probe, replacement_master, 0.12)
            if any(frame[5:9] in (b"\xEE" * 4, b"\xFF" * 4) for frame in recent):
                raise AssertionError("a pusher command resumed after serial reconnection")

            publish_pusher(probe, "RETRACT")
            wait_for_status(probe, "status", "HOST_COMMAND_ACCEPTED_RETRACT")
            wait_for_frame(probe, replacement_master, retract)
            publish_pusher(probe, "STOP")
            wait_for_frame(probe, replacement_master, zero)

            print("PASS: real ROS bridge node, isolated ROS domain, PTY-only serial peer")
            print("PASS: byte-exact motion, stop, extend (EE), retract (FF), explicit and timeout stop")
            print("PASS: velocity/pusher arbitration, reversal interlock, invalid command, NaN/range rejection, disconnect reset/reconnect")
            print("LIMIT: host-to-PTY only; no STM32 acknowledgement or physical actuator validation")
        finally:
            probe.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2)
            if process.returncode not in (0, -signal.SIGINT):
                output = process.stdout.read() if process.stdout is not None else ""
                raise RuntimeError(f"bridge process failed ({process.returncode}):\n{output}")
            if master >= 0:
                os.close(master)
            if replacement_master is not None:
                os.close(replacement_master)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--executable", default=os.environ.get("CHASSIS_BRIDGE_EXECUTABLE"))
    args = parser.parse_args()
    if not args.executable:
        parser.error("pass --executable or set CHASSIS_BRIDGE_EXECUTABLE")
    os.environ["ROS_DOMAIN_ID"] = "231"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    rclpy.init(args=None)
    try:
        verify_disabled_by_default(args.executable)
        run(args.executable)
    except Exception as exc:  # print a compact integration failure to CI logs
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
