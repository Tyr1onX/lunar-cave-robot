#!/usr/bin/env python3
"""Fault-inject the real rm_serial_driver ROS 2 node through a PTY pair.

The peer emulates only the byte protocol. It is not STM32 or motor validation.
Run on ThinkBook with ROS 2 Humble and the isolated P4e overlay sourced.
"""
from __future__ import annotations

import math
import os
import pty
import select
import signal
import statistics
import struct
import subprocess
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8, String
from std_srvs.srv import Trigger


JOINTS = ["J1_joint", "J2_joint", "J3_joint", "J4_joint", "J5_joint", "J6_joint"]
ROS_TO_SERIAL = [-0.22473, -3.46819, -1.62393, -2.7583, -0.7566, -1.46542]
ROS_TO_SERIAL_SIGN = [1.0, 1.0, -1.0, 1.0, 1.0, 1.0]
LOWER = [-1.5708, -1.1345, -2.618, -2.8798, -1.578, -2.8798]
UPPER = [3.6652, 1.1345, 2.618, 2.8798, 1.5708, 2.8798]
FEEDBACK_SERIAL_ZERO = [-0.22473, -3.46819, -1.62393, -2.7583, -0.7566, -1.46542]


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AssertionError(message)


def parse_tx_frames(raw: bytes) -> tuple[list[bytes], bytes]:
    frames: list[bytes] = []
    offset = 0
    while offset + 46 <= len(raw):
        if raw[offset] != 0xA5:
            offset += 1
            continue
        candidate = raw[offset:offset + 46]
        if candidate[13] == 0x4A and candidate[14] == 0xB5 and candidate[45] == 0x4B:
            frames.append(candidate)
            offset += 46
        else:
            offset += 1
    return frames, raw[offset:]


def make_feedback(sequence: int) -> bytes:
    # 35-byte status variant: 5A, version=1, 4A, B5, valid, seq, 6x f32, gripper, 4B.
    return (
        bytes([0x5A, 1, 0x4A, 0xB5, 1])
        + struct.pack("<I6fB", sequence, *FEEDBACK_SERIAL_ZERO, 0)
        + bytes([0x4B])
    )


class PtyFirmwarePeer:
    def __init__(self, link_path: Path):
        self.link_path = link_path
        self.lock = threading.Lock()
        self.channels: list[dict] = []
        self.frames: list[tuple[float, int, bytes]] = []
        self.stop_event = threading.Event()
        self.sequence = 0
        self.reset_sequence_pending = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.add_channel(make_link=True)
        self.thread.start()

    def add_channel(self, make_link: bool = False) -> tuple[int, str]:
        master, slave = pty.openpty()
        path = os.ttyname(slave)
        os.set_blocking(master, False)
        os.close(slave)
        channel = {"master": master, "path": path, "rx": bytearray(), "feedback": True, "closed": False}
        self.channels.append(channel)
        if make_link:
            self._retarget_link(path)
        return len(self.channels) - 1, path

    def _retarget_link(self, target: str) -> None:
        temporary = self.link_path.with_name(self.link_path.name + ".next")
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
        os.symlink(target, temporary)
        os.replace(temporary, self.link_path)

    def switch_channel(self, old_index: int) -> int:
        new_index, new_path = self.add_channel()
        self._retarget_link(new_path)
        old = self.channels[old_index]
        old["closed"] = True
        try:
            os.close(old["master"])
        except OSError:
            pass
        return new_index

    def set_feedback(self, channel_index: int, enabled: bool) -> None:
        self.channels[channel_index]["feedback"] = enabled

    def reset_feedback_sequence(self) -> None:
        with self.lock:
            self.reset_sequence_pending = True

    def snapshot(self, channel_index: int | None = None) -> list[tuple[float, int, bytes]]:
        with self.lock:
            if channel_index is None:
                return list(self.frames)
            return [row for row in self.frames if row[1] == channel_index]

    def _run(self) -> None:
        next_feedback = [0.0] * 8
        while not self.stop_event.is_set():
            now = time.monotonic()
            for index, channel in enumerate(list(self.channels)):
                if channel["closed"]:
                    continue
                master = channel["master"]
                try:
                    ready, _, _ = select.select([master], [], [], 0)
                    if ready:
                        data = os.read(master, 65536)
                        channel["rx"].extend(data)
                        parsed, remainder = parse_tx_frames(bytes(channel["rx"]))
                        channel["rx"] = bytearray(remainder)
                        if parsed:
                            with self.lock:
                                self.frames.extend((time.monotonic(), index, frame) for frame in parsed)
                except (BlockingIOError, OSError):
                    pass
                if channel["feedback"] and now >= next_feedback[index]:
                    with self.lock:
                        if self.reset_sequence_pending:
                            self.sequence = 0
                            self.reset_sequence_pending = False
                        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
                        feedback = make_feedback(self.sequence)
                    try:
                        os.write(master, feedback)
                    except (BlockingIOError, OSError):
                        pass
                    next_feedback[index] = now + 0.02
            time.sleep(0.001)

    def close(self) -> None:
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        for channel in self.channels:
            if not channel["closed"]:
                try:
                    os.close(channel["master"])
                except OSError:
                    pass
        try:
            self.link_path.unlink()
        except FileNotFoundError:
            pass


class CommandStream:
    def __init__(self, test_node: Node, publisher, positions: list[float], period_sec: float,
                 task_id: str, epoch: int):
        self.publisher = publisher
        self.test_node = test_node
        self.positions = positions
        self.task_id = task_id
        self.epoch = epoch
        self.sequence = 0
        self.count = 0
        self.times: list[float] = []
        self.timer = test_node.create_timer(period_sec, self.publish)

    def publish(self) -> None:
        msg = JointState()
        msg.header.stamp = self.test_node.get_clock().now().to_msg()
        self.sequence += 1
        msg.header.frame_id = f"p4f:{self.task_id}:{self.epoch}:{self.sequence}"
        msg.name = list(JOINTS)
        msg.position = list(self.positions)
        self.publisher.publish(msg)
        self.count += 1
        self.times.append(time.monotonic())

    def stop(self) -> None:
        self.timer.cancel()

    def resume(self) -> None:
        self.timer.reset()


class IntegrationTest:
    def __init__(self, temp: Path):
        self.temp = temp
        self.link = temp / "tty-arm"
        self.peer = PtyFirmwarePeer(self.link)
        self.test_node = Node("p4e_arm_host_safety_test")
        self.command_pub = self.test_node.create_publisher(
            JointState, "/p4e/arm_joint_command", qos_profile_sensor_data)
        self.gripper_pub = self.test_node.create_publisher(Int8, "/p4e/gripper", 1)
        self.tagged_gripper_pub = self.test_node.create_publisher(String, "/p4e/tagged_gripper", 10)
        self.status = ""
        self.cancel_ack = ""
        self.epoch_counter = time.time_ns()
        self.last_command_context = None
        self.cancel_pub = self.test_node.create_publisher(String, "/p4e/cancel", 10)
        self.cancel_ack_sub = self.test_node.create_subscription(
            String, "/p4e/cancel_ack", self._on_cancel_ack, 10)
        self.status_sub = self.test_node.create_subscription(String, "/p4e/status", self._on_status, 10)
        self.reset_client = self.test_node.create_client(Trigger, "/p4e/reset")
        self.enable_client = self.test_node.create_client(Trigger, "/p4e/enable")
        self.node_log = open(temp / "driver.log", "w", encoding="utf-8")
        self.driver = None
        self.node_index = 0
        self.run_number = 0
        self.start_driver()

    def _on_status(self, msg: String) -> None:
        self.status = msg.data

    def _on_cancel_ack(self, msg: String) -> None:
        self.cancel_ack = msg.data

    def _new_context(self) -> tuple[str, int]:
        self.epoch_counter = max(self.epoch_counter + 1, time.time_ns())
        return f"ptytask{self.epoch_counter}", self.epoch_counter

    def start_driver(self) -> None:
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = env.get("ROS_DOMAIN_ID", "190")
        env["ROS_LOCALHOST_ONLY"] = "1"
        env["ROS2CLI_DISABLE_DAEMON"] = "1"
        params = self.temp / f"driver{self.run_number}.yaml"
        start_count = len(self.peer.snapshot(self.node_index))
        params.write_text(
            f'''/**:
  ros__parameters:
    device_name: "{self.link}"
    baud_rate: 115200
    flow_control: "none"
    parity: "none"
    stop_bits: "1"
    tzb.arm_command_topic: "/p4e/arm_joint_command"
    tzb.gripper_command_topic: "/p4e/gripper"
    tzb.trajectory_gripper_topic: "/p4e/tagged_gripper"
    tzb.arm_state_topic: "/p4e/arm_state"
    tzb.arm_safety_status_topic: "/p4e/status"
    tzb.arm_safety_reset_service: "/p4e/reset"
    tzb.arm_safety_enable_service: "/p4e/enable"
    tzb.trajectory_cancel_topic: "/p4e/cancel"
    tzb.trajectory_cancel_ack_topic: "/p4e/cancel_ack"
    tzb.trajectory_epoch_state_path: "{self.temp}/trajectory_epoch_highwater"
    tzb.arm_command_publish_hz: 100.0
    tzb.arm_command_lease_sec: 0.1
    tzb.arm_command_max_age_sec: 0.1
    tzb.arm_command_future_tolerance_sec: 0.05
    tzb.arm_feedback_timeout_sec: 0.5
    tzb.arm_rearm_quiet_sec: 0.1
    tzb.arm_safety_status_hz: 20.0
    tzb.arm_joint_names: {JOINTS}
''',
            encoding="utf-8",
        )
        self.driver = subprocess.Popen(
            ["ros2", "run", "rm_serial_driver", "node", "--ros-args", "--params-file", str(params)],
            stdout=self.node_log,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        self._wait(
            lambda: len(self.peer.snapshot(self.node_index)) >= start_count + 10,
            12.0,
            "new 46-byte PTY frames",
        )
        self._wait(
            lambda: "state=DISARMED" in self.status and "event=startup" in self.status,
            6.0,
            "fresh startup safety status",
        )

    def _spin(self, duration: float) -> None:
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            rclpy.spin_once(
                self.test_node,
                timeout_sec=min(0.02, max(0.0, deadline - time.monotonic())))

    def _wait(self, predicate, timeout: float, label: str) -> None:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.test_node, timeout_sec=0.02)
            if predicate():
                return
            if self.driver is not None and self.driver.poll() is not None:
                raise AssertionError(f"driver exited while waiting for {label}: {self.driver.returncode}")
        raise AssertionError(f"timed out waiting for {label}; status={self.status}")

    def call(self, client, label: str, expected_success: bool = True) -> str:
        self._wait(client.service_is_ready, 8.0, f"{label} service discovery")
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=6.0)
        require(future.done(), f"{label} service did not respond")
        response = future.result()
        require(response is not None, f"{label} returned no response")
        require(response.success == expected_success,
                f"{label} success={response.success}, expected={expected_success}: "
                f"{response.message}; observed_status={self.status}")
        return response.message

    def send_command(self, positions: list[float], stamp=None, names=None,
                     context=None) -> tuple[str, int, int]:
        task_id, epoch = context[:2] if context is not None else self._new_context()
        sequence = context[2] if context is not None else 1
        msg = JointState()
        msg.header.stamp = stamp if stamp is not None else self.test_node.get_clock().now().to_msg()
        msg.header.frame_id = f"p4f:{task_id}:{epoch}:{sequence}"
        msg.name = list(JOINTS if names is None else names)
        msg.position = list(positions)
        self.command_pub.publish(msg)
        self.last_command_context = (task_id, epoch, sequence)
        return self.last_command_context

    def start_stream(self, positions: list[float], rate_hz: float = 25.0) -> CommandStream:
        task_id, epoch = self._new_context()
        stream = CommandStream(self.test_node, self.command_pub, positions, 1.0 / rate_hz,
                               task_id, epoch)
        self._spin(0.18)
        return stream

    def wait_state(self, token: str, timeout: float = 3.0) -> None:
        self._wait(lambda: f"state={token}" in self.status, timeout, f"state {token}")

    def wait_new_frame(self, start_index: int, predicate, timeout: float = 2.0, channel=None) -> bytes:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.test_node, timeout_sec=0.02)
            frames = self.peer.snapshot(channel)
            for _, _, frame in frames[start_index:]:
                if predicate(frame):
                    return frame
        raise AssertionError("timed out waiting for expected command frame")

    def ensure_disarmed_and_quiet(self) -> None:
        self._wait(lambda: "state=DISARMED" in self.status or "state=FAULT_LATCHED" in self.status,
                   6.0, "safe disarmed/fault state")
        if "state=FAULT_LATCHED" in self.status:
            self._spin(0.16)
            self.call(self.reset_client, "manual reset")
            self.wait_state("DISARMED")
        else:
            self._spin(0.16)

    def ensure_fault(self) -> None:
        self._wait(lambda: "state=FAULT_LATCHED" in self.status, 3.0, "fault latch")

    def test_startup_and_periods(self) -> None:
        initial = self.peer.snapshot(self.node_index)
        require(all(frame[15] == 0 and frame[44] == 0 for _, _, frame in initial[-8:]),
                "startup must send disabled arm frames and cleared gripper")
        print("PASS startup is DISARMED; PTY receives protocol-compatible 46-byte frames with arm bit/gripper zero")

        task_id, epoch = self._new_context()
        stream = CommandStream(self.test_node, self.command_pub, [0.05] * 6, 0.01, task_id, epoch)
        end = time.monotonic() + 5.0
        while time.monotonic() < end:
            rclpy.spin_once(self.test_node, timeout_sec=0.01)
        stream.stop()
        intervals_ms = [
            (b - a) * 1000.0 for a, b in zip(stream.times, stream.times[1:])
        ]
        require(len(intervals_ms) > 300, f"ROS timer probe emitted too few samples: {len(intervals_ms)}")
        ordered = sorted(intervals_ms)
        p99 = ordered[min(len(ordered) - 1, math.ceil(0.99 * len(ordered)) - 1)]
        max_interval = max(intervals_ms)
        tx_times = [row[0] for row in self.peer.snapshot(self.node_index)]
        tx_intervals = [(b - a) * 1000.0 for a, b in zip(tx_times, tx_times[1:])]
        tx_sorted = sorted(tx_intervals)
        tx_p99 = tx_sorted[min(len(tx_sorted) - 1, math.ceil(0.99 * len(tx_sorted)) - 1)]
        print(
            "MEASURE ros_publish_timer_100Hz "
            f"n={len(intervals_ms)} median_ms={statistics.median(intervals_ms):.3f} "
            f"p99_ms={p99:.3f} max_ms={max_interval:.3f}; "
            f"driver_tx_frames={len(tx_intervals)} tx_median_ms={statistics.median(tx_intervals):.3f} "
            f"tx_p99_ms={tx_p99:.3f} tx_max_ms={max(tx_intervals):.3f}"
        )
        self._spin(0.16)
        require(all(frame[15] == 0 for _, _, frame in self.peer.snapshot(self.node_index)[-8:]),
                "pre-enable messages must not arm or stage a target")

    def test_valid_conversion_and_timeout(self) -> None:
        self.call(self.enable_client, "initial manual enable")
        self.wait_state("ARMED_WAIT_COMMAND")
        mark = len(self.peer.snapshot(self.node_index))
        target = [0.10] * 6
        stream = self.start_stream(target)
        self._wait(lambda: any(row[2][15] == 1 for row in self.peer.snapshot(self.node_index)[mark:]),
                   3.0, "valid command activation")
        self._spin(0.30)
        active = [row[2] for row in self.peer.snapshot(self.node_index)[mark:] if row[2][15] == 1]
        require(len(active) >= 5, "fresh stream did not produce repeated active frames")
        values = struct.unpack("<6f", active[-1][20:44])
        expected = [ROS_TO_SERIAL_SIGN[i] * target[i] + ROS_TO_SERIAL[i] for i in range(6)]
        require(all(abs(values[i] - expected[i]) < 0.003 for i in range(6)),
                f"wire joint conversion mismatch: got {values}, expected {expected}")
        stream.stop()
        gripper_sequence = stream.sequence + 100
        self.tagged_gripper_pub.publish(String(
            data=f"{stream.task_id}|{stream.epoch}|{gripper_sequence}|-1"))
        self._wait(lambda: f"trajectory_sequence={gripper_sequence}" in self.status,
                   2.0, "task-scoped initial gripper command")
        self.send_command(target, context=(stream.task_id, stream.epoch, gripper_sequence + 1))
        self._wait(lambda: any(row[2][44] == 0xFF and row[2][15] == 1
                              for row in self.peer.snapshot(self.node_index)[mark:]),
                   2.0, "active tagged gripper command")
        print("PASS fresh six-joint target and task-scoped gripper map to the 46-byte frame")

        self.ensure_fault()
        self._spin(0.15)
        after_command = [row[2] for row in self.peer.snapshot(self.node_index)[mark:]]
        first_active = next(
            (index for index, frame in enumerate(after_command) if frame[15] == 1), None)
        require(first_active is not None, "no active command frame was observed")
        first_disabled = next(
            (index for index, frame in enumerate(after_command)
             if index > first_active and frame[15] == 0), None)
        require(first_disabled is not None, "command lease expiry did not emit a disabled frame")
        after_revoke = after_command[first_disabled:]
        require(len(after_revoke) >= 8 and
                all(frame[15] == 0 and frame[44] == 0 for frame in after_revoke),
                f"driver resumed a stale target after lease revocation: "
                f"flags={[(frame[15], frame[44]) for frame in after_revoke[-8:]]}")
        print("PASS upstream silence expires the command lease, latches fault, and emits no enabled replay")

    def test_invalid_commands(self) -> None:
        if "state=FAULT_LATCHED" in self.status:
            active_fault_stream = self.start_stream([0.02] * 6)
            self.call(self.reset_client, "reset while upstream active", expected_success=False)
            active_fault_stream.stop()
            self._spin(0.16)
            self.call(self.reset_client, "reset after upstream quiet")
            self.wait_state("DISARMED")
        else:
            self.ensure_disarmed_and_quiet()
        disarmed_stream = self.start_stream([0.02] * 6)
        self.call(self.enable_client, "enable while upstream active", expected_success=False)
        disarmed_stream.stop()
        self._spin(0.16)
        print("PASS reset and enable refuse while the upstream command stream is still active")

        cases = [
            ("old timestamp", [0.1] * 6, None, None, "old"),
            ("future timestamp", [0.1] * 6, None, None, "future"),
            ("NaN", [float("nan")] + [0.1] * 5, None, None, "valid"),
            ("Inf", [float("inf")] + [0.1] * 5, None, None, "valid"),
            ("out of range", [UPPER[0] + 0.1] + [0.1] * 5, None, None, "valid"),
            ("incomplete", [0.1] * 5, None, JOINTS[:5], "valid"),
        ]
        for label, positions, _, names, stamp_mode in cases:
            self.ensure_disarmed_and_quiet()
            mark = len(self.peer.snapshot(self.node_index))
            self.call(self.enable_client, f"enable before {label}")
            self.wait_state("ARMED_WAIT_COMMAND")
            if stamp_mode == "old":
                stamp = self.test_node.get_clock().now().to_msg()
                self._spin(0.02)
                stamp.sec -= 1
            elif stamp_mode == "future":
                stamp = self.test_node.get_clock().now().to_msg()
                stamp.sec += 2
            else:
                stamp = self.test_node.get_clock().now().to_msg()
            self.send_command(positions, stamp=stamp, names=names)
            self.ensure_fault()
            recent = [row[2] for row in self.peer.snapshot(self.node_index)[mark:][-8:]]
            require(recent and all(frame[15] == 0 for frame in recent),
                    f"{label} command was emitted as enabled motion")
            print(f"PASS {label} target rejected and fault latched")

    def test_feedback_timeout(self) -> None:
        self.ensure_disarmed_and_quiet()
        self.call(self.enable_client, "enable before feedback timeout")
        stream = self.start_stream([0.05] * 6)
        self._wait(lambda: "state=ACTIVE_HOST" in self.status, 2.0, "active before feedback loss")
        self.peer.set_feedback(self.node_index, False)
        self.ensure_fault()
        stream.stop()
        self._spin(0.16)
        self.call(self.reset_client, "reset without recent feedback", expected_success=False)
        self.peer.set_feedback(self.node_index, True)
        self._spin(0.20)
        require("state=FAULT_LATCHED" in self.status,
                "feedback recovery must not clear fault latch automatically")
        self.call(self.reset_client, "reset after feedback recovery")
        self.wait_state("DISARMED")
        print("PASS whole-frame feedback timeout faults active state; feedback return does not auto-enable")

    def test_controller_restart(self) -> None:
        self.ensure_disarmed_and_quiet()
        self.call(self.enable_client, "enable before STM32 sequence restart")
        stream = self.start_stream([0.03] * 6)
        self._wait(lambda: "state=ACTIVE_HOST" in self.status, 2.0, "active before MCU restart")
        self.peer.reset_feedback_sequence()
        self.ensure_fault()
        stream.stop()
        self._spin(0.12)
        require("STM32 feedback sequence restarted" in self.status,
                f"controller epoch restart was not diagnosed: {self.status}")
        print("PASS backward STM32 feedback sequence locks the host and requires manual recovery")

    def test_serial_reconnect(self) -> int:
        self.ensure_disarmed_and_quiet()
        self.call(self.enable_client, "enable before serial disconnect")
        stream = self.start_stream([0.08] * 6)
        self._wait(lambda: "state=ACTIVE_HOST" in self.status, 2.0, "active before disconnect")
        old_index = self.node_index
        new_index = self.peer.switch_channel(old_index)
        self._wait(lambda: len(self.peer.snapshot(new_index)) >= 8, 12.0, "reconnected PTY traffic")
        self.ensure_fault()
        stream.stop()
        self._spin(0.16)
        after_reconnect = self.peer.snapshot(new_index)
        require(after_reconnect and all(row[2][15] == 0 for row in after_reconnect[-10:]),
                "serial reconnect must not automatically resume cached movement")
        self.node_index = new_index
        print("PASS PTY disconnect/reconnect stays fault-latched with disabled frames")
        return new_index

    def test_trajectory_replay_and_cancel(self) -> None:
        self.ensure_disarmed_and_quiet()
        self.call(self.enable_client, "enable before trajectory lifecycle")
        self.wait_state("ARMED_WAIT_COMMAND")
        stream = self.start_stream([0.10] * 6, rate_hz=50.0)
        self._wait(lambda: "state=ACTIVE_HOST" in self.status, 3.0, "active trajectory stream")
        task_id, epoch = stream.task_id, stream.epoch
        mark = len(self.peer.snapshot(self.node_index))
        self.gripper_pub.publish(Int8(data=-1))
        self._wait(lambda: "rejected untagged legacy gripper" in self.status,
                   2.0, "untagged gripper replay rejection")
        deadline = time.monotonic() + 1.0
        while "duplicate/backward trajectory sequence" not in self.status and time.monotonic() < deadline:
            self.send_command([0.25] * 6, context=(task_id, epoch, 1))
            self._spin(0.01)
        latest_wire = struct.unpack("<6f", self.peer.snapshot(self.node_index)[-1][2][20:44])
        expected = [ROS_TO_SERIAL_SIGN[i] * 0.10 + ROS_TO_SERIAL[i] for i in range(6)]
        require(all(abs(a-b) < 0.003 for a,b in zip(latest_wire, expected)),
                "duplicate sequence changed the trajectory target")
        require("duplicate/backward trajectory sequence" in self.status,
                f"duplicate sequence rejection was not reported: {self.status}")
        print("PASS repeated command sequence is rejected while later monotonic samples continue")

        stream.stop()
        tagged_gripper_sequence = stream.sequence + 1
        self.tagged_gripper_pub.publish(String(
            data=f"{task_id}|{epoch}|{tagged_gripper_sequence}|1"))
        self._wait(lambda: f"trajectory_sequence={tagged_gripper_sequence}" in self.status,
                   2.0, "task-scoped gripper sequence acceptance")
        self._wait(lambda: any(row[2][44] == 1 and row[2][15] == 1
                              for row in self.peer.snapshot(self.node_index)[mark:]),
                   2.0, "tagged gripper output")
        stream.sequence = tagged_gripper_sequence
        stream.resume()
        self._spin(0.12)
        cancel = String(data=f"{task_id}|{epoch}")
        self.cancel_pub.publish(cancel)
        stream.stop()
        self._wait(lambda: f"{task_id}|{epoch}|ROS_HOST_CANCELLED_STM32_UNCONFIRMED" in self.cancel_ack,
                   2.0, "host cancellation acknowledgement")
        stopped_frame = self.wait_new_frame(mark, lambda frame: frame[15] == 0, timeout=1.0)
        require(stopped_frame[44] == 0, "cancel did not clear the host gripper target")
        print("PASS cancellation clears the host target and reports STM32 stop as unconfirmed")

        new_stream = self.start_stream([0.04] * 6, rate_hz=25.0)
        self._wait(lambda: "state=ACTIVE_HOST" in self.status, 2.0, "higher epoch task takeover")
        mismatched_cancel = String(data=f"wrongtask{new_stream.epoch}|{new_stream.epoch + 1}")
        self.cancel_pub.publish(mismatched_cancel)
        self._wait(lambda: mismatched_cancel.data + "|REJECTED_STALE_OR_MALFORMED" in self.cancel_ack,
                   2.0, "mismatched cancellation rejection")
        require("state=ACTIVE_HOST" in self.status and
                f"trajectory_epoch={new_stream.epoch}" in self.status,
                "a mismatched future cancellation disrupted the active higher epoch task")
        print("PASS mismatched future cancellation cannot clear another active task")

        self.send_command([0.28] * 6, context=(task_id, epoch, stream.sequence + 2))
        self._wait(lambda: "rejected old epoch or cancelled task" in self.status,
                   2.0, "cancelled epoch replay rejection")
        require(f"trajectory_epoch={new_stream.epoch}" in self.status,
                f"higher epoch did not own the active command gate: {self.status}")
        print("PASS higher epoch task takes over and old epoch replay is rejected")

        self.tagged_gripper_pub.publish(String(
            data=f"{task_id}|{epoch}|{tagged_gripper_sequence + 200}|-1"))
        self._wait(lambda: "rejected stale/cancelled tagged gripper" in self.status,
                   2.0, "cancelled task gripper replay rejection")
        print("PASS cancelled task gripper replay is rejected after higher epoch takeover")

        self.send_command([0.30] * 6, context=(task_id, epoch - 1, 999))
        self._wait(lambda: "rejected old epoch or cancelled task" in self.status,
                   2.0, "backward epoch rejection")
        new_stream.stop()

    def test_restart_and_manual_recovery(self, channel_index: int) -> None:
        self.ensure_disarmed_and_quiet()
        self.call(self.enable_client, "enable before restart probe")
        stream = self.start_stream([0.06] * 6)
        self._wait(lambda: "state=ACTIVE_HOST" in self.status, 2.0, "active before node restart")
        stale_context = (stream.task_id, stream.epoch, stream.sequence)
        stream.stop()
        old = self.driver
        os.killpg(old.pid, signal.SIGKILL)
        old.wait(timeout=4)
        start_index = len(self.peer.snapshot(channel_index))
        self.run_number += 1
        self.start_driver()
        self._spin(0.30)
        restarted = [row[2] for row in self.peer.snapshot(channel_index)[start_index:]]
        new_epoch = next(
            (index for index, frame in enumerate(restarted)
             if struct.unpack("<I", frame[16:20])[0] == 0), None)
        require(new_epoch is not None, "restarted driver did not begin a fresh sequence epoch")
        restarted = restarted[new_epoch:]
        require(restarted and all(frame[15] == 0 and frame[44] == 0 for frame in restarted),
                "node restart must start disarmed and must not replay old joint/gripper command")
        print("PASS abrupt ROS process restart starts DISARMED; no cached motion/gripper replay")

        self._spin(0.14)
        print(f"INFO post-restart pre-enable status: {self.status}")
        self.call(self.enable_client, "manual enable after restart")
        self._spin(0.02)
        self.send_command([0.20] * 6, context=(stale_context[0], stale_context[1], stale_context[2] + 1))
        self._spin(0.08)
        self.wait_state("ARMED_WAIT_COMMAND")
        require("epoch replay" in self.status or "trajectory_epoch_high_water" in self.status,
                f"stale pre-restart epoch was not rejected after manual recovery: {self.status}")
        mark = len(self.peer.snapshot(channel_index))
        self.send_command([0.04] * 6)
        self.wait_state("ACTIVE_HOST")
        self.wait_new_frame(mark, lambda frame: frame[15] == 1, channel=channel_index)
        print("PASS after restart only explicit enable plus a fresh command produces enabled frames")

    def run(self) -> None:
        self.test_startup_and_periods()
        self.test_valid_conversion_and_timeout()
        self.test_invalid_commands()
        self.test_feedback_timeout()
        self.test_controller_restart()
        self.test_trajectory_replay_and_cancel()
        channel = self.test_serial_reconnect()
        self.test_restart_and_manual_recovery(channel)
        print("RESULT: real ROS 2 Humble rm_serial_driver node passed PTY integration/fault injection")

    def close(self) -> None:
        if self.driver is not None:
            try:
                os.killpg(self.driver.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            if self.driver.poll() is None:
                try:
                    self.driver.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(self.driver.pid, signal.SIGKILL)
                    self.driver.wait(timeout=2)
            else:
                # ros2 run may already have exited while a child node remains in its session.
                try:
                    os.killpg(self.driver.pid, 0)
                    os.killpg(self.driver.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self.test_node.destroy_node()
        self.peer.close()
        self.node_log.close()


def main() -> int:
    domain = int(os.environ.get("ROS_DOMAIN_ID", "0"))
    require(2 <= domain <= 232, "Set ROS_DOMAIN_ID to an isolated value in 2..232")
    require(os.environ.get("ROS_LOCALHOST_ONLY", "") == "1", "Set ROS_LOCALHOST_ONLY=1")
    rclpy.init()
    test = None
    try:
        with tempfile.TemporaryDirectory(prefix="p4e-arm-pty-") as directory:
            test = IntegrationTest(Path(directory))
            test.run()
    finally:
        if test is not None:
            test.close()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

