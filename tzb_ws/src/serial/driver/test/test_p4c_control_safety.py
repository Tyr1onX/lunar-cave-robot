#!/usr/bin/env python3
"""Hardware-isolated P4c fault injection against rm_serial_driver over a PTY.

Run after sourcing ROS 2 Humble and the P4b install, with an isolated ROS_DOMAIN_ID.
The serial node opens only a freshly-created /dev/pts pseudo-terminal.
"""
from __future__ import annotations

import os
import pty
import re
import select
import signal
import struct
import subprocess
import tempfile
import threading
import time
from pathlib import Path

JOINTS = ["J1_joint", "J2_joint", "J3_joint", "J4_joint", "J5_joint", "J6_joint"]
OFFSET = [-0.22473, -3.46819, -1.62393, -2.7583, -0.7566, -1.46542]
SIGN = [1, 1, -1, 1, 1, 1]


def fail(message: str) -> None:
    raise AssertionError(message)


def publish_joint_state(positions: list[float], stamp_sec: int) -> None:
    names_yaml = "[" + ", ".join(JOINTS) + "]"
    positions_yaml = "[" + ", ".join(f"{x:.9g}" for x in positions) + "]"
    message = (
        "{header: {stamp: {sec: " + str(stamp_sec) + ", nanosec: 0}}, "
        f"name: {names_yaml}, position: {positions_yaml}}}"
    )
    subprocess.run(
        ["ros2", "topic", "pub", "--once", "/tzb_catch/arm_joint_command",
         "sensor_msgs/msg/JointState", message],
        check=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=15,
    )


def read_frames(raw: bytes) -> list[bytes]:
    frames: list[bytes] = []
    index = 0
    while index + 46 <= len(raw):
        if raw[index] != 0xA5:
            index += 1
            continue
        candidate = raw[index:index + 46]
        if candidate[13] == 0x4A and candidate[14] == 0xB5 and candidate[45] == 0x4B:
            frames.append(candidate)
            index += 46
        else:
            index += 1
    return frames


def main() -> int:
    ros_domain = os.environ.get("ROS_DOMAIN_ID", "")
    if not ros_domain or ros_domain in {"0", "1"}:
        fail("Set an isolated ROS_DOMAIN_ID in the range 2..232 before running.")
    if not shutil_which("ros2"):
        fail("ros2 is not available; source ROS 2 Humble and the P4b install first.")

    repo_override = os.environ.get("P4C_REPO")
    repo = Path(repo_override) if repo_override else Path(__file__).resolve().parents[4]
    config = repo / "src/serial/driver/config/serial_params.yaml"
    config_text = config.read_text()
    if not re.search(r"(?m)^\s*device_name:\s*", config_text):
        fail(f"No device_name field in {config}")

    master_fd, slave_fd = pty.openpty()
    pty_path = os.ttyname(slave_fd)
    if not pty_path.startswith("/dev/pts/"):
        fail(f"Refusing non-PTY serial path: {pty_path}")
    os.set_blocking(master_fd, False)
    captured = bytearray()
    capture_lock = threading.Lock()
    stop_reader = threading.Event()

    def reader() -> None:
        while not stop_reader.is_set():
            try:
                ready, _, _ = select.select([master_fd], [], [], 0.05)
                if ready:
                    block = os.read(master_fd, 65536)
                    if block:
                        with capture_lock:
                            captured.extend(block)
            except (BlockingIOError, OSError):
                time.sleep(0.01)

    reader_thread = threading.Thread(target=reader, daemon=True)
    node = None
    try:
        with tempfile.TemporaryDirectory(prefix="p4c-pty-") as temp_dir:
            params = Path(temp_dir) / "serial_params.yaml"
            params.write_text(re.sub(
                r'(?m)^(\s*device_name:\s*).+$',
                lambda m: m.group(1) + '"' + pty_path + '"',
                config_text,
                count=1,
            ))
            node = subprocess.Popen(
                ["ros2", "run", "rm_serial_driver", "node", "--ros-args",
                 "--params-file", str(params)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
            reader_thread.start()

            deadline = time.monotonic() + 10.0
            while time.monotonic() < deadline:
                with capture_lock:
                    frames = read_frames(bytes(captured))
                if len(frames) >= 10:
                    break
                if node.poll() is not None:
                    fail(f"serial node exited before PTY packets; status={node.returncode}")
                time.sleep(0.05)
            else:
                fail("serial node did not emit packets to the isolated PTY")

            with capture_lock:
                before = read_frames(bytes(captured))
            if not before or any(frame[15] != 0 for frame in before[-5:]):
                fail("driver was expected to keep arm_command_enabled=0 before any valid JointState")
            print("PASS: no-command startup emits disabled arm frames on PTY")

            # Deliberately stale stamp: sec=1 is far in the past. The current callback ignores it.
            stale_target = [0.20] * 6
            publish_joint_state(stale_target, stamp_sec=1)
            time.sleep(0.65)
            with capture_lock:
                after_stale = read_frames(bytes(captured))
            enabled = [frame for frame in after_stale if frame[15] == 1]
            expected = [SIGN[i] * stale_target[i] + OFFSET[i] for i in range(6)]
            if not enabled:
                fail("stale JointState was not transmitted; expected current driver behavior to accept it")
            values = struct.unpack("<6f", enabled[-1][20:44])
            if any(abs(values[i] - expected[i]) > 2e-3 for i in range(6)):
                fail(f"stale command payload mismatch: got={values}, expected={expected}")
            print("REPRODUCED: a JointState stamped at ROS time 1 s is accepted and reaches the PTY command frame")

            # A rejected command does not revoke the previously accepted command or its enable bit.
            invalid = stale_target.copy()
            invalid[0] = 100.0
            publish_joint_state(invalid, stamp_sec=1)
            time.sleep(0.25)
            with capture_lock:
                after_invalid = read_frames(bytes(captured))
            enabled = [frame for frame in after_invalid if frame[15] == 1]
            if len(enabled) < 5:
                fail("enabled packets stopped after invalid command; expected prior command to remain latched")
            stable = [struct.unpack("<6f", frame[20:44]) for frame in enabled[-5:]]
            if any(any(abs(stable[j][i] - expected[i]) > 2e-3 for i in range(6)) for j in range(5)):
                fail(f"invalid command changed or unsettled prior target: {stable[-1]}")
            seqs = [struct.unpack("<I", frame[16:20])[0] for frame in enabled[-5:]]
            if not all(seqs[i] != seqs[i - 1] for i in range(1, len(seqs))):
                fail(f"serial packets did not continue while ROS command updates were absent: {seqs}")
            print("REPRODUCED: out-of-range message is rejected, but last target remains enabled and is resent at 100 Hz")

            # Observe a quiet interval after the last message: the periodic serial target remains latched.
            time.sleep(0.30)
            with capture_lock:
                after_quiet = read_frames(bytes(captured))
            quiet_frames = [frame for frame in after_quiet if frame[15] == 1]
            if len(quiet_frames) < len(enabled) + 15:
                fail("driver did not continue publishing the accepted target during command silence")
            latest = struct.unpack("<6f", quiet_frames[-1][20:44])
            if any(abs(latest[i] - expected[i]) > 2e-3 for i in range(6)):
                fail(f"target changed during command silence: {latest}")
            print("REPRODUCED: stopping ROS command updates does not disable the serial command stream")

    finally:
        stop_reader.set()
        if node is not None and node.poll() is None:
            try:
                os.killpg(node.pid, signal.SIGINT)
                node.wait(timeout=3)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(node.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        reader_thread.join(timeout=1)
        os.close(master_fd)
        os.close(slave_fd)
    return 0


def shutil_which(name: str) -> str | None:
    for directory in os.environ.get("PATH", "").split(os.pathsep):
        candidate = Path(directory) / name
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return str(candidate)
    return None


if __name__ == "__main__":
    raise SystemExit(main())
