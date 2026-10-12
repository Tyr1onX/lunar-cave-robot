#!/usr/bin/env python3
"""Offline byte-level peers for the P5b 14-byte and P4e 46-byte serial protocols.

Only ptys created by this process are exposed to the ROS bridge nodes. The chassis
simulator receives a Twist only after a complete 14-byte command is decoded here.
The arm feedback stream is generated from actual 46-byte commands observed on its
PTY. This module does not emulate STM32 firmware safety or physical stopping.
"""
from __future__ import annotations

import argparse
import errno
import json
import math
import os
import pty
import select
import signal
import struct
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


BASE_SIZE = 14
ARM_TX_SIZE = 46
ARM_RX_SIZE = 35
BASE_INITIAL = (0.0, 0.0, 0.0)
ARM_SERIAL_ZERO = [-0.22473, -3.46819, -1.62393, -2.7583, -0.7566, -1.46542]


def frame_feedback(sequence: int, positions: list[float], gripper: int) -> bytes:
    return (bytes((0x5A, 1, 0x4A, 0xB5, 1))
            + struct.pack('<I6fB', sequence & 0xFFFFFFFF, *positions, gripper & 0xFF)
            + bytes((0x4B,)))


def pull_frames(buffer: bytearray, size: int, header: int, tail_index: int, tail: int):
    frames = []
    while buffer:
        try:
            start = buffer.index(header)
        except ValueError:
            buffer.clear()
            break
        if start:
            del buffer[:start]
        if len(buffer) < size:
            break
        if buffer[tail_index] != tail:
            del buffer[0]
            continue
        frames.append(bytes(buffer[:size]))
        del buffer[:size]
    return frames


class PtyStm32Simulator(Node):
    def __init__(self, runtime_dir: Path):
        super().__init__('p7_pty_stm32_simulator')
        self.runtime_dir = runtime_dir
        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.chassis_fd, self.chassis_tty = self._make_tty('chassis-device')
        self.arm_fd, self.arm_tty = self._make_tty('arm-device')
        self._make_link('chassis-device', self.chassis_tty)
        self._make_link('arm-device', self.arm_tty)
        self.ready_file = self.runtime_dir / 'pty-ready.json'
        self.ready_file.write_text(json.dumps({
            'mode': 'offline-pty-only',
            'chassis_device': str(self.runtime_dir / 'chassis-device'),
            'chassis_tty': self.chassis_tty,
            'arm_device': str(self.runtime_dir / 'arm-device'),
            'arm_tty': self.arm_tty,
            'pid': os.getpid(),
        }, sort_keys=True) + '\n', encoding='utf-8')

        self.decoded_pub = self.create_publisher(Twist, '/p7/chassis/decoded_command', 10)
        self.status_pub = self.create_publisher(String, '/p7/sim/wire_status', 10)
        self.arm_event_pub = self.create_publisher(String, '/p7/sim/arm_wire_event', 20)
        self.create_subscription(String, '/p7/sim/fault', self._on_fault, 10)

        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._base_rx = bytearray()
        self._arm_rx = bytearray()
        self._base_connected = True
        self._arm_connected = True
        self._arm_feedback_enabled = True
        self._arm_feedback_sequence = 0
        self._arm_positions = list(ARM_SERIAL_ZERO)
        self._arm_gripper = 0
        self._last_arm_sequence = None
        self._last_arm_target = tuple(self._arm_positions)
        self._stats = {
            'chassis_frames': 0, 'chassis_motion_frames': 0,
            'arm_frames': 0, 'arm_enabled_frames': 0, 'arm_disabled_frames': 0,
            'arm_feedback_frames': 0, 'arm_joint_changes': 0,
            'arm_gripper_close_frames': 0, 'arm_gripper_open_frames': 0,
            'invalid_chassis_frames': 0, 'invalid_arm_frames': 0,
            'duplicate_arm_sequences': 0, 'last_chassis_vx': 0.0,
            'last_chassis_wz': 0.0, 'last_arm_sequence': 0,
            'last_enabled_arm_sequence': 0,
        }
        self._io_thread = threading.Thread(target=self._io_loop, name='p7-pty-io', daemon=True)
        self._io_thread.start()
        self.create_timer(0.25, self._publish_status)
        self.get_logger().warning(
            'OFFLINE ONLY: PTY peers created under %s; 14B and 46B wire frames are decoded; '
            'physical serial devices are not opened.' % self.runtime_dir)

    @staticmethod
    def _make_tty(name: str) -> tuple[int, str]:
        master, slave = pty.openpty()
        tty_path = os.ttyname(slave)
        os.set_blocking(master, False)
        os.close(slave)
        return master, tty_path

    def _make_link(self, name: str, target: str) -> None:
        link = self.runtime_dir / name
        next_link = self.runtime_dir / (name + '.next')
        for candidate in (link, next_link):
            try:
                candidate.unlink()
            except FileNotFoundError:
                pass
        os.symlink(target, next_link)
        os.replace(next_link, link)

    def _on_fault(self, msg: String) -> None:
        fault = msg.data.strip()
        with self._lock:
            if fault == 'arm_feedback_stop':
                self._arm_feedback_enabled = False
                self.get_logger().error('FAULT INJECTION: arm feedback frames stopped; commands still decoded.')
            elif fault == 'arm_disconnect':
                self._arm_connected = False
                self._close_fd('arm_fd')
                self.get_logger().error('FAULT INJECTION: arm PTY peer disconnected.')
            elif fault == 'chassis_disconnect':
                self._base_connected = False
                self._close_fd('chassis_fd')
                self.get_logger().error('FAULT INJECTION: chassis PTY peer disconnected.')
            elif fault == 'arm_feedback_resume':
                if self._arm_connected:
                    self._arm_feedback_enabled = True
                    self.get_logger().warning('Test fixture resumed arm feedback; P4e remains latched until operator recovery.')
            elif fault == 'restore':
                self._restore_links()
                self.get_logger().warning('Test fixture restored new PTYs; stale commands remain discarded.')
            else:
                self.get_logger().error('Unknown P7 simulation fault: %s' % fault)

    def _close_fd(self, attribute: str) -> None:
        fd = getattr(self, attribute)
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
            setattr(self, attribute, None)

    def _restore_links(self) -> None:
        if not self._base_connected:
            self.chassis_fd, self.chassis_tty = self._make_tty('chassis-device')
            self._base_rx.clear()
            self._base_connected = True
            self._make_link('chassis-device', self.chassis_tty)
        if not self._arm_connected:
            self.arm_fd, self.arm_tty = self._make_tty('arm-device')
            self._arm_rx.clear()
            self._arm_connected = True
            self._make_link('arm-device', self.arm_tty)
            self._arm_feedback_enabled = True
            self._last_arm_sequence = None
            self._arm_feedback_sequence = 0
        self.ready_file.write_text(json.dumps({
            'mode': 'offline-pty-only', 'chassis_device': str(self.runtime_dir / 'chassis-device'),
            'chassis_tty': self.chassis_tty, 'arm_device': str(self.runtime_dir / 'arm-device'),
            'arm_tty': self.arm_tty, 'pid': os.getpid(),
        }, sort_keys=True) + '\n', encoding='utf-8')

    def _io_loop(self) -> None:
        next_arm_feedback = time.monotonic()
        while not self._stop.is_set() and rclpy.ok():
            now = time.monotonic()
            with self._lock:
                read_fds = [fd for fd, enabled in ((self.chassis_fd, self._base_connected),
                                                    (self.arm_fd, self._arm_connected)) if enabled and fd is not None]
            if read_fds:
                try:
                    ready, _, _ = select.select(read_fds, [], [], 0.005)
                except (OSError, ValueError):
                    ready = []
                for fd in ready:
                    if fd == self.chassis_fd:
                        self._read_chassis(fd)
                    elif fd == self.arm_fd:
                        self._read_arm(fd)
            if now >= next_arm_feedback:
                self._write_arm_feedback()
                next_arm_feedback = now + 0.02
            time.sleep(0.001)

    def _read_chassis(self, fd: int) -> None:
        try:
            data = os.read(fd, 65536)
        except (BlockingIOError, OSError) as exc:
            if isinstance(exc, OSError) and exc.errno not in (errno.EIO, errno.EBADF):
                self.get_logger().error('Chassis PTY read failed: %s' % exc)
            return
        self._base_rx.extend(data)
        for frame in pull_frames(self._base_rx, BASE_SIZE, 0xA5, 13, 0x4A):
            vx, vy, wz = struct.unpack('<fff', frame[1:13])
            if not all(math.isfinite(v) for v in (vx, vy, wz)) or abs(vy) > 1e-5:
                with self._lock:
                    self._stats['invalid_chassis_frames'] += 1
                continue
            msg = Twist()
            msg.linear.x = vx
            msg.angular.z = wz
            try:
                self.decoded_pub.publish(msg)
            except Exception as exc:
                if rclpy.ok():
                    self.get_logger().error('Decoded chassis publish failed: %s' % exc)
                return
            with self._lock:
                self._stats['chassis_frames'] += 1
                self._stats['last_chassis_vx'] = vx
                self._stats['last_chassis_wz'] = wz
                if abs(vx) > 0.015 or abs(wz) > 0.04:
                    self._stats['chassis_motion_frames'] += 1

    def _read_arm(self, fd: int) -> None:
        try:
            data = os.read(fd, 65536)
        except (BlockingIOError, OSError) as exc:
            if isinstance(exc, OSError) and exc.errno not in (errno.EIO, errno.EBADF):
                self.get_logger().error('Arm PTY read failed: %s' % exc)
            return
        self._arm_rx.extend(data)
        for frame in pull_frames(self._arm_rx, ARM_TX_SIZE, 0xA5, 45, 0x4B):
            if frame[13] != 0x4A or frame[14] != 0xB5:
                with self._lock:
                    self._stats['invalid_arm_frames'] += 1
                continue
            enabled = frame[15] == 1
            sequence = struct.unpack('<I', frame[16:20])[0]
            positions = list(struct.unpack('<6f', frame[20:44]))
            gripper = struct.unpack('<b', frame[44:45])[0]
            if not all(math.isfinite(v) for v in positions):
                with self._lock:
                    self._stats['invalid_arm_frames'] += 1
                continue
            with self._lock:
                if self._last_arm_sequence is not None and sequence == self._last_arm_sequence:
                    self._stats['duplicate_arm_sequences'] += 1
                self._last_arm_sequence = sequence
                self._stats['arm_frames'] += 1
                self._stats['last_arm_sequence'] = sequence
                if enabled:
                    self._stats['arm_enabled_frames'] += 1
                    self._stats['last_enabled_arm_sequence'] = sequence
                    target = tuple(positions)
                    if any(abs(a - b) > 0.0005 for a, b in zip(target, self._last_arm_target)):
                        self._stats['arm_joint_changes'] += 1
                    self._last_arm_target = target
                    # Simulated servo position follows the wire command by a bounded step.
                    for index, target_value in enumerate(target):
                        delta = target_value - self._arm_positions[index]
                        self._arm_positions[index] += max(-0.04, min(0.04, delta))
                    self._arm_gripper = gripper
                    if gripper > 0:
                        self._stats['arm_gripper_close_frames'] += 1
                    elif gripper < 0:
                        self._stats['arm_gripper_open_frames'] += 1
                else:
                    self._stats['arm_disabled_frames'] += 1
                try:
                    self.arm_event_pub.publish(String(data=json.dumps({
                        'sequence': sequence, 'enabled': enabled, 'positions_serial': positions,
                        'gripper': gripper, 'mode': 'simulated_controller_wire_response',
                    }, sort_keys=True)))
                except Exception as exc:
                    if rclpy.ok():
                        self.get_logger().error('Could not publish decoded arm frame: %s' % exc)
                    return

    def _write_arm_feedback(self) -> None:
        with self._lock:
            if not self._arm_connected or not self._arm_feedback_enabled or self.arm_fd is None:
                return
            self._arm_feedback_sequence = (self._arm_feedback_sequence + 1) & 0xFFFFFFFF
            sequence = self._arm_feedback_sequence
            positions = list(self._arm_positions)
            gripper = self._arm_gripper
            fd = self.arm_fd
        frame = frame_feedback(sequence, positions, gripper)
        try:
            os.write(fd, frame)
        except (BlockingIOError, OSError) as exc:
            if isinstance(exc, OSError) and exc.errno not in (errno.EIO, errno.EBADF):
                self.get_logger().error('Arm PTY feedback write failed: %s' % exc)
            return
        with self._lock:
            self._stats['arm_feedback_frames'] += 1

    def _publish_status(self) -> None:
        with self._lock:
            fields = dict(self._stats)
            fields.update({
                'chassis_connected': self._base_connected,
                'arm_connected': self._arm_connected,
                'arm_feedback_enabled': self._arm_feedback_enabled,
                'arm_serial_joints': ','.join('%.4f' % value for value in self._arm_positions),
                'arm_gripper': self._arm_gripper,
            })
        msg = String()
        msg.data = ' '.join('%s=%s' % (key, value) for key, value in fields.items())
        self.status_pub.publish(msg)

    def destroy_node(self):
        self._stop.set()
        self._io_thread.join(timeout=1.0)
        for attribute in ('chassis_fd', 'arm_fd'):
            self._close_fd(attribute)
        for name in ('chassis-device', 'chassis-device.next', 'arm-device', 'arm-device.next'):
            try:
                (self.runtime_dir / name).unlink()
            except FileNotFoundError:
                pass
        super().destroy_node()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--runtime-dir', required=True)
    args = parser.parse_args()
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('P7 safety guard: ROS_LOCALHOST_ONLY=1 is required')
    rclpy.init(args=None)
    node = PtyStm32Simulator(Path(args.runtime_dir))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
