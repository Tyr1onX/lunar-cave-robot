#!/usr/bin/env python3
"""Offline models of the checked ROS and STM32 source paths.

This deliberately uses only the Python standard library. It never opens a
serial device, imports ROS, connects to hardware, or claims hardware acceptance.
The modeled constants and behavior are transcribed from the source paths named
in docs/p5a/chassis-source-audit.md.
"""

from __future__ import annotations

import math
import struct
import unittest
from dataclasses import dataclass


FRAME_HEAD = 0xA5
FRAME_TAIL = 0x4A
FRAME_SIZE = 14
RX_BUFFER_SIZE = FRAME_SIZE * 2
V_CMD_MAX = 0.6
YAW_RATE_MAX = 1.5
SIDE_SPEED_MAX = 0.6
TRACK_WIDTH = 0.535
WHEEL_RADIUS = 0.248 / 2.0
G36_V_MAX = 8.0
G36_KD_MAX = 5.0
CMD_TIMEOUT_MS = 200
SBUS_TIMEOUT_MS = 100


@dataclass(frozen=True)
class Twist:
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class ChassisFrame:
    vx: float
    vy: float
    vz: float


def encode_14(vx: float, vy: float, vz: float) -> bytes:
    """Mirror the packed ROS SendPacket layout; no host-side clamping."""
    return struct.pack("<BfffB", FRAME_HEAD, vx, vy, vz, FRAME_TAIL)


def decode_14(frame: bytes) -> ChassisFrame:
    if len(frame) != FRAME_SIZE:
        raise ValueError("wrong frame size")
    head, vx, vy, vz, tail = struct.unpack("<BfffB", frame)
    if head != FRAME_HEAD or tail != FRAME_TAIL:
        raise ValueError("bad frame marker")
    if not all(math.isfinite(value) for value in (vx, vy, vz)):
        raise ValueError("non-finite field")
    return ChassisFrame(vx, vy, vz)


def ros_twist_to_nav2_wire(twist: Twist) -> bytes:
    """Mirror Nav2 rm_serial_driver::DriverNode::navCallback."""
    return encode_14(twist.linear_x, -twist.linear_y, twist.angular_z)


def ros_twist_to_arm_bridge_wire(twist: Twist) -> bytes:
    """Mirror the 14-byte prefix emitted by the tzb serial driver."""
    return encode_14(twist.linear_x, -twist.linear_y, twist.angular_z)


def encode_arm_extension(
    *, enabled: int = 1, sequence: int = 0,
    joint_angles: tuple[float, ...] = (0.0,) * 6,
    gripper: int = 1,
) -> bytes:
    if len(joint_angles) != 6:
        raise ValueError("six arm joints required")
    # B5, enabled byte, uint32 sequence, six float32 positions, int8 gripper, 4B
    return struct.pack("<BBI6fbB", 0xB5, enabled, sequence,
                       *joint_angles, gripper, 0x4B)


class FirmwareStreamParser:
    """Offline transcription of Chaiss_Communication.c stream parsing."""

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.valid_count = 0
        self.invalid_count = 0

    def feed(self, data: bytes) -> list[ChassisFrame]:
        frames: list[ChassisFrame] = []
        for byte in data:
            if len(self.buffer) == RX_BUFFER_SIZE:
                del self.buffer[0]
                self.invalid_count += 1
            self.buffer.append(byte)
            while self.buffer:
                if self.buffer[0] != FRAME_HEAD:
                    del self.buffer[0]
                    self.invalid_count += 1
                    continue
                if len(self.buffer) < FRAME_SIZE:
                    break
                if self.buffer[FRAME_SIZE - 1] != FRAME_TAIL:
                    del self.buffer[0]
                    self.invalid_count += 1
                    continue
                values = struct.unpack("<fff", self.buffer[1:13])
                if all(math.isfinite(value) for value in values):
                    frames.append(ChassisFrame(*values))
                    self.valid_count += 1
                else:
                    self.invalid_count += 1
                del self.buffer[:FRAME_SIZE]
        return frames


def firmware_mode_from_ch8(pwm: int) -> str:
    if pwm < 1300:
        return "disabled"
    if pwm > 1700:
        return "sbus"
    return "serial"


def sbus_online(frame_age_ms: int) -> bool:
    return frame_age_ms <= SBUS_TIMEOUT_MS


def serial_command_fresh(command_age_ms: int) -> bool:
    return command_age_ms <= CMD_TIMEOUT_MS


def firmware_limit(vx: float, vy: float, vz: float) -> tuple[str, float, float]:
    if (abs(vx) <= 1e-4 and abs(vz) <= 1e-4 and
            abs(vy - 1.0) <= 1e-4):
        return "heading_lock", 0.0, 0.0
    return (
        "velocity",
        max(-V_CMD_MAX, min(V_CMD_MAX, vx)),
        max(-YAW_RATE_MAX, min(YAW_RATE_MAX, vz)),
    )


def six_wheel_setpoints(vx: float, yaw_rate: float) -> tuple[float, ...]:
    """Mirror kinematics_setpoints_6wd after the controller's yaw request."""
    turn_out = -0.5 * TRACK_WIDTH * yaw_rate
    left = max(-SIDE_SPEED_MAX, min(SIDE_SPEED_MAX, vx + turn_out))
    right = max(-SIDE_SPEED_MAX, min(SIDE_SPEED_MAX, vx - turn_out))
    motor_limit = G36_V_MAX * 0.95
    values = [left / WHEEL_RADIUS] * 3 + [right / WHEEL_RADIUS * -1.0] * 3
    return tuple(max(-motor_limit, min(motor_limit, speed)) for speed in values)


def fake_velocity_transform(twist: Twist, yaw: float, *, spin_enabled: bool,
                            configured_spin_speed: float = 0.0) -> Twist:
    """Mirror fake_vel_transform.cpp, including its angular-z override."""
    x = twist.linear_x * math.cos(yaw) + twist.linear_y * math.sin(yaw)
    y = -twist.linear_x * math.sin(yaw) + twist.linear_y * math.cos(yaw)
    angular = configured_spin_speed if spin_enabled else 0.0
    return Twist(x, y, angular)


class HeldCommandBridge:
    """Mirror the Nav2 serial node's cached Twist + 4 ms transmit timer."""

    def __init__(self) -> None:
        self.last_wire = encode_14(0.0, 0.0, 0.0)

    def receive_twist(self, twist: Twist) -> None:
        self.last_wire = ros_twist_to_nav2_wire(twist)

    def timer_tick(self) -> bytes:
        return self.last_wire


def hm_float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    """Transcribe hm_float_to_uint's range clamp and truncating conversion."""
    value = max(low, min(high, value))
    return int((value - low) * ((1 << bits) - 1) / (high - low))


def hm_uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    return value * (high - low) / ((1 << bits) - 1) + low


class ChassisProtocolTests(unittest.TestCase):
    def assertFrameAlmostEqual(self, actual: ChassisFrame,
                               expected: ChassisFrame) -> None:
        for actual_value, expected_value in zip(
                (actual.vx, actual.vy, actual.vz),
                (expected.vx, expected.vy, expected.vz)):
            self.assertAlmostEqual(actual_value, expected_value, places=6)

    def test_documented_forward_frame_is_exact_little_endian(self) -> None:
        self.assertEqual(
            encode_14(0.05, 0.0, 0.0).hex(" ").upper(),
            "A5 CD CC 4C 3D 00 00 00 00 00 00 00 00 4A",
        )

    def test_forward_reverse_turns_and_stop_round_trip(self) -> None:
        cases = [
            (Twist(0.2, 0.0, 0.0), (0.2, 0.0, 0.0)),
            (Twist(-0.2, 0.0, 0.0), (-0.2, 0.0, 0.0)),
            (Twist(0.0, 0.0, 0.5), (0.0, 0.0, 0.5)),
            (Twist(0.0, 0.0, -0.5), (0.0, 0.0, -0.5)),
            (Twist(), (0.0, 0.0, 0.0)),
        ]
        for twist, expected in cases:
            with self.subTest(twist=twist):
                frame = ros_twist_to_nav2_wire(twist)
                self.assertEqual(len(frame), 14)
                decoded = decode_14(frame)
                for actual, value in zip((decoded.vx, decoded.vy, decoded.vz),
                                         expected):
                    self.assertAlmostEqual(actual, value, places=6)

    def test_parser_reassembles_fragmented_and_back_to_back_frames(self) -> None:
        parser = FirmwareStreamParser()
        first = encode_14(0.1, 0.0, 0.0)
        second = encode_14(-0.2, 0.0, 0.4)
        self.assertEqual(parser.feed(b"noise" + first[:4]), [])
        got = parser.feed(first[4:] + second)
        self.assertEqual(len(got), 2)
        self.assertFrameAlmostEqual(got[0], ChassisFrame(0.1, 0.0, 0.0))
        self.assertFrameAlmostEqual(got[1], ChassisFrame(-0.2, 0.0, 0.4))
        self.assertEqual(parser.valid_count, 2)

    def test_parser_discards_bad_markers_nan_and_infinity(self) -> None:
        parser = FirmwareStreamParser()
        bad_head = bytes([0x00]) + encode_14(0.1, 0.0, 0.0)[1:]
        bad_tail = bytearray(encode_14(0.1, 0.0, 0.0))
        bad_tail[-1] = 0
        nan_frame = encode_14(math.nan, 0.0, 0.0)
        inf_frame = encode_14(0.0, 0.0, math.inf)
        good = encode_14(0.3, 0.0, -0.2)
        frames = parser.feed(bad_head + bytes(bad_tail) + nan_frame + inf_frame + good)
        self.assertEqual(len(frames), 1)
        self.assertFrameAlmostEqual(frames[0], ChassisFrame(0.3, 0.0, -0.2))
        self.assertGreaterEqual(parser.invalid_count, 4)

    def test_ros_driver_does_not_filter_nonfinite_but_firmware_rejects(self) -> None:
        parser = FirmwareStreamParser()
        for twist in (Twist(math.nan, 0.0, 0.0), Twist(0.0, 0.0, math.inf)):
            wire = ros_twist_to_nav2_wire(twist)
            self.assertEqual(len(wire), 14)
            self.assertEqual(parser.feed(wire), [])
        self.assertEqual(parser.valid_count, 0)

    def test_finite_out_of_range_commands_are_clamped_by_firmware(self) -> None:
        wire = ros_twist_to_nav2_wire(Twist(4.0, 0.0, -9.0))
        parsed = decode_14(wire)
        self.assertEqual(firmware_limit(parsed.vx, parsed.vy, parsed.vz),
                         ("velocity", 0.6, -1.5))

    def test_vy_one_is_heading_lock_magic_not_lateral_motion(self) -> None:
        self.assertEqual(firmware_limit(0.0, 1.0, 0.0),
                         ("heading_lock", 0.0, 0.0))
        self.assertEqual(firmware_limit(0.0, -0.5, 0.0),
                         ("velocity", 0.0, 0.0))

    def test_six_wheel_forward_reverse_and_ccw_allocation(self) -> None:
        forward = six_wheel_setpoints(0.2, 0.0)
        reverse = six_wheel_setpoints(-0.2, 0.0)
        ccw = six_wheel_setpoints(0.0, 1.0)
        self.assertTrue(all(value > 0 for value in forward[:3]))
        self.assertTrue(all(value < 0 for value in forward[3:]))
        self.assertTrue(all(value < 0 for value in reverse[:3]))
        self.assertTrue(all(value > 0 for value in reverse[3:]))
        self.assertTrue(all(value < 0 for value in ccw[:3]))
        self.assertTrue(all(value < 0 for value in ccw[3:]))
        self.assertAlmostEqual(abs(ccw[0]), 0.5 * TRACK_WIDTH / WHEEL_RADIUS)
        self.assertAlmostEqual(abs(ccw[3]), 0.5 * TRACK_WIDTH / WHEEL_RADIUS)
        self.assertEqual(len(ccw), 6)

    def test_ch8_mode_edges_and_watchdog_edges(self) -> None:
        self.assertEqual(firmware_mode_from_ch8(1299), "disabled")
        self.assertEqual(firmware_mode_from_ch8(1300), "serial")
        self.assertEqual(firmware_mode_from_ch8(1700), "serial")
        self.assertEqual(firmware_mode_from_ch8(1701), "sbus")
        self.assertTrue(serial_command_fresh(200))
        self.assertFalse(serial_command_fresh(201))
        self.assertTrue(sbus_online(100))
        self.assertFalse(sbus_online(101))

    def test_46_byte_arm_frame_starts_with_the_same_nav_prefix(self) -> None:
        twist = Twist(0.25, 0.0, -0.3)
        prefix = ros_twist_to_arm_bridge_wire(twist)
        extension = encode_arm_extension(sequence=3)
        mixed = prefix + extension
        self.assertEqual(len(prefix), 14)
        self.assertEqual(len(extension), 32)
        self.assertEqual(len(mixed), 46)
        self.assertEqual(mixed[:14], ros_twist_to_nav2_wire(twist))
        parser = FirmwareStreamParser()
        frames = parser.feed(mixed)
        self.assertEqual(len(frames), 1)
        self.assertFrameAlmostEqual(frames[0], ChassisFrame(0.25, 0.0, -0.3))

    def test_fake_transform_overrides_nav2_angular_command(self) -> None:
        transformed = fake_velocity_transform(Twist(0.2, 0.0, 0.7), 0.0,
                                              spin_enabled=False)
        self.assertEqual(transformed, Twist(0.2, 0.0, 0.0))

    def test_silent_cmd_vel_publisher_still_gets_stale_serial_frames(self) -> None:
        bridge = HeldCommandBridge()
        bridge.receive_twist(Twist(0.25, 0.0, 0.0))
        last_command = bridge.timer_tick()
        parser = FirmwareStreamParser()
        # 76 timer ticks are 304 ms at 4 ms/tick, with no new ROS callback.
        parsed = parser.feed(b"".join(bridge.timer_tick() for _ in range(76)))
        self.assertTrue(all(frame.vx == 0.25 for frame in parsed))
        self.assertGreater(len(parsed), 1)
        self.assertEqual(bridge.timer_tick(), last_command)

    def test_g36_kd_60_2_is_encoded_and_decoded_as_maximum_5(self) -> None:
        kd_word = hm_float_to_uint(60.2, 0.0, G36_KD_MAX, 12)
        self.assertEqual(kd_word, 4095)
        self.assertEqual(hm_uint_to_float(kd_word, 0.0, G36_KD_MAX, 12), 5.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
