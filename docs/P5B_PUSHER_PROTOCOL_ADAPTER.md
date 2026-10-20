# Chassis pusher serial adapter

## Protocol interpretation

This change uses the protocol fields confirmed for this task:

- 115200 baud, 8 data bits, no parity, 1 stop bit.
- A 14-byte frame: `A5 | VX[4] | VY[4] | VZ[4] | 4A`.
- `VX` and `VZ` remain little-endian IEEE-754 `float32` values.
- Normal differential-drive commands use four zero bytes in `VY`.
- Raw `VY` bytes `FF FF FF FF` request pusher retract; `EE EE EE EE` request pusher extend; `00 00 00 00` requests stop.

The command tokens are copied into the frame as four bytes. They are not converted to floating-point values. The `FF` sequence has a NaN bit pattern if interpreted as IEEE-754; that interpretation is not used by the host adapter.

## ROS host interface and arbitration

The existing `chassis_serial_bridge` remains the only process that opens the chassis serial device. Pusher commands use a separate `std_msgs/msg/String` input at `/chassis/pusher_command`; accepted values are exactly `EXTEND`, `RETRACT`, and `STOP`. Host-side status is published on `/chassis/pusher_status`. Status values describe host command handling and do not acknowledge the physical actuator.

Pusher command handling is disabled by default with `pusher_commands_enabled: false`. A test or reviewed deployment can opt in with an explicit parameter. The default lease is 500 ms, bounded to 1–2000 ms. The bridge sends the pusher token periodically while the lease is fresh, then sends a zero-VY stop frame. It clears command state on serial disconnect/reconnect. A nonzero `/cmd_vel` is rejected while a pusher command or its stop interlock is active; a fresh motion command is required after stopping. Chassis motion is forced to zero in all pusher frames.

The existing `/cmd_vel` finite-value checks, unsupported-axis checks, speed limits, 100 ms freshness timeout, 20 ms default transmit period, reconnect loop, and single serial owner are retained. Invalid input clears pending host commands. A direction reversal requires a stop frame before a new pusher direction is accepted.

## Byte examples

These examples follow the field ordering above and are regression-test fixtures. They are not asserted to be copied from an unavailable protocol attachment.

| Case | 14-byte frame |
| --- | --- |
| VX `0.05`, VY zero, VZ `0.25` | `A5 CD CC 4C 3D 00 00 00 00 00 00 80 3E 4A` |
| chassis and pusher stop | `A5 00 00 00 00 00 00 00 00 00 00 00 00 4A` |
| pusher extend, chassis stopped | `A5 00 00 00 00 EE EE EE EE 00 00 00 00 4A` |
| pusher retract, chassis stopped | `A5 00 00 00 00 FF FF FF FF 00 00 00 00 4A` |

## Firmware compatibility boundary

The repository firmware source at this revision still parses all three payload fields as finite floats and treats `VX≈0`, `VZ≈0`, `VY≈1` as the legacy heading-lock command. It has no parser for the pusher tokens. Consequently, the old source rejects the `FF` token as a non-finite float and treats the finite `EE` bit pattern as an ignored VY value. The repository firmware cannot execute the new pusher commands as written.

This task does not change or flash STM32 firmware. The NUC was not reachable during the read-only check, and the physical controller's flashed firmware identity is not established. Repository contents and host-to-PTY tests do not prove the NUC or controller is updated. The pusher adapter must remain disabled on deployed systems until the installed controller firmware is independently verified as implementing the byte tokens and its stop/timeout behavior is approved. No deployment or hardware test is covered by this change.

## Validation boundary

`tools/p5b_offline/test_chassis_pusher_bridge_pty.py` builds and starts the actual ROS bridge against a symlinked pseudo-terminal in an isolated ROS domain. It checks full 14-byte frames, speed command freshness, explicit/automatic pusher stops, input rejection, command arbitration, disconnect reset, and reconnect. The pseudo-terminal is not a serial controller and provides no motor, actuator, firmware, or physical stop evidence.
