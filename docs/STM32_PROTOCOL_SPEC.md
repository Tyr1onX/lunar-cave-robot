# STM32 communication protocol audit

Audit basis: repository source at the current commit; no controller was connected. This documents the host and firmware byte layouts and behavior that can be verified in code. It is not proof of the protocol currently running on a physical board.

## Six-wheel chassis: ROS 2 host to STM32

Transport in source: USART1, 115200 baud, 8 data bits, no parity, 1 stop bit. The frame is fixed length and little-endian:

| Bytes | Type | Meaning | Firmware behavior |
| --- | --- | --- | --- |
| 0 | `uint8` | `0xA5` header | Bytewise resynchronization |
| 1–4 | IEEE-754 `float32` LE | `vx`, m/s | Published to the chassis command snapshot |
| 5–8 | IEEE-754 `float32` LE | `vy`, reserved/heading-lock selector | If `vx≈0`, `vz≈0`, `vy≈1`, requests heading lock; otherwise ignored for differential drive |
| 9–12 | IEEE-754 `float32` LE | `vz`, rad/s | Published as yaw-rate command |
| 13 | `uint8` | `0x4A` trailer | Fixed trailer check |

The ROS bridge and `Chaiss_Communication.c` agree on frame length, header/trailer, field order, little-endian float representation and units. The bridge currently sends `vy=0`, so it does not request the firmware's special heading-lock mode. The bridge uses 115200 baud, sends at 50 Hz by default, and sends zero frames after its 100 ms host command lease expires. The STM32 separately considers its most recent command fresh for 200 ms in the 200 Hz chassis task.

There is no checksum, sequence number, acknowledgement, or firmware telemetry over this UART protocol. The STM32 accepts a frame with correct sentinels and finite floats; a finite payload bit error can therefore become a command. The parser's 200 ms `online` observation is not the controller command lease. An invalid or absent frame does not itself reset the shared command; the control task is responsible for expiring its timestamp.

Source: `firmware/chassis/Core/Src/usart.c`, `firmware/chassis/User/APP/Chaiss_Communication.{h,c}`, `firmware/chassis/User/APP/vehicle_config.h`, `firmware/chassis/User/APP/chassis_task.c`, `Nav2_rea_ws/src/drivers/serial/driver/src/chassis_serial_bridge_node.cpp`.

## Arm: ROS 2 host to STM32 USB CDC

The firmware parser expects a 46-byte downlink frame. Multibyte values are little-endian in the current ROS serializer and STM32 build targets:

| Bytes | Type | Meaning | Firmware behavior |
| --- | --- | --- | --- |
| 0 | `uint8` | `0xA5` | Header |
| 1–4 | `float32` LE | chassis `vx` placeholder | Parsed then discarded |
| 5–8 | `float32` LE | chassis `vy` placeholder | Parsed then discarded |
| 9–12 | `float32` LE | chassis `wz` placeholder | Parsed then discarded |
| 13–14 | two `uint8` | `0x4A 0xB5` | Sync bytes |
| 15 | `uint8` | arm enable | Nonzero is required to consider adopting a target |
| 16–19 | 4 bytes | host sequence/reserved | Sent by ROS as a sequence; STM32 does not inspect it |
| 20–43 | 6 × `float32` LE | J1–J6 absolute continuous encoder angles, rad | All six must be finite; then passed to the stream-target path when adopted |
| 44 | signed `int8` | gripper: `+1` close, `-1` open, `0` hold | Selects one of two preset gripper targets |
| 45 | `uint8` | `0x4B` | Trailer |

Firmware adoption requires both a nonzero frame enable and `Remote_IsHostControl()` (S2 up). Each structurally valid request produces a 35-byte response:

| Bytes | Type | Meaning |
| --- | --- | --- |
| 0–3 | `uint8[4]` | `0x5A 0x01 0x4A 0xB5` (header, protocol version, sync) |
| 4 | `uint8` | feedback initialization flag |
| 5–8 | `uint32` LE | firmware response sequence |
| 9–32 | 6 × `float32` | J1–J6 measured continuous encoder angles, rad |
| 33 | signed `int8` | gripper command state |
| 34 | `uint8` | `0x4B` trailer |

ROS and firmware agree on lengths, offsets, sentinels, angle representation, sequence location in the response and signed gripper values. ROS's downlink task sequence at bytes 16–19 is not consumed or echoed by STM32, so P4f task/epoch/sequence barriers are host-side only. Neither direction has a CRC. The response does not identify a completed task, confirm a stop, provide per-joint feedback ages/errors, or prove zero motor velocity.

`Host_IsOnline()` is a 500 ms observation based on receiving any USB bytes, including malformed bytes; source comments explicitly say it does not participate in hold/disable policy. Separately, the remote-control error task holds/disables arm motors when RC frames are lost for more than 3 seconds or both switches request emergency stop. Thus the ROS driver's host cancellation/fault latch is not an MCU stop acknowledgement.

Source: `firmware/arm/others/inc/app/Host.h`, `firmware/arm/others/src/app/Host.cpp`, `firmware/arm/others/src/callcallback/usb_callback.cpp`, `firmware/arm/others/src/app/Remote.c`, `firmware/arm/others/src/thread/task_error.cpp`, `tzb_ws/src/serial/driver/include/rm_serial_driver/packet.hpp`, `tzb_ws/src/serial/driver/src/node.cpp`.

## Compatibility verdict

| Link | Source-level compatibility | Important limit |
| --- | --- | --- |
| ROS chassis bridge ↔ chassis STM32 | Frame-level compatible | No CRC/ACK; host zero-frame timeout and MCU command timeout are independent; no real UART/CAN test |
| ROS arm driver ↔ arm STM32 | Frame-level compatible | MCU ignores ROS task sequence; USB loss is not an MCU hold condition; no stop/zero-velocity acknowledgement |

Do not infer deployed firmware compatibility from these source comparisons. Record controller identity and firmware build SHA only after the hardware owner reads/confirms them through an approved procedure.
