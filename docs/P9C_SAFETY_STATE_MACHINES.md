# P9-C Offline Safety State Machines

This document describes the source-level policy added on branch `p9c/offline-stability-stm32-safety`. It is not a safety certification, physical stop test, or permission to energize a robot.

## Chassis STM32

The pure C policy is in `firmware/chassis/User/APP/chassis_safety.c` and is called by the FreeRTOS control task.

| State | Meaning | Exit condition |
| --- | --- | --- |
| `BOOT_LOCKED` | Startup is disarmed; no drive-enable command is sent. | A fresh, neutral disabled-selector step enters `DISARMED`. |
| `DISARMED` | Motion command gate is closed. | After the disabled-neutral step, select SBUS with centered controls, or select serial mode with a fresh neutral 14-byte command. |
| `ARMED` | The selected source may publish commands through the gate. | Receiver loss, selected-mode change, serial command lease expiry, or UART error enters `FAULT_LATCHED`. |
| `FAULT_LATCHED` | Motion remains gated. | Move CH8 to disabled with neutral controls, then deliberately select a source and satisfy its neutral-input condition. |

Startup no longer broadcasts motor enable. The chassis task only sends enable frames after it reads a fresh enabled command. The existing ROS-to-MCU 14-byte UART frame is unchanged. Invalid UART receive state is reset and counted before DMA reception is restarted.

## Arm STM32

The pure C policy is in `firmware/arm/others/src/app/arm_safety_state.c`; the integration gates host target adoption and the existing control thread.

| State | Meaning | Exit condition |
| --- | --- | --- |
| `BOOT_LOCKED` | Arm thread is held and all seven motor channels receive disable commands. | Fresh joint feedback, online RC, and a deliberate manual re-arm gesture. |
| `ARMED` | Only the selected control source is accepted. Host control also requires a current accepted command lease. | RC loss, host disable, stale host lease, or control-source change enters `FAULT_LATCHED`. |
| `FAULT_LATCHED` | Targets are not adopted; control thread is held and motors are sent disable commands while feedback polling continues. | Repeat the manual gesture with neutral sticks. Host mode then also requires a fresh disabled frame followed by a fresh enabled command. |

The manual gesture is: hold S1 and S2 down with neutral sticks, then move S1 to middle while keeping S2 down for manual control or moving S2 up for host control. The 46-byte command and 35-byte status formats are unchanged. The host command lease is 100 ms.

`ArmHold()` sends the motor manufacturer's disable command. The hardware effect of that command is not established by these tests. An arm may fall or move under gravity when torque is removed. A hardware owner must approve a supported load, brake/counterbalance behavior, and recovery sequence before powered validation.

## Offline evidence and limits

- Chassis host tests compile the production parser/command path with a mocked HAL and exercise the pure C safety transitions.
- Arm host tests exercise the pure C safety transitions only. The product integration cannot be built because the XRobot-generated `firmware/arm/Modules/CMakeLists.txt` and `firmware/arm/User/xrobot_main.hpp` are missing.
- No real serial port, STM32, CAN bus, motor, or robot was opened, programmed, or moved.
- The existing command protocols have no end-to-end physical stop acknowledgement. A host write, software `ARMED`/`FAULT_LATCHED` state, disabled CAN command, or ROS cancellation is not proof that an actuator stopped.

## Required hardware-owner review before actuator tests

1. Confirm whether each motor's disable/relax command causes torque hold, coast, brake, or another state under the actual load.
2. Confirm an independent physical E-stop and power isolation path; software stop behavior must not stand in for it.
3. Define acceptable chassis stopping distance and arm support/recovery behavior, including loss of power and MCU reset.
4. Verify drive feedback, CAN/UART/USB loss, reset, and re-arm behavior on a mechanically supported, power-limited bench before any motion test.
5. Identify the exact firmware build and controller-installed version before any approved deployment.
