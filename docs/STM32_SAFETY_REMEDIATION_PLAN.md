# STM32 safety remediation plan

This plan follows the P9-B static audit. It is a design proposal only; this branch does not change motor control behavior or authorize a hardware test.

## Gate 0: physical safety path

1. Hardware owner documents the actual power tree, motor-driver enable pins, contactor/brake behavior, physical E-stop wiring, reset behavior, and safe state on MCU brownout/reset.
2. Make the physical E-stop remove actuator energy independently of ROS, USB/UART, MCU firmware, and CAN.
3. Change chassis startup so it remains disabled until a deliberate, fresh enable edge and E-stop permissive; remove unconditional boot enable.
4. Define and measure acceptable stopping time and residual motion for chassis and arm using an approved isolated setup. Keep wheels lifted and the arm mechanically supported for the first supervised checks.

**Exit evidence:** reviewed wiring, power-off continuity test, controlled disable test, measured stop trace, and a signed hardware-owner record. Software state alone is not exit evidence.

## Gate 1: firmware command lease and state machine

Implement the same explicit states on both controllers: `BOOT_SAFE`, `DISARMED`, `ARMED_IDLE`, `ACTIVE`, `FAULT_LATCHED`. A valid frame is necessary but never sufficient to arm. Entry to `ACTIVE` requires a fresh controller heartbeat, enabled safety chain, fresh required feedback, and a deliberate operator action. Any lease expiry, invalid sequence, parser overrun, motor fault, stale feedback, bus-off, or watchdog reset transitions to a latched safe state. Re-arm requires neutral inputs, fault clearance and a new arm edge; never restore an old target automatically.

- Arm: start with a short MCU-side lease (initial proposal 100 ms to match host command cadence), measured from a fully validated and accepted frame. At expiry freeze the safe reference, send the verified motor disable/hold command, latch the reason, and require explicit reset plus new target. RC loss should use its 100 ms online threshold for gating, not a 3 s delay.
- Chassis: use the 50 Hz host stream with a lease materially shorter than 200 ms (initial proposal 100 ms); on expiry immediately publish disabled zero and latch serial loss. Keep CH8 and physical E-stop as independent permissives.
- Watchdog: configure IWDG with a bounded service point tied to successful control-loop completion and healthy sensor/feedback state. Never feed it from an unrelated periodic task.

Thresholds above are starting design values, not hardware-validated settings. Set them only after timing measurements and drive/controller documentation review.

## Gate 2: protocol revision

Introduce a versioned protocol (retain explicit legacy mode only if required for staged migration):

- Header, protocol version, message type, payload length, monotonic boot/session ID, monotonic sequence, payload, CRC-32C, trailer.
- Include the controller boot ID in feedback and bind each accepted arm command to task ID/epoch/sequence. Reject replay, duplicate, stale, out-of-order, wrong-session, non-finite and out-of-range values.
- Return an explicit acknowledgement only after the safety state machine has accepted/rejected the command. A stop acknowledgement must be distinct from measured stopped confirmation.
- Add measured velocities, per-motor fault bits, per-feedback age, CAN/UART/USB error counters, lease remaining, and latched fault to telemetry.
- Use bounded parser buffers, exact length checks, CRC, resynchronization, queue overflow policy, and tests for all split/concatenated/corrupted/replayed cases.

Do not silently reinterpret the existing 14/46/35-byte formats. Host and MCU must negotiate or be upgraded together under a controlled, rollback-capable release.

## Gate 3: motor/feedback fault handling

- Add per-wheel/per-joint last-valid-feedback timestamps; treat stale feedback as a latched fault.
- Decode and act on motor error/temperature states; define severity thresholds with the motor manufacturer.
- Propagate CAN transmit results; handle bus-off/recovery explicitly and require all required motors healthy before enable.
- On a fault, disable the whole coordinated group and publish its reason. Do not permit unaffected axes to continue moving if that creates a coupled hazard.
- Confirm disable semantics in the drive manual; measure physical response. A CAN disable frame is only a request until verified.

## Gate 4: offline and hardware acceptance

Before code review: build both firmware targets with reproducible, pinned tools; run native protocol/parser fault tests; run ROS P4f/P7 PTY suites; add tests for stale lease, CRC bit flip, replay/sequence reset, queue overflow, malformed RC input, each individual motor stale/fault/TX failure, and simultaneous multi-axis safe stop. Keep test artifacts keyed to source SHA and test range.

Only after Gates 0–3 are reviewed may an independent, approved hardware test plan proceed. Start with motor power isolated, then one axis/wheel under support, physical E-stop reachable, low current/speed limits, and an observer. Never use this offline test target to open real devices or actuate motors.
