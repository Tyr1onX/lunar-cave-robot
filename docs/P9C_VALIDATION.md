# P9-C Validation Record

Date: 2026-10-13. Work branch: `p9c/offline-stability-stm32-safety`, stacked on open PR #6 branch `p9b/stm32-build-and-safety-audit`. P9-C code revision under validation: `2af38430b23691a6cf9f12f695b0293000bb24e0`.

All ROS integration runs used ThinkBook offline worktrees, `ROS_LOCALHOST_ONLY=1`, and runtime-created PTYs. No controller serial interface, STM32, motor, or robot was connected. The test runner does not start a physical device. Complete-suite status below is recorded only when all 12 cases finish in the same run.

## Changes exercised

- P7 now waits up to 30 seconds for `map -> base_link` TF to resolve, waits for fault/mission event subscribers to be discovered before publishing, assigns unique result directories, and isolates each scenario in a fresh ROS domain. Existing outcome and wire-level assertions are unchanged.
- P6 `NavigateToPose` map goals retain a zero timestamp so recovery behaviors query the latest `map -> odom -> base_link` transform after long-running actions.
- Chassis and arm source-level safety policies add startup inhibit, fault latching, and deliberate re-arm. The existing ROS command frame layouts are unchanged.

## P7 full-suite attempts before the latest TF change

| Run | Result | Failure evidence |
| --- | --- | --- |
| `20261012T115616Z-581543` (`/tmp/p9c-p7-run1`) | 8/12 cases passed; aggregate failed at `driver_restart`. | Nav2 target goal aborted twice before `FAULT_INJECTION`; DWB reported empty local plans/progress failures. The required fault-not-injected assertion failed. |
| `20261012T122953Z-592461` (`/tmp/p9c-p7-run2`) | Failed in the first `normal` case during arm pick. | P6 planner reported `Unknown planning group:` with an empty group after earlier requests logged group `arm`. No source parameter mutation was found; isolated normal case passed. Root cause remains unresolved. |
| `20261012T123813Z-595244` (`/tmp/p9c-p7-run3`) | 7/12 cases passed; aggregate failed at `arm_disconnect`. | Nav2 controller repeatedly reported `Resulting plan has 0 poses in it`, then aborted the target route twice. The arm disconnect fault was correctly not injected. |

An isolated `driver_restart` case passed at `/tmp/p9c-driver-repro/20261012T122728Z-591023/driver_restart`; isolated `normal` passed at `/tmp/p9c-normal-repro/20261012T123522Z-593887/normal`. An isolated pre-fix `arm_disconnect` case passed at `/tmp/p9c-arm-disconnect/20261012T125250Z-arm-disconnect-repro`.

The route failures show that TF readiness and domain isolation alone do not make P7 deterministic. The P6 latest-TF change addresses stale transform timestamps seen in Nav2 recovery logs. It does not change target positions, navigation tolerances, retry count, task outcomes, or assertions. Latest isolated and full-run results follow.

## Latest P7 validation

The latest isolated `arm_disconnect` case ran on the P6 latest-TF source and passed at `/tmp/p9c-arm-disconnect/20261012T155458Z-arm-disconnect-latest-tf`. Nav2 reached the target; the injected disconnect occurred in `ARM_PICK`; P6 aborted and confirmed upstream ROS cancellation. The Nav2 log for that run contained no TF extrapolation error.

The ThinkBook full P7 suite completed all 12 cases in one run at `/tmp/p9c-final-p7/20261012T160101Z-604751`, ROS domains 32–43. Aggregate: `P7_E2E_SCENARIOS=PASS`, 12/12. The P7 scripts retained their original case assertions.

| Scenario | Result state | Elapsed (s) | Result |
| --- | --- | ---: | --- |
| `normal` | `SUCCEEDED` | 106.124 | PASS; target and home Nav2 actions, stop/pose checks, arm pick and unload confirmed |
| `nav_failure` | `ABORTED` | 73.822 | PASS; navigation failure/retry ended in the required abort |
| `target_loss` | `ABORTED` | 51.353 | PASS |
| `grasp_failure` | `ABORTED` | 37.830 | PASS; cancel and abort confirmed |
| `communication_loss` | `ABORTED` | 41.649 | PASS; arm result drop/cancel path confirmed |
| `duplicate_trigger` | `SUCCEEDED` | 95.533 | PASS; duplicate triggers were ignored |
| `arm_feedback_stop` | `ABORTED` | 36.982 | PASS; feedback fault injected in `ARM_PICK`, cancel confirmed |
| `arm_disconnect` | `ABORTED` | 39.606 | PASS; disconnect injected in `ARM_PICK`, cancel confirmed |
| `driver_restart` | `ABORTED` | 40.477 | PASS; restart fault injected and remained disarmed |
| `chassis_disconnect` | `ABORTED` | 4.025 | PASS; disconnect detected in 0.003 s |
| `chassis_disconnect_return` | `ABORTED` | 50.019 | PASS; after confirmed pick, disconnect detected in 0.022 s |
| `chassis_bridge_stop` | `ABORTED` | 3.489 | PASS; stale host-status detected in 0.529 s |

A second full run is required before calling the suite repeat-stable. The earlier empty-local-path failures and one arm planner `Unknown planning group:` failure remain recorded above; the latter has not been reproduced or explained.

## Firmware policy tests and build availability

Chassis host tests compile the production UART parser/command code with mocked HAL and exercise boot, neutral re-arm, RC loss, UART error, serial lease timeout, source changes, and recovery. Arm host tests exercise the pure C safety policy for boot lock, explicit physical re-arm, host disable, lease expiry, source loss, RC loss, feedback readiness, and recovery. These tests do not run the complete product firmware image.

P9-B PR #6 previously recorded a full chassis cross-build using ARM GNU GCC 10.3.1 at source `732f78545d1701a89870699827cb4e55b0ac9d8c`. In the current P9-C ThinkBook session, the firmware builder could not find `arm-none-eabi-gcc`; therefore the P9-C chassis integration changes have no new cross-build result. The arm image remains blocked by missing generated `firmware/arm/Modules/CMakeLists.txt` and `firmware/arm/User/xrobot_main.hpp`; no generated file was fabricated.

No firmware was flashed. Tests of the policy do not prove motor stop, arm holding, physical E-stop, or any behavior under powered hardware. See [P9-C safety state machines](P9C_SAFETY_STATE_MACHINES.md) and [STM32 safety audit](STM32_SAFETY_AUDIT.md).

## Remaining S0 / unresolved items

- Physical semantics of chassis CAN relax/disable and arm motor disable are not verified; loss of arm torque may cause the arm to fall.
- Neither STM32 path has end-to-end measured-stop acknowledgement or an independently verified physical E-stop path.
- The P7 full-suite route-following and arm-planner unknown-group intermittent failures require more investigation if the latest aggregate remains incomplete or fails.
- The arm product integration cannot be compiled until the authentic XRobot project configuration/generated files are recovered.
- Real hardware review, controller build identity, and no-power bring-up remain outstanding.
