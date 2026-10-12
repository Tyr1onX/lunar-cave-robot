"""Design-oracle tests for the proposed arm safety state machine.

This is a standalone policy model. It does not import or execute ROS production
code, STM32 firmware, a serial driver, CAN, or motor hardware.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
import math
import unittest


class State(Enum):
    BOOT_INHIBIT = auto()
    READY_HOLD = auto()
    ACTIVE_HOST = auto()
    ACTIVE_REMOTE = auto()
    STOPPING_HOLD = auto()
    FAULT_LATCHED = auto()


@dataclass
class ArmSafetyModel:
    command_lease_ms: int = 100
    state: State = State.BOOT_INHIBIT
    session: int = 0
    last_sequence: int = -1
    last_command_ms: int | None = None
    goal: tuple[float, ...] | None = None
    torque_action: str = "INHIBIT"
    fault: str | None = None
    events: list[str] = field(default_factory=list)

    def qualify_ready(self, feedback_fresh: bool, no_motor_errors: bool,
                      remote_online: bool, controls_neutral: bool) -> bool:
        if self.state is not State.BOOT_INHIBIT:
            return False
        if not all((feedback_fresh, no_motor_errors, remote_online, controls_neutral)):
            self.events.append("boot_checks_failed")
            return False
        self.state = State.READY_HOLD
        self.torque_action = "HOLD_POLICY_REQUIRED"
        self.events.append("ready_hold")
        return True

    def arm_host(self, operator_confirmed: bool, remote_online: bool,
                 host_switch_selected: bool) -> int | None:
        if (self.state is not State.READY_HOLD or not operator_confirmed or
                not remote_online or not host_switch_selected):
            return None
        self.session += 1
        self.last_sequence = -1
        self.last_command_ms = None
        self.goal = None
        self.state = State.ACTIVE_HOST
        self.torque_action = "HOLD_TORQUE_UNTIL_NEW_COMMAND"
        self.events.append("host_armed_new_session")
        return self.session

    def command(self, session: int, sequence: int, now_ms: int,
                positions: tuple[float, ...], bounds: tuple[tuple[float, float], ...]) -> bool:
        if self.state is not State.ACTIVE_HOST or session != self.session:
            self.events.append("command_wrong_state_or_session")
            return False
        if len(positions) != 6 or len(bounds) != 6:
            self._latch("malformed_command")
            return False
        if sequence <= self.last_sequence:
            self.events.append("replayed_sequence_rejected")
            return False
        if any(not math.isfinite(x) for x in positions):
            self._latch("non_finite_command")
            return False
        if any(x < low or x > high for x, (low, high) in zip(positions, bounds)):
            self._latch("out_of_range_command")
            return False
        self.goal = positions
        self.last_sequence = sequence
        self.last_command_ms = now_ms
        self.torque_action = "ACTIVE_CLOSED_LOOP"
        self.events.append("fresh_host_command")
        return True

    def tick(self, now_ms: int, feedback_fresh: bool = True,
             no_motor_errors: bool = True) -> None:
        if self.state not in (State.ACTIVE_HOST, State.ACTIVE_REMOTE):
            return
        if not feedback_fresh or not no_motor_errors:
            self._latch("joint_feedback_or_motor_fault")
            return
        if (self.state is State.ACTIVE_HOST and self.last_command_ms is not None and
                now_ms - self.last_command_ms >= self.command_lease_ms):
            self.goal = None
            self.session += 1  # invalidate every frame from the lost command session
            self.last_command_ms = None
            self.state = State.STOPPING_HOLD
            self.torque_action = "CONTROLLED_DECEL_KEEP_TORQUE"
            self.events.append("command_lease_expired")

    def request_remote_takeover(self, remote_online: bool,
                                controls_neutral: bool) -> bool:
        if (self.state is not State.ACTIVE_HOST or not remote_online or
                not controls_neutral):
            return False
        self.goal = None
        self.session += 1
        self.last_command_ms = None
        self.state = State.STOPPING_HOLD
        self.torque_action = "CONTROLLED_DECEL_KEEP_TORQUE"
        self.events.append("host_goal_invalidated_for_handover")
        return True

    def finish_controlled_stop(self) -> bool:
        if self.state is not State.STOPPING_HOLD:
            return False
        self.state = State.READY_HOLD
        self.torque_action = "HOLD_POLICY_REQUIRED"
        self.events.append("stopped_waiting_for_operator")
        return True

    def arm_remote(self, operator_confirmed: bool, remote_online: bool,
                   controls_neutral: bool) -> bool:
        if (self.state is not State.READY_HOLD or not operator_confirmed or
                not remote_online or not controls_neutral):
            return False
        self.session += 1
        self.state = State.ACTIVE_REMOTE
        self.torque_action = "REMOTE_CONTROL_ACTIVE"
        self.events.append("remote_armed")
        return True

    def manual_reset(self, operator_confirmed: bool, feedback_fresh: bool,
                     no_motor_errors: bool, remote_online: bool,
                     controls_neutral: bool) -> bool:
        if self.state is not State.FAULT_LATCHED:
            return False
        if not all((operator_confirmed, feedback_fresh, no_motor_errors,
                    remote_online, controls_neutral)):
            return False
        self.state = State.READY_HOLD
        self.goal = None
        self.last_command_ms = None
        self.torque_action = "HOLD_POLICY_REQUIRED"
        self.events.append("manual_reset_to_ready_only")
        return True

    def _latch(self, reason: str) -> None:
        self.state = State.FAULT_LATCHED
        self.session += 1
        self.goal = None
        self.last_command_ms = None
        self.fault = reason
        # This is an intent marker, not a motor command. Actual failed-axis action
        # remains a hardware/policy decision; the model forbids blanket disable.
        self.torque_action = "FAULT_LATCHED_NO_BLANKET_DISABLE"
        self.events.append(reason)


BOUNDS = ((-3.0, 3.0),) * 6  # test fixture only; not robot joint limits
POSE = (0.1, 0.2, 0.3, 0.4, 0.5, 0.6)


def active_host_model() -> tuple[ArmSafetyModel, int]:
    model = ArmSafetyModel()
    assert model.qualify_ready(True, True, True, True)
    session = model.arm_host(True, True, True)
    assert session is not None
    assert model.command(session, 1, 0, POSE, BOUNDS)
    return model, session


class ArmSafetyModelTests(unittest.TestCase):
    def test_boot_requires_all_health_and_explicit_enable(self):
        model = ArmSafetyModel()
        self.assertFalse(model.qualify_ready(False, True, True, True))
        self.assertEqual(model.state, State.BOOT_INHIBIT)
        self.assertTrue(model.qualify_ready(True, True, True, True))
        self.assertIsNone(model.arm_host(False, True, True))
        self.assertEqual(model.state, State.READY_HOLD)

    def test_command_lease_stops_and_invalidates_old_goal_without_disarm(self):
        model, session = active_host_model()
        model.tick(99)
        self.assertEqual(model.state, State.ACTIVE_HOST)
        model.tick(100)
        self.assertEqual(model.state, State.STOPPING_HOLD)
        self.assertIsNone(model.goal)
        self.assertEqual(model.torque_action, "CONTROLLED_DECEL_KEEP_TORQUE")
        self.assertFalse(model.command(session, 2, 101, POSE, BOUNDS))
        self.assertTrue(model.finish_controlled_stop())
        self.assertEqual(model.state, State.READY_HOLD)

    def test_nan_inf_and_out_of_bounds_latch_and_clear_goal(self):
        for bad_value, expected_fault in ((math.nan, "non_finite_command"),
                                           (math.inf, "non_finite_command"),
                                           (4.0, "out_of_range_command")):
            model, session = active_host_model()
            bad = (bad_value,) + POSE[1:]
            self.assertFalse(model.command(session, 2, 10, bad, BOUNDS))
            self.assertEqual(model.state, State.FAULT_LATCHED)
            self.assertEqual(model.fault, expected_fault)
            self.assertIsNone(model.goal)
            self.assertEqual(model.torque_action, "FAULT_LATCHED_NO_BLANKET_DISABLE")

    def test_feedback_fault_latches_and_reconnect_does_not_resume(self):
        model, old_session = active_host_model()
        model.tick(10, feedback_fresh=False)
        self.assertEqual(model.state, State.FAULT_LATCHED)
        self.assertFalse(model.command(old_session, 2, 11, POSE, BOUNDS))
        self.assertFalse(model.manual_reset(True, False, True, True, True))
        self.assertTrue(model.manual_reset(True, True, True, True, True))
        new_session = model.arm_host(True, True, True)
        self.assertIsNotNone(new_session)
        self.assertNotEqual(old_session, new_session)
        self.assertFalse(model.command(old_session, 99, 12, POSE, BOUNDS))
        self.assertIsNone(model.goal)

    def test_remote_takeover_requires_fresh_neutral_and_invalidates_host(self):
        model, old_session = active_host_model()
        self.assertFalse(model.request_remote_takeover(False, True))
        self.assertFalse(model.request_remote_takeover(True, False))
        self.assertEqual(model.state, State.ACTIVE_HOST)
        self.assertTrue(model.request_remote_takeover(True, True))
        self.assertIsNone(model.goal)
        self.assertFalse(model.command(old_session, 2, 20, POSE, BOUNDS))
        self.assertTrue(model.finish_controlled_stop())
        self.assertTrue(model.arm_remote(True, True, True))
        self.assertEqual(model.state, State.ACTIVE_REMOTE)

    def test_repeated_or_older_sequence_does_not_refresh_lease(self):
        model, session = active_host_model()
        accepted_at = model.last_command_ms
        self.assertFalse(model.command(session, 1, 90, POSE, BOUNDS))
        self.assertEqual(model.last_command_ms, accepted_at)
        model.tick(100)
        self.assertEqual(model.state, State.STOPPING_HOLD)


if __name__ == "__main__":
    unittest.main(verbosity=2)
