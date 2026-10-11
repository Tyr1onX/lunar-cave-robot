import unittest

from p6_autonomous_mission.mission_logic import MissionModel, MissionState


class MissionLogicTest(unittest.TestCase):
    def test_only_confirmed_full_sequence_can_succeed(self):
        mission = MissionModel()
        self.assertTrue(mission.accept_trigger())
        path = [
            MissionState.NAVIGATING_TO_TARGET,
            MissionState.CONFIRMING_TARGET_STOP,
            MissionState.DETECTING_TARGET,
            MissionState.ARM_PICK,
            MissionState.NAVIGATING_HOME,
            MissionState.CONFIRMING_HOME_STOP,
            MissionState.ARM_UNLOAD,
            MissionState.SUCCEEDED,
        ]
        for state in path:
            mission.transition(state)
        self.assertEqual(mission.state, MissionState.SUCCEEDED)
        self.assertIn(MissionState.ARM_PICK, mission.history)
        self.assertIn(MissionState.ARM_UNLOAD, mission.history)

    def test_duplicate_trigger_is_ignored(self):
        mission = MissionModel()
        self.assertTrue(mission.accept_trigger())
        self.assertFalse(mission.accept_trigger())
        self.assertEqual(mission.history.count(MissionState.WAITING_FOR_STACK), 1)

    def test_navigation_cannot_jump_directly_to_pick(self):
        mission = MissionModel()
        mission.accept_trigger()
        with self.assertRaises(ValueError):
            mission.transition(MissionState.ARM_PICK)

    def test_failure_is_terminal_and_never_success(self):
        mission = MissionModel()
        mission.accept_trigger()
        self.assertTrue(mission.transition(MissionState.ABORTED))
        self.assertFalse(mission.transition(MissionState.ABORTED))
        self.assertFalse(mission.accept_trigger())
        self.assertEqual(mission.state, MissionState.ABORTED)

    def test_arm_failure_cannot_continue_to_return_or_unload(self):
        mission = MissionModel()
        mission.accept_trigger()
        mission.transition(MissionState.NAVIGATING_TO_TARGET)
        mission.transition(MissionState.CONFIRMING_TARGET_STOP)
        mission.transition(MissionState.DETECTING_TARGET)
        mission.transition(MissionState.ARM_PICK)
        mission.transition(MissionState.ABORTED)
        with self.assertRaises(ValueError):
            mission.transition(MissionState.NAVIGATING_HOME)


if __name__ == '__main__':
    unittest.main()
