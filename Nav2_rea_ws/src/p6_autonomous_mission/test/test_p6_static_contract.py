from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]


class OfflineContractTest(unittest.TestCase):
    def test_launch_is_localhost_only_and_uses_stock_nav2(self):
        launch = (ROOT / 'Nav2_rea_ws/src/p6_autonomous_mission/launch/'
                  'p6_autonomous_mission.launch.py').read_text()
        self.assertIn("SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')", launch)
        self.assertIn("get_package_share_directory('nav2_bringup')", launch)
        self.assertIn("executable='cave_simulator'", launch)

    def test_offline_entry_uses_distinct_workspaces_and_never_enables_serial(self):
        runner = (ROOT / 'tools/p6_offline/run.sh').read_text()
        arm_launch = (ROOT / 'tzb_ws/src/lite_motion_planner/launch/'
                      'p3b_offline.launch.py').read_text()
        self.assertIn('source "$P6_NAV_WS/install/p6/setup.bash"', runner)
        self.assertIn('source "$P6_ARM_WS/install/p6/setup.bash"', runner)
        self.assertIn("'use_serial_driver': 'false'", arm_launch)
        self.assertIn('export ROS_LOCALHOST_ONLY=1', runner)

    def test_fault_scenarios_are_explicit(self):
        source = (ROOT / 'Nav2_rea_ws/src/p6_autonomous_mission/'
                  'p6_autonomous_mission/mission_coordinator.py').read_text()
        for scenario in ('nav_failure', 'target_loss', 'grasp_failure', 'communication_loss'):
            self.assertIn("'%s'" % scenario, source)
        self.assertIn('FAULT_LATCHED', source)
        self.assertIn("MissionState.ABORTED", source)


if __name__ == '__main__':
    unittest.main()
