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

    def test_target_topic_is_explicitly_forwarded_without_changing_legacy_default(self):
        launch = (ROOT / 'Nav2_rea_ws/src/p6_autonomous_mission/launch/'
                  'p6_autonomous_mission.launch.py').read_text()
        self.assertIn("DeclareLaunchArgument('target_topic', default_value='/p2_offline/pose_estimator/object_pose')", launch)
        self.assertIn("DeclareLaunchArgument('target_x', default_value='10.0')", launch)
        self.assertIn("'target_x': ParameterValue(LaunchConfiguration('target_x'), value_type=float)", launch)
        self.assertIn("'target_topic': LaunchConfiguration('target_topic')", launch)
        self.assertIn("'target_timeout_sec': ParameterValue(", launch)

    def test_nav_failure_event_contains_raw_odom_and_goal_evidence(self):
        source = (ROOT / 'Nav2_rea_ws/src/p6_autonomous_mission/'
                  'p6_autonomous_mission/mission_coordinator.py').read_text()
        self.assertIn("'action_status': action_status", source)
        self.assertIn("'odom_frame_id': self._odom.header.frame_id", source)
        self.assertIn("'odom_x_m': pose.position.x", source)
        self.assertIn("'odom_y_m': pose.position.y", source)
        self.assertIn("'goal_x_m': goal_x", source)
        self.assertIn("'goal_y_m': goal_y", source)

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

    def test_navigation_goals_use_latest_tf_for_recovery(self):
        source = (ROOT / 'Nav2_rea_ws/src/p6_autonomous_mission/'
                  'p6_autonomous_mission/mission_coordinator.py').read_text()
        self.assertIn('Keep its stamp at zero', source)
        self.assertNotIn('goal.pose.header.stamp = self.get_clock().now().to_msg()', source)

    def test_offline_dwb_uses_obstacle_avoidance_weight(self):
        config = (ROOT / 'Nav2_rea_ws/src/p6_autonomous_mission/config/'
                  'nav2_offline.yaml').read_text()
        fixture = (ROOT / 'tools/perception/p6_yolo_pose_synthetic.launch.py').read_text()
        self.assertIn('BaseObstacle.scale: 0.2', config)
        self.assertEqual(config.count('inflation_radius: 0.60'), 2)
        self.assertIn('width: 6', config)
        self.assertIn('height: 6', config)
        self.assertIn("'target_x': '10.5'", fixture)


if __name__ == '__main__':
    unittest.main()
