from pathlib import Path
import ast
import unittest


ROOT = Path(__file__).resolve().parents[2]


class P6PerceptionIntegrationContractTest(unittest.TestCase):
    def test_composed_launch_is_localhost_synthetic_and_uses_real_pose_topic(self):
        launch = (ROOT / 'tools/perception/p6_yolo_pose_synthetic.launch.py').read_text()
        self.assertIn("SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')", launch)
        self.assertIn("'target_topic': '/pose_estimator/object_pose'", launch)
        self.assertIn("DeclareLaunchArgument('input_rate_hz', default_value='1.0')", launch)
        self.assertIn("'--rate-hz', LaunchConfiguration('input_rate_hz')", launch)
        self.assertIn("'--complete-mission'", (ROOT / 'tools/perception/p6_yolo_pose_integration_smoke.py').read_text())
        self.assertIn("'--require-clean-nav2'", (ROOT / 'tools/perception/p6_yolo_pose_integration_smoke.py').read_text())
        self.assertIn("'arm_backend': 'synthetic'", launch)
        self.assertIn("default_value='0.1'", launch)
        self.assertIn('p6_yolo_pose_synthetic_inputs.py', launch)
        self.assertIn('p6_synthetic_task_feedback.py', launch)
        self.assertIn("'device': 'cpu'", launch)
        self.assertNotIn("'launch_camera': 'true'", launch)

    def test_composed_launch_does_not_include_physical_drivers_or_serial_ports(self):
        launch = (ROOT / 'tools/perception/p6_yolo_pose_synthetic.launch.py').read_text()
        for forbidden in ('orbbec_camera', 'livox_ros_driver2', 'chassis_serial_bridge',
                          'rm_serial_driver', '/dev/tty', 'serial_bridge'):
            self.assertNotIn(forbidden, launch)

    def test_synthetic_input_rate_is_bounded_and_timeout_profiles_are_explicit(self):
        source = (ROOT / 'tools/perception/p6_yolo_pose_synthetic_inputs.py').read_text()
        self.assertIn("'timeout', 'recovery'", source)
        self.assertIn('0.2 <= args.rate_hz <= 2.0', source)
        self.assertIn("message.data == 'DETECTING_TARGET'", source)
        self.assertIn('self.target_detection_started is not None', source)
        self.assertIn('time.monotonic() - self.target_detection_started >= 4.0', source)
        self.assertIn("args.scenario not in ('timeout',)", source)

    def test_scenario_smoke_checks_false_positive_and_cancel_evidence(self):
        source = (ROOT / 'tools/perception/p6_yolo_pose_integration_smoke.py').read_text()
        self.assertIn("args.scenario in ('no_target', 'low_confidence') and (detections != 0 or poses != 0)", source)
        self.assertIn("'no_target': '0.1'", source)
        self.assertIn("'low_confidence': '0.5'", source)
        self.assertIn("'0.0001' if args.scenario in ('no_target', 'low_confidence')", source)
        self.assertIn('last_yolo_confidence >= 0.5 or last_yolo_publish', source)
        self.assertIn("args.scenario == 'timeout' and (detection_messages == 0 or poses != 0 or cloud_messages != 0)", source)
        self.assertIn("'stage timeout in DETECTING_TARGET' not in recent_reasons", source)
        self.assertIn("first_cloud_after_detecting_sec < 3.5", source)
        self.assertIn("'TARGET_CONFIRMED', 'SAMPLE_PICK_CONFIRMED'", source)
        self.assertIn("args.scenario in ('normal', 'recovery')", source)
        self.assertIn("'ARM_CANCEL_SENT', 'ARM_CANCEL_CONFIRMED'", source)
        self.assertIn('ROS_LOCALHOST_ONLY', source)

    def test_smoke_collects_source_stamps_and_nav_path_evidence(self):
        source = (ROOT / 'tools/perception/p6_yolo_pose_integration_smoke.py').read_text()
        self.assertIn('from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath', source)
        self.assertIn('from sensor_msgs.msg import LaserScan, PointCloud2', source)
        self.assertIn("node.create_subscription(LaserScan, '/scan', on_scan", source)
        self.assertIn("node.create_subscription(OccupancyGrid, '/local_costmap/costmap', on_costmap", source)
        self.assertIn("'/local_costmap/published_footprint'", source)
        self.assertIn("'/tf:odom->base_link'", source)
        self.assertIn('discover_path_topics', source)
        self.assertIn('node.create_subscription(NavPath, topic', source)
        self.assertIn('source_age_sec_at_receive', source)
        self.assertIn('source_stamp_hz', source)
        self.assertIn('source_stamp_advance_count', source)
        self.assertIn('repeated_source_stamp_count', source)
        self.assertIn('topic_graph_endpoints', source)
        self.assertIn("'/camera/depth/color/points'", source)
        self.assertIn("'/yolo/detections'", source)
        self.assertIn("'/pose_estimator/object_pose'", source)
        self.assertIn("'detection_messages': detection_messages", source)
        self.assertIn("'pose_count': poses", source)
        self.assertIn('runtime-observations.jsonl', source)

    def test_integration_python_files_parse_without_importing_ros(self):
        paths = [
            ROOT / 'tools/perception/p6_yolo_pose_synthetic.launch.py',
            ROOT / 'tools/perception/p6_yolo_pose_synthetic_inputs.py',
            ROOT / 'tools/perception/p6_synthetic_task_feedback.py',
            ROOT / 'tools/perception/p6_yolo_pose_integration_smoke.py',
        ]
        for path in paths:
            with self.subTest(path=path.name):
                ast.parse(path.read_text(encoding='utf-8'), filename=str(path))


if __name__ == '__main__':
    unittest.main()
