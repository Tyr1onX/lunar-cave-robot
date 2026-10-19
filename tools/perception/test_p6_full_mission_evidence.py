import unittest

from tools.perception.p6_yolo_pose_integration_smoke import (
    collect_nav2_diagnostics,
    finalize_topic_observations,
    first_unexpected_process_exit,
    latest_yolo_gate_observation,
    update_topic_observation,
    validate_full_mission_evidence,
)


def full_mission_records():
    records = []
    for state in (
            'NAVIGATING_TO_TARGET', 'CONFIRMING_TARGET_STOP', 'DETECTING_TARGET',
            'ARM_PICK', 'NAVIGATING_HOME', 'CONFIRMING_HOME_STOP', 'ARM_UNLOAD', 'SUCCEEDED'):
        records.append({'event': 'STATE', 'state': state})
    records.extend([
        {'event': 'NAV_GOAL_SENT', 'destination': 'target'},
        {'event': 'NAV_GOAL_SENT', 'destination': 'home'},
        {'event': 'NAV_RESULT_CONFIRMED', 'status': 'SUCCEEDED'},
        {'event': 'NAV_RESULT_CONFIRMED', 'status': 'SUCCEEDED'},
        {'event': 'ARRIVAL_AND_STOP_CONFIRMED', 'destination': 'target'},
        {'event': 'ARRIVAL_AND_STOP_CONFIRMED', 'destination': 'home'},
        {'event': 'TARGET_CONFIRMED'},
        {'event': 'ARM_RESULT_RECEIVED', 'stage': 'pick', 'outcome': 'success'},
        {'event': 'SAMPLE_PICK_CONFIRMED'},
        {'event': 'ARM_RESULT_RECEIVED', 'stage': 'unload', 'outcome': 'success'},
        {'event': 'BASE_UNLOAD_CONFIRMED'},
    ])
    return records


class P6FullMissionEvidenceTest(unittest.TestCase):
    def test_requires_complete_nav_pick_return_unload_and_terminal_evidence(self):
        validate_full_mission_evidence('SUCCEEDED', full_mission_records())

    def test_pick_checkpoint_does_not_count_as_full_success(self):
        records = [record for record in full_mission_records()
                   if record.get('state') not in ('NAVIGATING_HOME', 'CONFIRMING_HOME_STOP',
                                                  'ARM_UNLOAD', 'SUCCEEDED')
                   and record.get('destination') != 'home'
                   and record.get('stage') != 'unload'
                   and record.get('event') not in ('BASE_UNLOAD_CONFIRMED',)]
        with self.assertRaises(AssertionError):
            validate_full_mission_evidence('ARM_PICK', records)

    def test_success_state_without_return_or_unload_evidence_is_rejected(self):
        records = [record for record in full_mission_records()
                   if record.get('destination') != 'home'
                   and record.get('stage') != 'unload'
                   and record.get('event') not in ('BASE_UNLOAD_CONFIRMED',)
                   and record.get('state') not in ('NAVIGATING_HOME',
                                                   'CONFIRMING_HOME_STOP', 'ARM_UNLOAD')]
        with self.assertRaises(AssertionError):
            validate_full_mission_evidence('SUCCEEDED', records)

    def test_abort_never_counts_as_success_even_with_pick_event(self):
        with self.assertRaises(AssertionError):
            validate_full_mission_evidence('ABORTED', full_mission_records())

    def test_nav2_log_counter_separates_empty_paths_and_cycle_misses(self):
        report = collect_nav2_diagnostics(
            '[controller_server] Resulting plan has 0 poses in it.\n'
            '[controller_server] Controller patience exceeded\n'
            '[controller_server] Control loop missed its desired rate\n'
            '[planner_server] Planner loop missed its desired rate\n'
            '[planner_server] Planning algorithm GridBased failed to generate a valid path\n'
            '[behavior_server] Current footprint not available\n')
        self.assertEqual(report['empty_controller_plans'], 1)
        self.assertEqual(report['controller_patience_exceeded'], 1)
        self.assertEqual(report['controller_cycle_misses'], 1)
        self.assertEqual(report['planner_cycle_misses'], 1)
        self.assertEqual(report['planner_invalid_paths'], 1)
        self.assertEqual(report['behavior_footprint_failures'], 1)
        self.assertEqual(report['controller_progress_failures'], 0)
        self.assertEqual(report['planner_start_in_lethal_space'], 0)

    def test_yolo_gate_observation_survives_failure_reports(self):
        confidence, publish = latest_yolo_gate_observation(
            '[yolo_node] [YOLO TIMING] total=95ms max_conf=0.383 publish=True\n'
            '[yolo_node] [YOLO TIMING] total=90ms max_conf=0.090 publish=False\n')
        self.assertEqual(confidence, 0.09)
        self.assertFalse(publish)
        self.assertEqual(latest_yolo_gate_observation('no timing records'), (None, None))

    def test_launch_child_exit_is_reported_before_mission_timeout(self):
        line = '[ERROR] [python3-3]: process has died [pid 123, exit code 1]'
        self.assertEqual(first_unexpected_process_exit('startup\n' + line), line)
        self.assertIsNone(first_unexpected_process_exit('P6 mission completed successfully'))

    def test_topic_evidence_preserves_stale_source_time_and_receive_gap(self):
        observations = {}
        first = update_topic_observation(
            observations, '/local_costmap/published_footprint', 10_000_000_000,
            1_000_000_000, 12_000_000_000, {'frame_id': 'odom'})
        second = update_topic_observation(
            observations, '/local_costmap/published_footprint', 10_500_000_000,
            2_500_000_000, 13_500_000_000, {'frame_id': 'odom'})
        summary = finalize_topic_observations(
            observations, now_ros_ns=13_750_000_000, now_monotonic_ns=3_000_000_000)
        self.assertEqual(first['source_age_sec_at_receive'], 2.0)
        self.assertEqual(second['source_age_sec_at_receive'], 3.0)
        self.assertEqual(second['receive_gap_sec'], 1.5)
        self.assertEqual(summary['/local_costmap/published_footprint']['count'], 2)
        self.assertEqual(summary['/local_costmap/published_footprint']['last_source_stamp_ns'],
                         10_500_000_000)
        self.assertEqual(summary['/local_costmap/published_footprint']['max_source_age_sec'], 3.0)
        self.assertEqual(summary['/local_costmap/published_footprint']['current_source_age_sec'], 3.25)
        self.assertEqual(summary['/local_costmap/published_footprint']['current_receive_silence_sec'], 0.5)

    def test_topic_evidence_marks_non_monotonic_source_time(self):
        observations = {}
        update_topic_observation(observations, '/odom', 20, 100, 20)
        update_topic_observation(observations, '/odom', 19, 200, 20)
        self.assertEqual(finalize_topic_observations(observations)['/odom'][
            'non_monotonic_source_stamps'], 1)

    def test_topic_evidence_separates_fresh_receives_from_stopped_source_stamps(self):
        observations = {}
        first = update_topic_observation(
            observations, '/local_costmap/published_footprint', 12_000_000_000,
            1_000_000_000, 12_000_000_000, {'frame_id': 'odom'})
        second = update_topic_observation(
            observations, '/local_costmap/published_footprint', 12_000_000_000,
            1_125_000_000, 12_125_000_000, {'frame_id': 'odom'})
        summary = finalize_topic_observations(
            observations, now_ros_ns=12_250_000_000, now_monotonic_ns=1_250_000_000)
        footprint = summary['/local_costmap/published_footprint']

        self.assertIsNotNone(first['received_at_utc'])
        self.assertEqual(second['receive_gap_sec'], 0.125)
        self.assertIsNone(second['source_stamp_gap_sec'])
        self.assertEqual(footprint['receive_hz'], 8.0)
        self.assertEqual(footprint['source_stamp_hz'], 0.0)
        self.assertEqual(footprint['repeated_source_stamp_count'], 1)
        self.assertEqual(footprint['observed_source_duration_sec'], 0.0)
        self.assertEqual(footprint['current_source_age_sec'], 0.25)
        self.assertEqual(footprint['current_receive_silence_sec'], 0.125)

    def test_source_stamp_rate_counts_advances_not_republished_messages(self):
        observations = {}
        for stamp_sec, receive_sec in (
                (10, 1.0), (10, 1.1), (11, 1.2), (11, 1.3), (12, 1.4)):
            update_topic_observation(
                observations, '/received_global_plan', stamp_sec * 1_000_000_000,
                int(receive_sec * 1_000_000_000), stamp_sec * 1_000_000_000)
        summary = finalize_topic_observations(observations)['/received_global_plan']
        self.assertEqual(summary['receive_hz'], 10.0)
        self.assertEqual(summary['source_stamp_advance_count'], 2)
        self.assertEqual(summary['repeated_source_stamp_count'], 2)
        self.assertEqual(summary['source_stamp_hz'], 1.0)


if __name__ == '__main__':
    unittest.main()
