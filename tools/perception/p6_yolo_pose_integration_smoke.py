#!/usr/bin/env python3
"""Run one localhost-only YOLO → pose estimator → P6 synthetic scenario."""

from __future__ import annotations

import argparse
import json
import os
import random
import re
import signal
import subprocess
import sys
import tempfile
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = Path(__file__).with_name('p6_yolo_pose_synthetic.launch.py')


def validate_full_mission_evidence(state: str, records: list[dict]) -> None:
    """Require action, arrival, pick, return, unload and terminal P6 evidence."""
    if state != 'SUCCEEDED':
        raise AssertionError('full mission must end in SUCCEEDED, observed %r' % state)

    state_sequence = [record.get('state') for record in records
                      if record.get('event') == 'STATE']
    required_sequence = (
        'NAVIGATING_TO_TARGET', 'CONFIRMING_TARGET_STOP', 'DETECTING_TARGET',
        'ARM_PICK', 'NAVIGATING_HOME', 'CONFIRMING_HOME_STOP', 'ARM_UNLOAD', 'SUCCEEDED',
    )
    cursor = 0
    for observed in state_sequence:
        if cursor < len(required_sequence) and observed == required_sequence[cursor]:
            cursor += 1
    if cursor != len(required_sequence):
        raise AssertionError('P6 state sequence incomplete: %r' % state_sequence)

    goals = {record.get('destination') for record in records
             if record.get('event') == 'NAV_GOAL_SENT'}
    arrivals = {record.get('destination') for record in records
                if record.get('event') == 'ARRIVAL_AND_STOP_CONFIRMED'}
    nav_successes = [record for record in records
                     if record.get('event') == 'NAV_RESULT_CONFIRMED'
                     and record.get('status') == 'SUCCEEDED']
    arm_successes = {(record.get('stage'), record.get('outcome')) for record in records
                     if record.get('event') == 'ARM_RESULT_RECEIVED'}
    event_names = {record.get('event') for record in records}
    if goals < {'target', 'home'} or arrivals < {'target', 'home'}:
        raise AssertionError('target/home goals and stop confirmations are incomplete')
    if len(nav_successes) < 2:
        raise AssertionError('both NavigateToPose actions must report SUCCEEDED')
    if not {('pick', 'success'), ('unload', 'success')} <= arm_successes:
        raise AssertionError('matching successful pick and unload results are required')
    if not {'TARGET_CONFIRMED', 'SAMPLE_PICK_CONFIRMED',
            'BASE_UNLOAD_CONFIRMED'} <= event_names:
        raise AssertionError('target, pick and unload evidence is incomplete')
    if 'MISSION_ABORTED' in event_names:
        raise AssertionError('aborted mission cannot be reported as a completed mission')


def collect_nav2_diagnostics(log_text: str) -> dict[str, int]:
    patterns = {
        'empty_controller_plans': r'Resulting plan has 0 poses in it',
        'controller_patience_exceeded': r'Controller patience exceeded',
        'controller_progress_failures': r'Failed to make progress',
        'controller_cycle_misses': r'Control loop missed its desired rate',
        'planner_cycle_misses': r'Planner loop missed its desired rate',
        'planner_invalid_paths': r'Planning algorithm .* failed to generate a valid path',
        'planner_start_in_lethal_space': r'Starting point in lethal space',
        'tf_extrapolation_errors': r'Extrapolation Error',
        'behavior_footprint_failures': r'Current footprint not available|Pose Goes Off Grid',
        'behavior_recovery_failures': r'backup failed|spin failed|DriveOnHeading failed',
    }
    return {name: len(re.findall(pattern, log_text, flags=re.IGNORECASE))
            for name, pattern in patterns.items()}


def first_unexpected_process_exit(log_text: str) -> str | None:
    """Return a launch child-exit line before teardown has begun, if present."""
    for line in log_text.splitlines():
        if 'process has died' in line:
            return line.strip()
    return None


def update_topic_observation(
        observations: dict, topic: str, source_stamp_ns: int,
        received_monotonic_ns: int, now_ros_ns: int, details: dict | None = None) -> dict:
    """Accumulate source-stamp freshness and receive-gap evidence for a ROS topic."""
    source_stamp_ns = int(source_stamp_ns)
    received_monotonic_ns = int(received_monotonic_ns)
    now_ros_ns = int(now_ros_ns)
    entry = observations.setdefault(topic, {
        'count': 0,
        'first_source_stamp_ns': None,
        'last_source_stamp_ns': None,
        'last_received_monotonic_ns': None,
        'max_receive_gap_sec': 0.0,
        'max_source_age_sec': 0.0,
        'non_monotonic_source_stamps': 0,
    })
    previous_received = entry['last_received_monotonic_ns']
    previous_source = entry['last_source_stamp_ns']
    gap_sec = None
    if previous_received is not None:
        gap_sec = max(0.0, (received_monotonic_ns - previous_received) / 1e9)
        entry['max_receive_gap_sec'] = max(entry['max_receive_gap_sec'], gap_sec)
    age_sec = ((now_ros_ns - source_stamp_ns) / 1e9
               if source_stamp_ns > 0 else None)
    if age_sec is not None:
        entry['max_source_age_sec'] = max(entry['max_source_age_sec'], age_sec)
        if previous_source is not None and source_stamp_ns < previous_source:
            entry['non_monotonic_source_stamps'] += 1
    if entry['first_source_stamp_ns'] is None:
        entry['first_source_stamp_ns'] = source_stamp_ns or None
    entry['count'] += 1
    entry['last_source_stamp_ns'] = source_stamp_ns or None
    entry['last_received_monotonic_ns'] = received_monotonic_ns
    row = {
        'topic': topic,
        'count': entry['count'],
        'source_stamp_ns': source_stamp_ns or None,
        'received_monotonic_ns': received_monotonic_ns,
        'source_age_sec_at_receive': age_sec,
        'receive_gap_sec': gap_sec,
    }
    if details:
        row.update(details)
    return row


def finalize_topic_observations(
        observations: dict, now_ros_ns: int | None = None,
        now_monotonic_ns: int | None = None) -> dict:
    """Return a JSON-safe summary without leaking internal monotonic timestamps."""
    summary = {}
    for topic, entry in observations.items():
        last_source_stamp_ns = entry['last_source_stamp_ns']
        last_received_monotonic_ns = entry['last_received_monotonic_ns']
        current_source_age_sec = (
            (int(now_ros_ns) - last_source_stamp_ns) / 1e9
            if now_ros_ns is not None and last_source_stamp_ns else None)
        current_receive_silence_sec = (
            max(0.0, (int(now_monotonic_ns) - last_received_monotonic_ns) / 1e9)
            if now_monotonic_ns is not None and last_received_monotonic_ns is not None else None)
        summary[topic] = {
            'count': entry['count'],
            'first_source_stamp_ns': entry['first_source_stamp_ns'],
            'last_source_stamp_ns': last_source_stamp_ns,
            'max_receive_gap_sec': round(entry['max_receive_gap_sec'], 6),
            'max_source_age_sec': round(entry['max_source_age_sec'], 6),
            'current_source_age_sec': (round(current_source_age_sec, 6)
                                       if current_source_age_sec is not None else None),
            'current_receive_silence_sec': (
                round(current_receive_silence_sec, 6)
                if current_receive_silence_sec is not None else None),
            'non_monotonic_source_stamps': entry['non_monotonic_source_stamps'],
        }
    return summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=('normal', 'no_target', 'low_confidence', 'timeout', 'cancel', 'recovery'), required=True)
    parser.add_argument('--model-dir', required=True, help='Externally provided OpenVINO model bundle; not copied into the repository')
    parser.add_argument('--model-pcd', default=str(ROOT / 'tzb_ws/src/pose_estimator/config/half_block.pcd'))
    parser.add_argument('--domain-id', type=int, default=random.randint(150, 230))
    parser.add_argument('--timeout-sec', type=float, default=300.0)
    parser.add_argument('--complete-mission', action='store_true',
                        help='wait for P6 SUCCEEDED and verify return/unload event evidence')
    parser.add_argument('--require-clean-nav2', action='store_true',
                        help='fail if empty paths, progress failures, or controller/planner cycle misses are logged')
    parser.add_argument('--rate-hz', type=float, default=1.0,
                        help='synthetic RGB-D input rate from 0.2 through 2.0 Hz')
    parser.add_argument('--results-dir', default='/tmp/lunar-p6-yolo-pose-integration')
    args = parser.parse_args()

    if args.complete_mission and args.scenario not in ('normal', 'recovery'):
        parser.error('--complete-mission is supported only for normal or recovery scenarios')
    if args.require_clean_nav2 and not args.complete_mission:
        parser.error('--require-clean-nav2 requires --complete-mission')
    if not 0.2 <= args.rate_hz <= 2.0:
        parser.error('--rate-hz must be between 0.2 and 2.0')

    if not 0 <= args.domain_id <= 232:
        parser.error('--domain-id must be between 0 and 232')
    model_dir = Path(args.model_dir).expanduser().resolve()
    model_pcd = Path(args.model_pcd).expanduser().resolve()
    for name in ('metadata.yaml', 'yoloe-26l-seg.xml', 'yoloe-26l-seg.bin'):
        if not (model_dir / name).is_file():
            parser.error('model directory is missing ' + name)
    if not model_pcd.is_file():
        parser.error('synthetic model PCD is missing: ' + str(model_pcd))

    os.environ['ROS_DOMAIN_ID'] = str(args.domain_id)
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    os.environ['ROS2CLI_DISABLE_DAEMON'] = '1'

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from geometry_msgs.msg import PolygonStamped
    from nav_msgs.msg import Odometry, Path
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage
    from yolo_node.msg import DetectionArray

    results_root = Path(args.results_dir).expanduser().resolve()
    results_root.mkdir(parents=True, exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix=args.scenario + '-', dir=results_root))
    event_path = run_dir / 'mission-events.jsonl'
    launch_log_path = run_dir / 'combined-launch.log'
    observation_path = run_dir / 'runtime-observations.jsonl'
    environment = os.environ.copy()
    conf_threshold = '0.0001' if args.scenario in ('no_target', 'low_confidence') else '0.1'
    publish_conf_threshold = {
        'no_target': '0.1',
        'low_confidence': '0.5',
    }.get(args.scenario, '0.1')
    input_scenario = args.scenario if args.scenario in (
        'no_target', 'low_confidence', 'timeout', 'recovery') else 'normal'
    mission_scenario = 'grasp_failure' if args.scenario == 'cancel' else 'normal'
    command = [
        'ros2', 'launch', str(LAUNCH),
        'domain_id:=%d' % args.domain_id,
        'mission_scenario:=' + mission_scenario,
        'input_scenario:=' + input_scenario,
        'input_rate_hz:=' + str(args.rate_hz),
        'model_path:=' + str(model_dir),
        'model_pcd:=' + str(model_pcd),
        'event_log_path:=' + str(event_path),
        'yolo_conf_threshold:=' + conf_threshold,
        'yolo_publish_conf_threshold:=' + publish_conf_threshold,
    ]

    state: str | None = None
    detections = 0
    detection_messages = 0
    poses = 0
    cloud_messages = 0
    detecting_since: float | None = None
    first_cloud_after_detecting_sec: float | None = None
    event_names: set[str] = set()
    event_records: list[dict] = []
    recent_reasons: list[str] = []
    last_launch_health_check = 0.0
    log = open(launch_log_path, 'w', encoding='utf-8')
    observation_log = open(observation_path, 'w', encoding='utf-8')
    child = subprocess.Popen(
        command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True, text=True)
    node = None
    observations = {}
    discovered_path_topics: set[str] = set()
    try:
        rclpy.init()
        node = rclpy.create_node('p6_yolo_pose_integration_smoke')

        def on_state(message):
            nonlocal state, detecting_since
            state = message.data
            if state == 'DETECTING_TARGET' and detecting_since is None:
                detecting_since = time.monotonic()

        def on_cloud(_message):
            nonlocal cloud_messages, first_cloud_after_detecting_sec
            cloud_messages += 1
            if detecting_since is not None and first_cloud_after_detecting_sec is None:
                first_cloud_after_detecting_sec = time.monotonic() - detecting_since

        def on_detection(message):
            nonlocal detections, detection_messages
            detection_messages += 1
            detections += len(message.detections)

        def on_pose(_message):
            nonlocal poses
            poses += 1

        def on_event(message):
            try:
                record = json.loads(message.data)
            except (TypeError, ValueError):
                return
            event_records.append(record)
            name = record.get('event')
            if isinstance(name, str):
                event_names.add(name)
            reason = record.get('reason')
            if isinstance(reason, str) and reason:
                recent_reasons.append(reason)

        def observe(topic, header, **details):
            stamp = header.stamp
            source_stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
            row = update_topic_observation(
                observations, topic, source_stamp_ns, time.monotonic_ns(),
                node.get_clock().now().nanoseconds, details or None)
            observation_log.write(json.dumps(row, sort_keys=True) + '\n')

        def on_odom(message):
            observe('/odom', message.header,
                    x_m=message.pose.pose.position.x,
                    y_m=message.pose.pose.position.y,
                    linear_speed_mps=message.twist.twist.linear.x,
                    angular_speed_rps=message.twist.twist.angular.z)

        def on_footprint(message):
            observe('/local_costmap/published_footprint', message.header,
                    frame_id=message.header.frame_id,
                    vertex_count=len(message.polygon.points))

        def on_tf(message):
            for transform in message.transforms:
                if (transform.header.frame_id, transform.child_frame_id) == ('odom', 'base_link'):
                    observe('/tf:odom->base_link', transform.header,
                            frame_id=transform.header.frame_id,
                            child_frame_id=transform.child_frame_id)

        def make_path_callback(topic):
            def on_path(message):
                first = message.poses[0].pose.position if message.poses else None
                last = message.poses[-1].pose.position if message.poses else None
                observe(topic, message.header,
                        frame_id=message.header.frame_id,
                        pose_count=len(message.poses),
                        first_xy_m=([first.x, first.y] if first else None),
                        last_xy_m=([last.x, last.y] if last else None))
            return on_path

        def discover_path_topics():
            for topic, types in node.get_topic_names_and_types():
                if ('nav_msgs/msg/Path' in types and
                        any(token in topic.lower() for token in ('plan', 'path')) and
                        topic not in discovered_path_topics):
                    node.create_subscription(Path, topic, make_path_callback(topic), 10)
                    discovered_path_topics.add(topic)

        node.create_subscription(Odometry, '/odom', on_odom, 10)
        node.create_subscription(PolygonStamped, '/local_costmap/published_footprint',
                                 on_footprint, 10)
        node.create_subscription(TFMessage, '/tf', on_tf, 50)
        node.create_timer(1.0, discover_path_topics)

        node.create_subscription(String, '/p6/mission_state', on_state, 10)
        node.create_subscription(String, '/p6/mission_events', on_event, 50)
        node.create_subscription(DetectionArray, '/yolo/detections', on_detection, 10)
        node.create_subscription(PoseStamped, '/pose_estimator/object_pose', on_pose, 10)
        node.create_subscription(PointCloud2, '/camera/depth/color/points', on_cloud, 10)

        expected_state = 'SUCCEEDED' if args.scenario in ('normal', 'recovery') else 'ABORTED'
        deadline = time.monotonic() + args.timeout_sec
        while time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError('combined launch exited before the P6 mission reached a terminal state')
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if now - last_launch_health_check >= 0.5:
                log.flush()
                observation_log.flush()
                unexpected_exit = first_unexpected_process_exit(
                    launch_log_path.read_text(encoding='utf-8', errors='replace'))
                if unexpected_exit:
                    raise RuntimeError('combined launch child exited unexpectedly: ' + unexpected_exit)
                last_launch_health_check = now
            perception_task_checkpoint_observed = (
                args.scenario in ('normal', 'recovery')
                and {'TARGET_CONFIRMED', 'SAMPLE_PICK_CONFIRMED'}.issubset(event_names)
            )
            if state in ('SUCCEEDED', 'ABORTED') and state != expected_state:
                raise RuntimeError(
                    'P6 reached unexpected terminal state %s; events=%s' %
                    (state, sorted(event_names)))
            checkpoint_is_terminal_for_this_run = (
                perception_task_checkpoint_observed and not args.complete_mission
            )
            if state == expected_state or checkpoint_is_terminal_for_this_run:
                # Allow the final event and pose callbacks to arrive before assertions.
                quiet_until = time.monotonic() + 1.0
                while time.monotonic() < quiet_until:
                    rclpy.spin_once(node, timeout_sec=0.05)
                break
        if (args.scenario not in ('normal', 'recovery') or args.complete_mission) and state != expected_state:
            raise TimeoutError('expected P6 state %s, observed %r' % (expected_state, state))
        if args.scenario in ('normal', 'recovery') and not {
                'TARGET_CONFIRMED', 'SAMPLE_PICK_CONFIRMED'}.issubset(event_names):
            raise TimeoutError('perception-to-task checkpoint was not confirmed: %r' %
                               sorted(event_names))

        log.flush()
        launch_log_text = launch_log_path.read_text(encoding='utf-8', errors='replace')
        timing_matches = re.findall(
            r'\[YOLO TIMING\].*?max_conf=([0-9.]+) publish=(True|False)',
            launch_log_text)
        last_yolo_confidence = float(timing_matches[-1][0]) if timing_matches else None
        last_yolo_publish = timing_matches[-1][1] == 'True' if timing_matches else None
        nav2_diagnostics = collect_nav2_diagnostics(launch_log_text)
        topic_observations = finalize_topic_observations(
            observations, node.get_clock().now().nanoseconds, time.monotonic_ns())

        if args.scenario in ('normal', 'recovery', 'cancel') and (detections == 0 or poses < 3):
            raise AssertionError('perception interface did not produce enough evidence: detections=%d poses=%d' % (detections, poses))
        if args.scenario == 'recovery' and not {
                'TARGET_CONFIRMED', 'SAMPLE_PICK_CONFIRMED'}.issubset(event_names):
            raise AssertionError('recovered depth stream did not lead to a confirmed P6 pick stage')
        if args.scenario in ('no_target', 'low_confidence') and (detections != 0 or poses != 0):
            raise AssertionError(
                'expected the YOLO publish gate to prevent detections and poses; '
                'detections=%d poses=%d' % (detections, poses))
        if (args.scenario == 'low_confidence' and
                (last_yolo_confidence is None or last_yolo_confidence >= 0.5 or last_yolo_publish)):
            raise AssertionError(
                'expected weak synthetic-target confidence below the 0.5 publish gate; '
                'max_conf=%r publish=%r' % (last_yolo_confidence, last_yolo_publish))
        if args.scenario == 'timeout' and (detection_messages == 0 or poses != 0 or cloud_messages != 0):
            raise AssertionError('timeout case did not isolate RGB/YOLO from the missing RGB-D cloud')
        if args.scenario in ('no_target', 'low_confidence', 'timeout') and (
                'stage timeout in DETECTING_TARGET' not in recent_reasons):
            raise AssertionError(
                'expected P6 to time out specifically while waiting for a confirmed target; '
                'reasons=%r' % recent_reasons)
        if args.scenario in ('normal', 'recovery', 'cancel') and cloud_messages == 0:
            raise AssertionError('expected synthetic depth pointcloud samples')
        if (args.scenario == 'recovery' and
                (first_cloud_after_detecting_sec is None or first_cloud_after_detecting_sec < 3.5)):
            raise AssertionError(
                'depth input did not recover after a visible detecting-stage gap: %r' %
                first_cloud_after_detecting_sec)
        if args.scenario == 'normal' and 'SAMPLE_PICK_CONFIRMED' not in event_names:
            raise AssertionError('normal P6 task feedback is incomplete: ' + repr(sorted(event_names)))
        if args.complete_mission:
            validate_full_mission_evidence(state, event_records)
        if args.require_clean_nav2 and any(nav2_diagnostics.values()):
            raise AssertionError('Nav2 runtime diagnostics were observed: %r' % nav2_diagnostics)
        if args.scenario == 'cancel' and not {'ARM_CANCEL_SENT', 'ARM_CANCEL_CONFIRMED'}.issubset(event_names):
            raise AssertionError('simulated cancellation was not confirmed upstream: ' + repr(sorted(event_names)))

        report = {
            'scenario': args.scenario,
            'domain_id': args.domain_id,
            'ros_localhost_only': environment['ROS_LOCALHOST_ONLY'],
            'mission_state': state,
            'validation_scope': 'full_mission' if args.complete_mission else 'scenario_checkpoint',
            'mission_checkpoint': (
                'SUCCEEDED' if args.complete_mission else
                'SAMPLE_PICK_CONFIRMED' if args.scenario in ('normal', 'recovery') else None),
            'p6_state_sequence': [record.get('state') for record in event_records
                                  if record.get('event') == 'STATE'],
            'nav_goal_destinations': [record.get('destination') for record in event_records
                                      if record.get('event') == 'NAV_GOAL_SENT'],
            'confirmed_arrivals': [record.get('destination') for record in event_records
                                   if record.get('event') == 'ARRIVAL_AND_STOP_CONFIRMED'],
            'arm_results': [
                {'stage': record.get('stage'), 'outcome': record.get('outcome')}
                for record in event_records if record.get('event') == 'ARM_RESULT_RECEIVED'
            ],
            'detection_messages': detection_messages,
            'detection_count': detections,
            'pose_count': poses,
            'cloud_count': cloud_messages,
            'first_cloud_after_detecting_sec': first_cloud_after_detecting_sec,
            'mission_events': sorted(event_names),
            'recent_reasons': recent_reasons[-5:],
            'nav2_diagnostics': nav2_diagnostics,
            'topic_observations': topic_observations,
            'path_topics_observed': sorted(discovered_path_topics),
            'runtime_observation_log': str(observation_path),
            'event_log': str(event_path),
            'launch_log': str(launch_log_path),
            'model_bundle_path': str(model_dir),
            'model_bundle_copied': False,
            'yolo_conf_threshold': float(conf_threshold),
            'yolo_publish_conf_threshold': float(publish_conf_threshold),
            'last_yolo_max_confidence': last_yolo_confidence,
            'last_yolo_gate_open': last_yolo_publish,
        }
        report_path = run_dir / 'summary.json'
        report['summary_file'] = str(report_path)
        report_path.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n',
                               encoding='utf-8')
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 0
    except Exception as exc:
        log.flush()
        observation_log.flush()
        failure_report = {
            'scenario': args.scenario,
            'domain_id': args.domain_id,
            'validation_scope': 'full_mission' if args.complete_mission else 'scenario_checkpoint',
            'mission_state': state,
            'mission_events': sorted(event_names),
            'nav2_diagnostics': collect_nav2_diagnostics(
                launch_log_path.read_text(encoding='utf-8', errors='replace')),
            'topic_observations': finalize_topic_observations(
                observations,
                node.get_clock().now().nanoseconds if node is not None else None,
                time.monotonic_ns()),
            'path_topics_observed': sorted(discovered_path_topics),
            'runtime_observation_log': str(observation_path),
            'error': str(exc),
            'event_log': str(event_path),
            'launch_log': str(launch_log_path),
        }
        report_path = run_dir / 'summary.json'
        failure_report['summary_file'] = str(report_path)
        report_path.write_text(json.dumps(failure_report, indent=2, ensure_ascii=False) + '\n',
                               encoding='utf-8')
        print(json.dumps(failure_report, indent=2, ensure_ascii=False), file=sys.stderr)
        raise
    finally:
        if child.poll() is None:
            try:
                os.killpg(child.pid, signal.SIGINT)
                child.wait(timeout=12.0)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(child.pid, signal.SIGTERM)
                    child.wait(timeout=5.0)
                except (ProcessLookupError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(child.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    child.wait()
        log.close()
        observation_log.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if child.returncode not in (0, -signal.SIGINT, 130):
            print('combined_launch_exit=%s' % child.returncode, file=sys.stderr)
            try:
                print(launch_log_path.read_text(encoding='utf-8')[-8000:], file=sys.stderr)
            except OSError:
                pass


if __name__ == '__main__':
    raise SystemExit(main())
