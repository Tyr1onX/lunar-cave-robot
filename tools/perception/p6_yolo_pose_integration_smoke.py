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


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=('normal', 'no_target', 'low_confidence', 'timeout', 'cancel', 'recovery'), required=True)
    parser.add_argument('--model-dir', required=True, help='Externally provided OpenVINO model bundle; not copied into the repository')
    parser.add_argument('--model-pcd', default=str(ROOT / 'tzb_ws/src/pose_estimator/config/half_block.pcd'))
    parser.add_argument('--domain-id', type=int, default=random.randint(150, 230))
    parser.add_argument('--timeout-sec', type=float, default=300.0)
    parser.add_argument('--results-dir', default='/tmp/lunar-p6-yolo-pose-integration')
    args = parser.parse_args()

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
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    from yolo_node.msg import DetectionArray

    results_root = Path(args.results_dir).expanduser().resolve()
    results_root.mkdir(parents=True, exist_ok=True)
    run_dir = Path(tempfile.mkdtemp(prefix=args.scenario + '-', dir=results_root))
    event_path = run_dir / 'mission-events.jsonl'
    launch_log_path = run_dir / 'combined-launch.log'
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
    recent_reasons: list[str] = []
    log = open(launch_log_path, 'w', encoding='utf-8')
    child = subprocess.Popen(
        command, cwd=ROOT, env=environment, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True, text=True)
    node = None
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
            name = record.get('event')
            if isinstance(name, str):
                event_names.add(name)
            reason = record.get('reason')
            if isinstance(reason, str) and reason:
                recent_reasons.append(reason)

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
            perception_task_checkpoint_observed = (
                args.scenario in ('normal', 'recovery')
                and {'TARGET_CONFIRMED', 'SAMPLE_PICK_CONFIRMED'}.issubset(event_names)
            )
            if state == expected_state or perception_task_checkpoint_observed:
                # Allow the final event and pose callbacks to arrive before assertions.
                quiet_until = time.monotonic() + 1.0
                while time.monotonic() < quiet_until:
                    rclpy.spin_once(node, timeout_sec=0.05)
                break
        if args.scenario not in ('normal', 'recovery') and state != expected_state:
            raise TimeoutError('expected P6 state %s, observed %r' % (expected_state, state))
        if args.scenario in ('normal', 'recovery') and not {
                'TARGET_CONFIRMED', 'SAMPLE_PICK_CONFIRMED'}.issubset(event_names):
            raise TimeoutError('perception-to-task checkpoint was not confirmed: %r' %
                               sorted(event_names))

        log.flush()
        timing_matches = re.findall(
            r'\[YOLO TIMING\].*?max_conf=([0-9.]+) publish=(True|False)',
            launch_log_path.read_text(encoding='utf-8', errors='replace'))
        last_yolo_confidence = float(timing_matches[-1][0]) if timing_matches else None
        last_yolo_publish = timing_matches[-1][1] == 'True' if timing_matches else None

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
        if args.scenario == 'cancel' and not {'ARM_CANCEL_SENT', 'ARM_CANCEL_CONFIRMED'}.issubset(event_names):
            raise AssertionError('simulated cancellation was not confirmed upstream: ' + repr(sorted(event_names)))

        report = {
            'scenario': args.scenario,
            'domain_id': args.domain_id,
            'ros_localhost_only': environment['ROS_LOCALHOST_ONLY'],
            'mission_state': state,
            'mission_checkpoint': (
                'SAMPLE_PICK_CONFIRMED' if args.scenario in ('normal', 'recovery') else None),
            'detection_messages': detection_messages,
            'detection_count': detections,
            'pose_count': poses,
            'cloud_count': cloud_messages,
            'first_cloud_after_detecting_sec': first_cloud_after_detecting_sec,
            'mission_events': sorted(event_names),
            'recent_reasons': recent_reasons[-5:],
            'event_log': str(event_path),
            'launch_log': str(launch_log_path),
            'model_bundle_path': str(model_dir),
            'model_bundle_copied': False,
            'yolo_conf_threshold': float(conf_threshold),
            'yolo_publish_conf_threshold': float(publish_conf_threshold),
            'last_yolo_max_confidence': last_yolo_confidence,
            'last_yolo_gate_open': last_yolo_publish,
        }
        print(json.dumps(report, indent=2, ensure_ascii=False))
        return 0
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
