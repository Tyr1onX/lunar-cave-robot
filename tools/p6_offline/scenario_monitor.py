#!/usr/bin/env python3
"""Run-time assertions for the six isolated P6 mission scenarios."""

import json
import os
import sys
import time
import uuid

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


class ScenarioMonitor(Node):
    def __init__(self, expected, scenario):
        super().__init__('p6_scenario_monitor')
        self.expected = expected
        self.scenario = scenario
        self.state = None
        self._duplicate_count = 0
        self._duplicate_timer = None
        self._arm_results = []
        self._cancel_acks = []
        self._tagged_arm_command_times = {}
        self._start_pub = self.create_publisher(Bool, '/p6/start', 10)
        self._arm_request_pub = self.create_publisher(String, '/p6/arm_task/request', 10)
        self.create_subscription(String, '/p6/arm_task/result', self._on_arm_result, 10)
        self.create_subscription(String, '/p6/arm_task/cancel_ack', self._on_cancel_ack, 10)
        self.create_subscription(JointState, '/tzb_catch/arm_joint_command', self._on_arm_command, qos_profile_sensor_data)
        self.create_subscription(String, '/p6/mission_state', self._on_state, 10)
        self.create_subscription(String, '/p6/mission_events', self._on_event, 20)
        self._duplicate_probe = self.create_timer(0.1, self._probe_duplicate_trigger)

    def _on_state(self, msg):
        self.state = msg.data

    def _probe_duplicate_trigger(self):
        if self.scenario != 'duplicate_trigger':
            self._duplicate_probe.cancel()
        elif self.count_subscribers('/p6/start') > 0:
            self._duplicate_probe.cancel()
            self._duplicate_timer = self.create_timer(0.15, self._send_duplicates)

    def _send_duplicates(self):
        if self._duplicate_count >= 3:
            self._duplicate_timer.cancel()
            return
        msg = Bool()
        msg.data = True
        self._start_pub.publish(msg)
        self._duplicate_count += 1

    def _on_arm_result(self, msg):
        self._arm_results.append(msg.data)

    def _on_cancel_ack(self, msg):
        self._cancel_acks.append(msg.data)

    def _on_arm_command(self, msg):
        fields = msg.header.frame_id.split(':')
        if len(fields) == 4 and fields[0] == 'p4f':
            key = (fields[1], fields[2])
            self._tagged_arm_command_times.setdefault(key, []).append(time.monotonic())

    def verify_cancelled_task_quiet(self, task_id, epoch, quiet_sec=0.5):
        key = (task_id, str(epoch))
        prefix = task_id + '|' + str(epoch) + '|UPSTREAM_QUIESCED'
        if not any(ack.startswith(prefix) for ack in self._cancel_acks):
            print('FAIL missing upstream cancel acknowledgement for %s/%s' % key, file=sys.stderr)
            return False
        before = len(self._tagged_arm_command_times.get(key, []))
        if before == 0:
            print('FAIL no tagged arm command reached the P4e driver input for %s/%s' % key,
                  file=sys.stderr)
            return False
        deadline = time.monotonic() + quiet_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        after = len(self._tagged_arm_command_times.get(key, []))
        if after != before:
            print('FAIL cancelled task published %d late command(s)' % (after-before), file=sys.stderr)
            return False
        print('PASS P4e input observed %d tagged arm commands; stream stayed quiet for %.2fs after acknowledgement' % (before, quiet_sec))
        return True

    def verify_new_task_takeover(self, timeout_sec=30.0):
        task_id = uuid.uuid4().hex
        epoch = time.time_ns()
        request = String()
        request.data = task_id + '|pick|' + str(epoch)
        self._arm_request_pub.publish(request)
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            for result in self._arm_results:
                fields = result.split('|')
                if len(fields) == 4 and fields[0] == task_id and fields[3] == str(epoch):
                    if fields[2] == 'success':
                        print('PASS cancelled worker accepted and completed a higher epoch follow-up task')
                        return True
                    print('FAIL follow-up task returned %s' % fields[2], file=sys.stderr)
                    return False
        print('FAIL no result for higher epoch follow-up task %s' % task_id, file=sys.stderr)
        return False

    def _on_event(self, _msg):
        # State is consumed live; authoritative event assertions use the flushed JSONL log.
        pass


def _read_events(path):
    events = []
    with open(path, 'r', encoding='utf-8') as stream:
        for line in stream:
            try:
                events.append(json.loads(line))
            except ValueError:
                continue
    return events


def _validate(scenario, expected, events):
    errors = []

    def matching(name, predicate=lambda event: True):
        return [event for event in events
                if event.get('event') == name and predicate(event)]

    def require(condition, message):
        if not condition:
            errors.append(message)

    if expected == 'ABORTED':
        terminal = matching('MISSION_ABORTED')
    else:
        terminal = matching('STATE', lambda event: event.get('state') == expected)
    terminal_state = terminal[-1].get('state') if terminal else None
    require(terminal_state == expected,
            'JSONL terminal state does not match expected %s' % expected)

    if scenario in ('normal', 'duplicate_trigger'):
        require(len(matching('NAV_RESULT_CONFIRMED',
                             lambda event: event.get('status') == 'SUCCEEDED')) >= 2,
                'both outbound and return Nav2 action results must succeed')
        require(bool(matching('TARGET_CONFIRMED')),
                'fresh target detections were not confirmed')
        require(bool(matching('SAMPLE_PICK_CONFIRMED')),
                'successful pick result was not confirmed')
        require(bool(matching('BASE_UNLOAD_CONFIRMED')),
                'successful unload result was not confirmed')
        require(bool(matching('STATE', lambda event: event.get('state') == 'SUCCEEDED')),
                'mission success transition was not recorded')

    if scenario == 'nav_failure':
        failures = matching('NAVIGATION_FAILURE')
        require(len(failures) >= 2, 'Nav2 must fail twice before mission abort')
        require(bool(matching('NAVIGATION_RETRY')),
                'navigation retry was not recorded')
        aborted = matching('MISSION_ABORTED')
        require(bool(aborted) and 'navigation failed after one retry' in aborted[-1].get('reason', ''),
                'mission must abort on the exhausted Nav2 retry policy')
        require(not matching('SAMPLE_PICK_CONFIRMED'),
                'arm pick must not run after navigation failure')

    if scenario == 'target_loss':
        aborted = matching('MISSION_ABORTED')
        require(bool(matching('ARRIVAL_AND_STOP_CONFIRMED',
                              lambda event: event.get('destination') == 'target')),
                'target stop was not confirmed before detection timeout')
        require(bool(aborted) and 'stage timeout in DETECTING_TARGET' in aborted[-1].get('reason', ''),
                'target loss must abort in DETECTING_TARGET')
        require(not matching('ARM_REQUEST_SENT', lambda event: event.get('stage') == 'pick'),
                'arm pick must not run when the target is missing')

    if scenario == 'grasp_failure':
        aborted = matching('MISSION_ABORTED')
        require(bool(matching('ARM_CANCEL_SENT',
                              lambda event: event.get('reason') == 'grasp failure injection')),
                'grasp failure cancellation was not issued')
        require(bool(matching('ARM_CANCEL_CONFIRMED')),
                'upstream ROS task did not confirm quiescence after cancellation')
        require(bool(matching('ARM_RESULT_RECEIVED',
                              lambda event: event.get('stage') == 'pick'
                              and event.get('outcome') == 'failure')),
                'arm planner failure result was not received')
        require(bool(aborted) and 'pick stage returned failure' in aborted[-1].get('reason', ''),
                'failed pick must abort the mission')
        require(not matching('NAV_GOAL_SENT', lambda event: event.get('destination') == 'home'),
                'return navigation must not start after failed pick')

    if scenario == 'communication_loss':
        aborted = matching('MISSION_ABORTED')
        require(bool(matching('ARM_REQUEST_SENT', lambda event: event.get('stage') == 'pick')),
                'pick request did not reach the arm node before result-channel loss')
        require(bool(matching('ARM_RESULT_DROPPED_FOR_TEST')),
                'the test did not drop the result-channel response')
        require(bool(matching('ARM_CANCEL_SENT',
                              lambda event: event.get('reason') == 'stage timeout in ARM_PICK')),
                'coordinator did not cancel the active pick after result timeout')
        require(bool(aborted) and 'stage timeout in ARM_PICK' in aborted[-1].get('reason', ''),
                'missing arm result must abort on communication timeout')
        require(bool(matching('ARM_CANCEL_CONFIRMED')),
                'active arm task did not confirm upstream quiescence after result loss')
        require(not matching('SAMPLE_PICK_CONFIRMED'),
                'sample pick must not be confirmed after communication loss')
        require(not matching('NAV_GOAL_SENT', lambda event: event.get('destination') == 'home'),
                'return navigation must not start after communication loss')

    if scenario == 'duplicate_trigger':
        accepted = len(matching('TRIGGER_ACCEPTED'))
        ignored = len(matching('TRIGGER_IGNORED'))
        require(accepted == 1, 'expected one accepted trigger, observed %d' % accepted)
        require(ignored >= 2, 'expected at least two ignored duplicate triggers, observed %d' % ignored)

    return errors


def main():
    expected = os.environ.get('P6_EXPECTED_STATE', 'SUCCEEDED')
    scenario = os.environ.get('P6_SCENARIO', 'normal')
    event_log_path = os.environ.get('P6_EVENT_LOG_PATH')
    timeout_sec = float(os.environ.get('P6_SCENARIO_TIMEOUT', '360'))
    rclpy.init(args=None)
    node = ScenarioMonitor(expected, scenario)
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.state in ('SUCCEEDED', 'ABORTED'):
                # Let the coordinator flush the terminal event before reading JSONL.
                end = time.monotonic() + 0.75
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if node.state != expected:
                    print('FAIL expected=%s actual=%s' % (expected, node.state), file=sys.stderr)
                    return 1
                if not event_log_path or not os.path.isfile(event_log_path):
                    print('FAIL mission event log is missing', file=sys.stderr)
                    return 1
                errors = _validate(scenario, expected, _read_events(event_log_path))
                if errors:
                    for error in errors:
                        print('FAIL %s' % error, file=sys.stderr)
                    return 1
                if scenario in ('grasp_failure', 'communication_loss'):
                    cancel = next((event for event in _read_events(event_log_path)
                                   if event.get('event') == 'ARM_CANCEL_CONFIRMED'), None)
                    if cancel is None or not node.verify_cancelled_task_quiet(
                            cancel['task_id'], cancel['trajectory_epoch']):
                        return 1
                    if not node.verify_new_task_takeover():
                        return 1
                print('PASS scenario=%s terminal=%s events=%s' %
                      (scenario, node.state, event_log_path))
                return 0
        print('FAIL scenario timed out; last_state=%s' % node.state, file=sys.stderr)
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        print('FAIL monitor interrupted; last_state=%s' % node.state, file=sys.stderr)
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
