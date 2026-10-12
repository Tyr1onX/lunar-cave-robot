#!/usr/bin/env python3
"""P7 offline mission trigger, simulated operator action, fault injection and assertions."""
from __future__ import annotations

import json
import os
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class MissionControl(Node):
    def __init__(self):
        super().__init__('p7_offline_mission_control')
        self.scenario = os.environ.get('P7_SCENARIO', 'normal')
        self.fault = os.environ.get('P7_FAULT', '').strip()
        default_expected = (
            'SUCCEEDED' if self.scenario in ('normal', 'duplicate_trigger') and not self.fault
            else 'ABORTED')
        self.expected = os.environ.get('P7_EXPECTED_STATE') or default_expected
        self.timeout = float(os.environ.get('P7_MISSION_TIMEOUT_SEC', '260'))
        self.operator_action = os.environ.get('P7_SIMULATED_OPERATOR_ACTION', '')
        self.log_path = Path(os.environ.get('P7_CONTROLLER_LOG', '/tmp/p7/controller-events.jsonl'))
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.started_at = time.monotonic()
        self.mission_state = ''
        self.safety = ''
        self.chassis_link_status = ''
        self.peer = {}
        self.arm_feedback_count = 0
        self.odom_count = 0
        self.events = []
        self.fault_sent = False
        self.fault_injection_time_unix = None
        self.driver_restarted = False
        self.last_state_rx = 0.0
        self.start_pub = self.create_publisher(Bool, '/p6/start', 10)
        self.fault_pub = self.create_publisher(String, '/p7/sim/fault', 10)
        self.create_subscription(String, '/p6/mission_state', self._on_state, 10)
        self.create_subscription(String, '/p6/mission_events', self._on_event, 20)
        self.create_subscription(String, '/serial/arm_safety/status', self._on_safety, 10)
        self.create_subscription(String, '/chassis/serial_link_status',
                                 self._on_chassis_link_status, 10)
        self.create_subscription(String, '/p7/sim/wire_status', self._on_peer, 10)
        self.create_subscription(JointState, '/p2_offline/arm_joint_state',
                                 self._on_arm_feedback, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.enable_client = self.create_client(Trigger, '/serial/arm_safety/enable')

    def _write(self, event, **fields):
        item = {'time_unix': time.time(), 'event': event, 'mission_state': self.mission_state}
        item.update(fields)
        if event == 'FAULT_INJECTION':
            self.fault_injection_time_unix = item['time_unix']
        with self.log_path.open('a', encoding='utf-8') as stream:
            stream.write(json.dumps(item, sort_keys=True) + '\n')
        print('P7_EVENT ' + json.dumps(item, sort_keys=True), flush=True)

    def _on_state(self, msg):
        self.mission_state = msg.data.strip()
        self.last_state_rx = time.monotonic()

    def _on_event(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        self.events.append(data)

    def _on_safety(self, msg):
        self.safety = msg.data

    def _on_chassis_link_status(self, msg):
        self.chassis_link_status = msg.data.strip()

    def _on_peer(self, msg):
        parsed = {}
        for token in msg.data.split():
            if '=' in token:
                key, value = token.split('=', 1)
                parsed[key] = value
        self.peer = parsed

    def _on_arm_feedback(self, _msg):
        self.arm_feedback_count += 1

    def _on_odom(self, _msg):
        self.odom_count += 1

    def spin_wait(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return
        raise RuntimeError('timeout waiting for ' + description)

    @staticmethod
    def _fields(text):
        result = {}
        for token in text.split():
            if '=' in token:
                key, value = token.split('=', 1)
                result[key] = value
        return result

    def _operator_enable(self):
        if self.operator_action != 'enable-offline-simulator':
            raise RuntimeError(
                'P7_SIMULATED_OPERATOR_ACTION=enable-offline-simulator is required; '
                'the safety enable service will not be called implicitly')
        def ready():
            f = self._fields(self.safety)
            try:
                age = float(f.get('feedback_age_s', '-1'))
            except ValueError:
                age = -1
            return (f.get('state') == 'DISARMED' and f.get('serial_open') == 'true'
                    and 0.0 <= age <= 0.5 and self.arm_feedback_count >= 3)
        self.spin_wait(ready, 25.0, 'DISARMED P4e status and live PTY feedback')
        f = self._fields(self.safety)
        self._write('SIMULATED_OPERATOR_ACTION', action='enable P4e offline PTY controller',
                    authorization=self.operator_action, safety_state=f.get('state'),
                    feedback_age_sec=float(f['feedback_age_s']), arm_feedback_samples=self.arm_feedback_count,
                    service='/serial/arm_safety/enable', reset_called=False)
        self.spin_wait(self.enable_client.service_is_ready, 10.0, 'P4e operator enable service')
        future = self.enable_client.call_async(Trigger.Request())
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done() or future.result() is None:
            raise RuntimeError('P4e simulated operator enable service did not respond')
        response = future.result()
        self._write('SIMULATED_OPERATOR_ENABLE_RESULT', success=response.success, message=response.message)
        if not response.success:
            raise RuntimeError('P4e enable interlock refused: ' + response.message)
        self.spin_wait(lambda: 'state=ARMED_WAIT_COMMAND' in self.safety, 3.0,
                       'P4e ARMED_WAIT_COMMAND after explicit simulated operator action')

    def _restart_driver(self):
        pid_file = Path(os.environ['P7_ARM_DRIVER_PID_FILE'])
        pid = int(pid_file.read_text().strip())
        self._write('FAULT_INJECTION', fault='driver_restart', old_driver_pid=pid,
                    action='terminate P4e node; restart remains DISARMED without reset or enable')
        try:
            os.killpg(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        time.sleep(0.6)
        arm_ws = os.environ['P7_ARM_WS']
        param_file = os.environ['P7_ARM_DRIVER_PARAMS']
        log_path = Path(os.environ['P7_ARM_RESTART_LOG'])
        log_path.parent.mkdir(parents=True, exist_ok=True)
        cmd = (f'source /opt/ros/humble/setup.bash; '
               f'source "{arm_ws}/install/p7/setup.bash"; '
               f'exec ros2 run rm_serial_driver node --ros-args --params-file "{param_file}"')
        log = log_path.open('a', encoding='utf-8')
        proc = subprocess.Popen(['bash', '-c', cmd], start_new_session=True,
                               stdout=log, stderr=subprocess.STDOUT, env=os.environ.copy())
        pid_file.write_text(str(proc.pid) + '\n', encoding='utf-8')
        self.driver_restarted = True
        self._write('P4E_DRIVER_RESTARTED', new_driver_pid=proc.pid,
                    recovery='DISARMED; no automatic reset or enable')

    def _inject_if_needed(self):
        if self.fault and not self.fault_sent:
            motion = int(float(self.peer.get('chassis_motion_frames', '0')))
            arm_enabled = int(float(self.peer.get('arm_enabled_frames', '0')))
            if self.fault == 'chassis_disconnect' and self.mission_state == 'NAVIGATING_TO_TARGET' and motion >= 20:
                self.fault_pub.publish(String(data='chassis_disconnect'))
                self.fault_sent = True
                self._write('FAULT_INJECTION', fault=self.fault, decoded_motion_frames=motion)
            elif (self.fault == 'chassis_disconnect_return' and
                  self.mission_state == 'NAVIGATING_HOME' and motion >= 20):
                self.fault_pub.publish(String(data='chassis_disconnect'))
                self.fault_sent = True
                self._write('FAULT_INJECTION', fault=self.fault, decoded_motion_frames=motion,
                            prior_pick_confirmed=any(event.get('event') == 'SAMPLE_PICK_CONFIRMED'
                                                     for event in self.events))
            elif (self.fault == 'chassis_bridge_stop' and
                  self.mission_state in ('NAVIGATING_TO_TARGET', 'NAVIGATING_HOME') and motion >= 20):
                pid_file = Path(os.environ['P7_CHASSIS_BRIDGE_PID_FILE'])
                bridge_pid = int(pid_file.read_text().strip())
                os.killpg(bridge_pid, signal.SIGINT)
                self.fault_sent = True
                self._write('FAULT_INJECTION', fault=self.fault, decoded_motion_frames=motion,
                            action='stop P5b bridge; P6 must detect stale host-status heartbeat')
            elif self.fault in ('arm_feedback_stop', 'arm_disconnect', 'driver_restart') and \
                    self.mission_state == 'ARM_PICK' and arm_enabled >= 5:
                if self.fault == 'driver_restart':
                    self._restart_driver()
                else:
                    self.fault_pub.publish(String(data=self.fault))
                    self._write('FAULT_INJECTION', fault=self.fault, enabled_arm_frames=arm_enabled)
                self.fault_sent = True

    def run(self):
        self._operator_enable()
        self.spin_wait(lambda: self.start_pub.get_subscription_count() > 0, 15.0,
                       'P6 mission-start subscriber discovery')
        self._write('MISSION_START_PUBLISHED', scenario=self.scenario, fault=self.fault or None)
        start_deadline = time.monotonic() + 10.0
        last_publish = 0.0
        while time.monotonic() < start_deadline and not any(
                event.get('event') == 'TRIGGER_ACCEPTED' for event in self.events):
            now = time.monotonic()
            if now - last_publish >= 0.5:
                self.start_pub.publish(Bool(data=True))
                last_publish = now
            rclpy.spin_once(self, timeout_sec=0.05)
        if not any(event.get('event') == 'TRIGGER_ACCEPTED' for event in self.events):
            raise RuntimeError('P6 did not confirm mission trigger acceptance')
        self._write('MISSION_TRIGGER_ACCEPTED', publish_retries=max(0, int((time.monotonic()-start_deadline+10.0)//0.5)-1))
        if self.scenario == 'duplicate_trigger':
            for _ in range(3):
                self.start_pub.publish(Bool(data=True))
                time.sleep(0.05)
            self._write('DUPLICATE_TRIGGER_INJECTED', count=3)
        deadline = time.monotonic() + self.timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            self._inject_if_needed()
            if self.mission_state in ('SUCCEEDED', 'ABORTED'):
                break
        if self.mission_state not in ('SUCCEEDED', 'ABORTED'):
            raise RuntimeError('mission did not reach a terminal state; last=' + self.mission_state)
        if self.mission_state != self.expected:
            raise RuntimeError(f'expected mission state {self.expected}, got {self.mission_state}')
        time.sleep(0.7)
        # Pump terminal events and final simulator diagnostics before assertions.
        for _ in range(8):
            rclpy.spin_once(self, timeout_sec=0.05)
        summary = {
            'scenario': self.scenario, 'fault': self.fault or None,
            'mission_state': self.mission_state,
            'elapsed_sec': round(time.monotonic() - self.started_at, 3),
            'arm_feedback_samples': self.arm_feedback_count,
            'odom_samples': self.odom_count,
            'wire_status': self.peer,
            'chassis_link_status': self.chassis_link_status,
            'mission_event_names': [event.get('event') for event in self.events],
        }
        if (self.fault in ('chassis_disconnect', 'chassis_disconnect_return')
                and self.fault_injection_time_unix is not None):
            health_events = [event for event in self.events
                             if event.get('event') == 'CHASSIS_LINK_UNHEALTHY']
            if health_events:
                summary['chassis_fault_detection_latency_sec'] = round(
                    health_events[0].get('time_unix', 0.0) - self.fault_injection_time_unix, 3)
        self._write('P7_RUN_SUMMARY', **summary)
        if self.mission_state == 'SUCCEEDED':
            required = ('SAMPLE_PICK_CONFIRMED', 'BASE_UNLOAD_CONFIRMED',
                        'P4E_EXECUTION_CONFIRMED', 'NAV_RESULT_CONFIRMED',
                        'ARRIVAL_AND_STOP_CONFIRMED')
            missing = [name for name in required if name not in summary['mission_event_names']]
            if not any(event.get('event') == 'STATE' and event.get('state') == 'SUCCEEDED'
                       for event in self.events):
                missing.append('STATE=SUCCEEDED')
            if missing:
                raise RuntimeError('success missing arm/nav evidence events: ' + ','.join(missing))
            if int(float(self.peer.get('chassis_motion_frames', '0'))) < 20:
                raise RuntimeError('success without decoded chassis motion frames')
            if int(float(self.peer.get('arm_enabled_frames', '0'))) < 20:
                raise RuntimeError('success without enabled 46-byte arm command frames')
            if int(float(self.peer.get('arm_feedback_frames', '0'))) < 20 or self.arm_feedback_count < 20:
                raise RuntimeError('success without controller-derived arm feedback')
            if int(float(self.peer.get('arm_joint_changes', '0'))) < 1:
                raise RuntimeError('success without arm joint changes on the wire')
            if int(float(self.peer.get('arm_gripper_close_frames', '0'))) < 1 or \
                    int(float(self.peer.get('arm_gripper_open_frames', '0'))) < 1:
                raise RuntimeError('success without close and unload/open gripper commands on the wire')
        if self.expected == 'ABORTED':
            if not any(event.get('event') == 'MISSION_ABORTED' for event in self.events):
                raise RuntimeError('ABORTED state has no recorded failure event')
            if any(event.get('event') == 'STATE' and event.get('state') == 'SUCCEEDED'
                   for event in self.events):
                raise RuntimeError('failure scenario emitted a success state')
        if self.fault and self.expected == 'ABORTED' and not self.fault_sent:
            raise RuntimeError('requested fault was never injected')
        aborted_reasons = [
            event.get('reason') for event in self.events if event.get('event') == 'MISSION_ABORTED']
        if self.fault == 'arm_feedback_stop':
            if 'P4e arm host safety reports FAULT_LATCHED' not in aborted_reasons:
                raise RuntimeError('P4e feedback-timeout latch was not propagated to P6')
        if self.fault == 'arm_disconnect':
            if not any(reason in (
                    'P4e arm host safety reports FAULT_LATCHED',
                    'required P4e arm safety status is missing or stale')
                    for reason in aborted_reasons):
                raise RuntimeError('P6 did not fail safe on P4e disconnect or stale safety status')
            if 'ARM_CANCEL_CONFIRMED' not in summary['mission_event_names']:
                raise RuntimeError('P6 did not confirm upstream cancellation after P4e disconnect')
            if self.peer.get('arm_connected') != 'False':
                raise RuntimeError('arm_disconnect fault was not confirmed by the PTY simulator')
        if self.fault == 'driver_restart':
            restart_log = Path(os.environ['P7_ARM_RESTART_LOG']).read_text(encoding='utf-8')
            if not self.driver_restarted or 'safety=DISARMED' not in restart_log:
                raise RuntimeError('restarted P4e process did not start DISARMED')
            if 'ARM_CANCEL_CONFIRMED' not in summary['mission_event_names']:
                raise RuntimeError('P6 did not confirm cancellation after P4e driver restart')
            last_arm = int(self.peer.get('last_arm_sequence', '-1'))
            last_enabled = int(self.peer.get('last_enabled_arm_sequence', '-1'))
            if last_arm < 0 or last_enabled < 0 or last_arm >= last_enabled:
                raise RuntimeError('restarted P4e emitted an enabled frame or no post-restart frame')
        if self.fault == 'chassis_disconnect' and 'ARM_REQUEST_SENT' in summary['mission_event_names']:
            raise RuntimeError('P6 requested arm operation after chassis communication failure')
        if self.fault in ('chassis_disconnect', 'chassis_disconnect_return', 'chassis_bridge_stop'):
            if 'CHASSIS_LINK_UNHEALTHY' not in summary['mission_event_names']:
                raise RuntimeError('P6 did not consume the P5b host-side serial link health status')
            health_events = [event for event in self.events
                             if event.get('event') == 'CHASSIS_LINK_UNHEALTHY']
            if self.fault_injection_time_unix is None:
                raise RuntimeError('chassis fault injection timestamp was not recorded')
            detection_latency = summary.get('chassis_fault_detection_latency_sec', -1.0)
            if not 0.0 <= detection_latency < 2.0:
                raise RuntimeError(
                    'P6 did not abort promptly after host-side PTY loss: %.3fs' % detection_latency)
        if self.fault == 'chassis_disconnect_return':
            if 'SAMPLE_PICK_CONFIRMED' not in summary['mission_event_names']:
                raise RuntimeError('return-navigation disconnect was injected before confirmed pick')
            if 'ARM_CANCEL_CONFIRMED' not in summary['mission_event_names']:
                raise RuntimeError('P6 did not stop and confirm the completed-task arm hold after return-navigation failure')
        if self.scenario in ('target_loss', 'nav_failure'):
            if 'ARM_REQUEST_SENT' in summary['mission_event_names']:
                raise RuntimeError('P6 requested an arm operation after navigation/detection failure')
        if self.scenario == 'duplicate_trigger':
            if sum(event.get('event') == 'TRIGGER_ACCEPTED' for event in self.events) != 1:
                raise RuntimeError('duplicate mission start was accepted more than once')
            if not any(event.get('event') == 'TRIGGER_IGNORED' for event in self.events):
                raise RuntimeError('duplicate mission start did not produce an ignored-trigger record')
            target_goals = [event for event in self.events
                            if event.get('event') == 'NAV_GOAL_SENT'
                            and event.get('destination') == 'target']
            if len(target_goals) != 1:
                raise RuntimeError('duplicate mission start created duplicate target navigation goals')
        if self.scenario in ('grasp_failure', 'communication_loss'):
            if 'ARM_CANCEL_CONFIRMED' not in summary['mission_event_names']:
                raise RuntimeError('P6 did not receive upstream trajectory cancellation confirmation')
        print('P7_ACCEPTANCE_RESULT=PASS', flush=True)
        print('P7_SUMMARY_JSON=' + json.dumps(summary, sort_keys=True), flush=True)


def main():
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('P7 safety guard: ROS_LOCALHOST_ONLY=1 is required')
    rclpy.init(args=None)
    node = MissionControl()
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
