#!/usr/bin/env python3
"""Wait for a P6 terminal state and assert duplicate-trigger behavior."""

import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


class ScenarioMonitor(Node):
    def __init__(self, expected, scenario):
        super().__init__('p6_scenario_monitor')
        self.expected = expected
        self.scenario = scenario
        self.state = None
        self.accepted = 0
        self.ignored = 0
        self._duplicate_sent = False
        self._duplicate_count = 0
        self._duplicate_timer = None
        self._start_pub = self.create_publisher(Bool, '/p6/start', 10)
        self.create_subscription(String, '/p6/mission_state', self._on_state, 10)
        self.create_subscription(String, '/p6/mission_events', self._on_event, 20)

    def _on_state(self, msg):
        self.state = msg.data
        if self.scenario == 'duplicate_trigger' and self.state == 'WAITING_FOR_STACK' and not self._duplicate_sent:
            self._duplicate_sent = True
            self._duplicate_timer = self.create_timer(0.15, self._send_duplicates)

    def _send_duplicates(self):
        if self._duplicate_count >= 3:
            self._duplicate_timer.cancel()
            return
        msg = Bool()
        msg.data = True
        self._start_pub.publish(msg)
        self._duplicate_count += 1

    def _on_event(self, msg):
        try:
            event = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        if event.get('event') == 'TRIGGER_ACCEPTED':
            self.accepted += 1
        elif event.get('event') == 'TRIGGER_IGNORED':
            self.ignored += 1


def main():
    expected = os.environ.get('P6_EXPECTED_STATE', 'SUCCEEDED')
    scenario = os.environ.get('P6_SCENARIO', 'normal')
    timeout_sec = float(os.environ.get('P6_SCENARIO_TIMEOUT', '360'))
    rclpy.init(args=None)
    node = ScenarioMonitor(expected, scenario)
    deadline = time.monotonic() + timeout_sec
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.state in ('SUCCEEDED', 'ABORTED'):
                # Allow the matching terminal event and duplicate audit events through DDS.
                end = time.monotonic() + 0.75
                while time.monotonic() < end:
                    rclpy.spin_once(node, timeout_sec=0.05)
                if node.state != expected:
                    print('FAIL expected=%s actual=%s' % (expected, node.state), file=sys.stderr)
                    return 1
                if scenario == 'duplicate_trigger' and (node.accepted != 1 or node.ignored < 2):
                    print('FAIL duplicate trigger counts accepted=%d ignored=%d' %
                          (node.accepted, node.ignored), file=sys.stderr)
                    return 1
                print('PASS scenario=%s terminal=%s accepted=%d ignored=%d' %
                      (scenario, node.state, node.accepted, node.ignored))
                return 0
        print('FAIL scenario timed out; last_state=%s' % node.state, file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
