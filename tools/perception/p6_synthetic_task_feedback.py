#!/usr/bin/env python3
"""Simulate the P6 task result/cancel peer without starting any arm interface."""

from __future__ import annotations

import argparse


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=('normal', 'grasp_failure', 'communication_loss'), default='normal')
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class SimulatedTaskPeer(Node):
        def __init__(self, scenario):
            super().__init__('p6_synthetic_task_feedback')
            self.scenario = scenario
            self.pending = None
            self.result_pub = self.create_publisher(String, '/p6/arm_task/result', 10)
            self.cancel_ack_pub = self.create_publisher(String, '/p6/arm_task/cancel_ack', 10)
            self.create_subscription(String, '/p6/arm_task/request', self.on_request, 10)
            self.create_subscription(String, '/p6/arm_task/cancel', self.on_cancel, 10)
            self.get_logger().warning('SYNTHETIC TASK FEEDBACK ONLY; no planner, serial link, arm or actuator is connected')

        def on_request(self, message):
            fields = message.data.split('|')
            if len(fields) != 3:
                self.get_logger().error('ignored malformed synthetic task request')
                return
            task_id, stage, epoch = fields
            if self.scenario == 'grasp_failure' and stage == 'pick':
                self.pending = (task_id, stage, epoch)
                self.get_logger().info('withholding synthetic pick result to exercise coordinator cancellation')
                return
            result = String()
            result.data = '%s|%s|success|%s' % (task_id, stage, epoch)
            self.result_pub.publish(result)

        def on_cancel(self, message):
            fields = message.data.split('|')
            if len(fields) != 2:
                self.get_logger().error('ignored malformed synthetic cancel request')
                return
            ack = String()
            ack.data = fields[0] + '|' + fields[1] + '|UPSTREAM_QUIESCED'
            self.cancel_ack_pub.publish(ack)
            if self.pending is not None and self.pending[0] == fields[0] and self.pending[2] == fields[1]:
                task_id, stage, epoch = self.pending
                result = String()
                result.data = '%s|%s|failure|%s' % (task_id, stage, epoch)
                self.result_pub.publish(result)
                self.pending = None
            self.get_logger().info('synthetic upstream-quiesced acknowledgement published')

    rclpy.init()
    node = SimulatedTaskPeer(args.scenario)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
