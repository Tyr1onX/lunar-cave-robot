#!/usr/bin/env python3
"""Refresh RSP joint-state timestamps from offline-only simulated arm feedback."""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import JointState


class OfflineJointStateRelay(Node):
    def __init__(self):
        super().__init__('p3b_offline_joint_state_relay')
        self.declare_parameter('state_topic', '/p2_offline/arm_joint_state')
        self.declare_parameter('joint_states_topic', '/p2_offline/joint_states')
        self._state_topic = str(self.get_parameter('state_topic').value)
        self._joint_states_topic = str(self.get_parameter('joint_states_topic').value)
        if not self._state_topic.startswith('/p2_offline/') or not self._joint_states_topic.startswith('/p2_offline/'):
            raise RuntimeError('P3b relay accepts only /p2_offline/* topics')
        self._latest = None
        self._last_receive = 0.0
        self._sub = self.create_subscription(
            JointState, self._state_topic, self._on_state, qos_profile_sensor_data)
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(JointState, self._joint_states_topic, reliable)
        self._timer = self.create_timer(0.02, self._publish)
        self.get_logger().warning(
            'OFFLINE SIMULATION ONLY: refreshing /p2_offline/joint_states from synthetic '
            'arm feedback; no hardware command or driver is connected.')

    def _on_state(self, msg):
        self._latest = msg
        self._last_receive = time.monotonic()

    def _publish(self):
        if self._latest is None or time.monotonic() - self._last_receive > 0.5:
            return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._latest.name)
        msg.position = list(self._latest.position)
        msg.velocity = list(self._latest.velocity)
        msg.effort = list(self._latest.effort)
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = OfflineJointStateRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
