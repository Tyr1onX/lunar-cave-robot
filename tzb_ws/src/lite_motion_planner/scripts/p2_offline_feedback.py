#!/usr/bin/env python3
"""Explicitly synthetic joint feedback for the localhost-only P2 offline test."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState


class OfflineArmFeedback(Node):
    def __init__(self):
        super().__init__('p2_offline_arm_feedback')
        self.declare_parameter('state_topic', '/p2_offline/arm_joint_state')
        self.declare_parameter('command_topic', '/p2_offline/arm_joint_command')
        self.declare_parameter('joint_names', [
            'J1_joint', 'J2_joint', 'J3_joint',
            'J4_joint', 'J5_joint', 'J6_joint',
        ])
        self.declare_parameter('initial_positions', [0.0, 1.0, -2.61, 0.0, 0.0, 0.0])
        names = list(self.get_parameter('joint_names').value)
        positions = list(self.get_parameter('initial_positions').value)
        if len(names) != 6 or len(positions) != len(names):
            raise ValueError('offline profile expects six matching arm joint names and positions')
        self._values = dict(zip(names, map(float, positions)))
        state_topic = str(self.get_parameter('state_topic').value)
        command_topic = str(self.get_parameter('command_topic').value)
        self._publisher = self.create_publisher(
            JointState, state_topic, qos_profile_sensor_data)
        self._subscription = self.create_subscription(
            JointState, command_topic, self._on_command, qos_profile_sensor_data)
        self._timer = self.create_timer(0.02, self._publish_state)
        self.get_logger().warning(
            'OFFLINE SIMULATION ONLY: publishes synthetic joint state and echoes '
            'commands only on /p2_offline/*; no serial or hardware node is launched.')

    def _on_command(self, message):
        for name, position in zip(message.name, message.position):
            if name in self._values:
                self._values[name] = float(position)

    def _publish_state(self):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(self._values)
        message.position = list(self._values.values())
        self._publisher.publish(message)


def main():
    rclpy.init()
    node = OfflineArmFeedback()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
