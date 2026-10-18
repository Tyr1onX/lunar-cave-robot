#!/usr/bin/env python3
"""Virtual lower-level arm controller used when no serial device is attached.
虚拟下位机
The real rm_serial_driver forwards /tzb_catch/arm_joint_command to the STM32
and publishes the measured joints on /serial/arm_joint_state at 100 Hz.
This node reproduces that interface, including a velocity limit and a first
order response, so tzb_catch can run its normal feedback-based sequence.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState


class SerialBridgeSim(Node):
    def __init__(self) -> None:
        super().__init__('serial_bridge_sim')

        self.declare_parameter('command_topic', '/tzb_catch/arm_joint_command')
        self.declare_parameter('state_topic', '/serial/arm_joint_state')
        self.declare_parameter(
            'joint_names',
            ['J1_joint', 'J2_joint', 'J3_joint',
             'J4_joint', 'J5_joint', 'J6_joint'])
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('follow_gain', 25.0)
        self.declare_parameter(
            'max_velocity', [1.2, 1.2, 1.2, 1.8, 1.8, 2.2])
        self.declare_parameter(
            'initial_positions', [0.0, 1.0, -2.61, 0.0, 0.0, 0.0])

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.positions = [float(v) for v in
                          self.get_parameter('initial_positions').value]
        self.velocities = [0.0] * len(self.joint_names)
        self.target = list(self.positions)
        self.max_velocity = [float(v) for v in
                             self.get_parameter('max_velocity').value]
        self.follow_gain = float(self.get_parameter('follow_gain').value)
        self.have_command = False

        self.publisher = self.create_publisher(
            JointState,
            str(self.get_parameter('state_topic').value), 10)
        # tzb_catch publishes command samples as best effort.
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.subscription = self.create_subscription(
            JointState,
            str(self.get_parameter('command_topic').value),
            self._on_command, command_qos)

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self._tick)
        self.last_time = self.get_clock().now()

        self.get_logger().info(
            f'Virtual serial arm bridge: {self.get_parameter("command_topic").value}'
            f' -> {self.get_parameter("state_topic").value} at {rate:.0f} Hz;'
            f' initial={self.positions}')

    def _on_command(self, msg: JointState) -> None:
        positions = list(msg.position)
        if len(positions) != len(self.target):
            self.get_logger().warning(
                f'Ignoring arm command with {len(positions)} joints')
            return
        self.target = [float(v) for v in positions]
        self.have_command = True

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = self.dt

        for i, joint in enumerate(self.joint_names):
            error = self.target[i] - self.positions[i]
            desired = error * self.follow_gain
            limit = self.max_velocity[i]
            velocity = max(-limit, min(limit, desired))
            step = velocity * dt
            if abs(step) > abs(error):
                step = error
                velocity = step / dt if dt > 0.0 else 0.0
            self.positions[i] += step
            self.velocities[i] = velocity

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(self.joint_names)
        msg.position = [float(v) for v in self.positions]
        msg.velocity = [float(v) for v in self.velocities]
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SerialBridgeSim()
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
