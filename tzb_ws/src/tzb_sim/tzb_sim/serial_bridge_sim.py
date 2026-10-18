#!/usr/bin/env python3
"""Virtual arm and gripper controller; never opens a physical device."""

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8


class SerialBridgeSim(Node):
    def __init__(self) -> None:
        super().__init__('serial_bridge_sim')
        self.declare_parameter('command_topic', '/tzb_catch/arm_joint_command')
        self.declare_parameter('state_topic', '/serial/arm_joint_state')
        self.declare_parameter('gripper_command_topic', '/tzb_catch/gripper_command')
        self.declare_parameter('joint_names', [
            'J1_joint', 'J2_joint', 'J3_joint', 'J4_joint', 'J5_joint', 'J6_joint'])
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('follow_gain', 25.0)
        self.declare_parameter('max_velocity', [1.2, 1.2, 1.2, 1.8, 1.8, 2.2])
        self.declare_parameter('initial_positions', [0.0, 1.0, -2.61, 0.0, 0.0, 0.0])
        self.declare_parameter('gripper_joint_name', 'Claw_left_joint')
        self.declare_parameter('gripper_open_position', -0.5236)
        self.declare_parameter('gripper_closed_position', 0.0)
        self.declare_parameter('gripper_max_velocity', 2.5)

        self.joint_names = list(self.get_parameter('joint_names').value)
        self.positions = [float(v) for v in self.get_parameter('initial_positions').value]
        self.velocities = [0.0] * len(self.joint_names)
        self.target = list(self.positions)
        self.max_velocity = [float(v) for v in self.get_parameter('max_velocity').value]
        if not self.joint_names or len(self.positions) != len(self.joint_names):
            raise ValueError('initial_positions must match the configured arm joints')
        if len(self.max_velocity) != len(self.joint_names):
            raise ValueError('max_velocity must match the configured arm joints')
        self.follow_gain = float(self.get_parameter('follow_gain').value)
        self.have_command = False

        self.gripper_joint_name = str(self.get_parameter('gripper_joint_name').value)
        self.gripper_open = float(self.get_parameter('gripper_open_position').value)
        self.gripper_closed = float(self.get_parameter('gripper_closed_position').value)
        self.gripper_max_velocity = float(self.get_parameter('gripper_max_velocity').value)
        self.gripper_position = self.gripper_open
        self.gripper_target = self.gripper_open
        self.gripper_velocity = 0.0

        self.publisher = self.create_publisher(
            JointState, str(self.get_parameter('state_topic').value), 10)
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1)
        self.subscription = self.create_subscription(
            JointState, str(self.get_parameter('command_topic').value),
            self._on_command, command_qos)
        self.gripper_subscription = self.create_subscription(
            Int8, str(self.get_parameter('gripper_command_topic').value),
            self._on_gripper_command, 10)

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.dt = 1.0 / rate
        self.timer = self.create_timer(self.dt, self._tick)
        self.last_time = self.get_clock().now()
        self.get_logger().info(
            f'Virtual-only controller: {self.get_parameter("command_topic").value}'
            f' -> {self.get_parameter("state_topic").value} at {rate:.0f} Hz; '
            f'gripper={self.get_parameter("gripper_command_topic").value}; '
            f'initial={self.positions}; physical serial is not opened')

    def _on_command(self, msg: JointState) -> None:
        positions = list(msg.position)
        if len(positions) != len(self.target):
            self.get_logger().warning(
                f'Ignoring arm command with {len(positions)} joints')
            return
        self.target = [float(v) for v in positions]
        self.have_command = True

    def _on_gripper_command(self, msg: Int8) -> None:
        if int(msg.data) == -1:
            self.gripper_target = self.gripper_open
            self.get_logger().info('Virtual gripper command: open')
        elif int(msg.data) == 1:
            self.gripper_target = self.gripper_closed
            self.get_logger().info('Virtual gripper command: close')
        else:
            self.get_logger().warning(
                f'Ignoring unsupported virtual gripper command {int(msg.data)}')

    @staticmethod
    def _step_towards(position, target, gain, max_velocity, dt):
        error = target - position
        desired = error * gain
        velocity = max(-max_velocity, min(max_velocity, desired))
        step = velocity * dt
        if abs(step) > abs(error):
            step = error
            velocity = step / dt if dt > 0.0 else 0.0
        return position + step, velocity

    def _tick(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.5:
            dt = self.dt

        for i in range(len(self.joint_names)):
            self.positions[i], self.velocities[i] = self._step_towards(
                self.positions[i], self.target[i], self.follow_gain,
                self.max_velocity[i], dt)
        self.gripper_position, self.gripper_velocity = self._step_towards(
            self.gripper_position, self.gripper_target, self.follow_gain,
            self.gripper_max_velocity, dt)

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(self.joint_names) + [self.gripper_joint_name]
        msg.position = [float(v) for v in self.positions] + [self.gripper_position]
        msg.velocity = [float(v) for v in self.velocities] + [self.gripper_velocity]
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
