"""Translate the existing planner's position targets into Gazebo joint torques."""
from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int8

from .gazebo_control import (
    ARM_JOINTS, ARM_MAX_VELOCITY, ARM_INITIAL_POSITIONS, ARM_KD, ARM_KI, ARM_KP,
    ARM_LOWER, ARM_UPPER, GRIPPER_JOINT, GRIPPER_MIMIC_JOINT,
    GRIPPER_EFFORT_LIMIT, GRIPPER_KD, GRIPPER_KI, GRIPPER_KP,
    GRIPPER_LOWER, GRIPPER_UPPER, GRIPPER_MIMIC_LOWER, GRIPPER_MIMIC_UPPER,
    clamp, effort_pd, slew_target, seed_hold_targets, GRIPPER_MAX_VELOCITY,
)


class GazeboEffortController(Node):
    """A saturated PD+I servo. State feedback comes only from Gazebo."""

    def __init__(self) -> None:
        super().__init__("gazebo_effort_controller")
        self.declare_parameter("joint_states_topic", "/joint_states")
        self.declare_parameter("arm_state_topic", "/p2_gazebo/arm_joint_state")
        self.declare_parameter("arm_command_topic", "/p2_gazebo/arm_joint_command")
        self.declare_parameter("gripper_command_topic", "/p2_gazebo/gripper_command")
        self.declare_parameter("arm_position_topic", "/arm_position_controller/commands")
        self.declare_parameter("gripper_effort_topic", "/gripper_effort_controller/commands")
        self.declare_parameter(
            "initial_positions", list(ARM_INITIAL_POSITIONS))
        self.declare_parameter("control_rate_hz", 200.0)
        self.declare_parameter("state_timeout_sec", 0.25)

        self._positions = {}
        self._velocities = {}
        initial = list(self.get_parameter("initial_positions").value)
        if len(initial) != len(ARM_JOINTS):
            raise ValueError("initial_positions must contain six joint values")
        self._targets = {
            name: clamp(float(initial[index]), ARM_LOWER[index], ARM_UPPER[index])
            for index, name in enumerate(ARM_JOINTS)
        }
        self._targets[GRIPPER_JOINT] = GRIPPER_LOWER
        self._targets[GRIPPER_MIMIC_JOINT] = GRIPPER_MIMIC_UPPER
        self._slewed_targets = dict(self._targets)
        self._initialized_from_state = False
        self._integrals = {name: 0.0 for name in (*ARM_JOINTS, GRIPPER_JOINT, GRIPPER_MIMIC_JOINT)}
        self._last_state_at = 0.0
        self._last_tick_at = time.monotonic()

        command_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE)
        state_topic = str(self.get_parameter("joint_states_topic").value)
        self._feedback = self.create_publisher(
            JointState, str(self.get_parameter("arm_state_topic").value), 20)
        self.create_subscription(JointState, state_topic, self._on_joint_state, 50)
        self.create_subscription(
            JointState, str(self.get_parameter("arm_command_topic").value),
            self._on_arm_command, qos_profile_sensor_data)
        self.create_subscription(
            Int8, str(self.get_parameter("gripper_command_topic").value),
            self._on_gripper_command, command_qos)
        self._arm_position = self.create_publisher(
            Float64MultiArray, str(self.get_parameter("arm_position_topic").value), 10)
        self._gripper_effort = self.create_publisher(
            Float64MultiArray,
            str(self.get_parameter("gripper_effort_topic").value), 10)

        rate = max(50.0, float(self.get_parameter("control_rate_hz").value))
        self._timer = self.create_timer(1.0 / rate, self._control_tick)
        self.get_logger().info(
            "Gazebo joint-command adapter ready; measured state source="
            f"{state_topic}; planner feedback="
            f"{self.get_parameter('arm_state_topic').value}; "
            "serial/camera hardware drivers are not launched")

    def _on_joint_state(self, msg: JointState) -> None:
        count = min(len(msg.name), len(msg.position))
        for index in range(count):
            name = msg.name[index]
            self._positions[name] = float(msg.position[index])
            if index < len(msg.velocity):
                self._velocities[name] = float(msg.velocity[index])
        if not all(name in self._positions for name in ARM_JOINTS):
            return
        self._last_state_at = time.monotonic()
        if not self._initialized_from_state:
            hold_names = (*ARM_JOINTS, GRIPPER_JOINT, GRIPPER_MIMIC_JOINT)
            self._targets = seed_hold_targets(
                self._targets, self._positions, hold_names)
            for index, name in enumerate(ARM_JOINTS):
                self._targets[name] = clamp(
                    self._targets[name], ARM_LOWER[index], ARM_UPPER[index])
            self._targets[GRIPPER_JOINT] = clamp(
                self._targets[GRIPPER_JOINT], GRIPPER_LOWER, GRIPPER_UPPER)
            self._targets[GRIPPER_MIMIC_JOINT] = clamp(
                self._targets[GRIPPER_MIMIC_JOINT],
                GRIPPER_MIMIC_LOWER, GRIPPER_MIMIC_UPPER)
            self._slewed_targets = dict(self._targets)
            self._initialized_from_state = True
            self.get_logger().info(
                "Initialized physical hold targets from Gazebo joint feedback: "
                + ", ".join(
                    f"{name}={self._targets[name]:.4f}"
                    for name in (*ARM_JOINTS, GRIPPER_JOINT, GRIPPER_MIMIC_JOINT)))

        feedback = JointState()
        feedback.header = msg.header
        for name in (*ARM_JOINTS, GRIPPER_JOINT, GRIPPER_MIMIC_JOINT):
            if name in self._positions:
                feedback.name.append(name)
                feedback.position.append(self._positions[name])
                feedback.velocity.append(self._velocities.get(name, 0.0))
        self._feedback.publish(feedback)

    def _on_arm_command(self, msg: JointState) -> None:
        for name, value in zip(msg.name, msg.position):
            if name not in ARM_JOINTS:
                continue
            index = ARM_JOINTS.index(name)
            previous = self._targets[name]
            self._targets[name] = clamp(
                float(value), ARM_LOWER[index], ARM_UPPER[index])
            if abs(self._targets[name] - previous) >= 0.05:
                self.get_logger().info(
                    f"Received physical arm target {name}={self._targets[name]:.4f}")

    def _on_gripper_command(self, msg: Int8) -> None:
        if msg.data == 1:
            self._targets[GRIPPER_JOINT] = GRIPPER_UPPER
            self._targets[GRIPPER_MIMIC_JOINT] = GRIPPER_MIMIC_LOWER
        elif msg.data == -1:
            self._targets[GRIPPER_JOINT] = GRIPPER_LOWER
            self._targets[GRIPPER_MIMIC_JOINT] = GRIPPER_MIMIC_UPPER
        else:
            self.get_logger().warning(
                f"ignoring unsupported gripper command {msg.data}; expected +/-1")

    def _control_tick(self) -> None:
        now = time.monotonic()
        dt = max(0.0, min(0.02, now - self._last_tick_at))
        self._last_tick_at = now
        age = now - self._last_state_at
        timeout = float(self.get_parameter("state_timeout_sec").value)
        if age > timeout or not all(name in self._positions for name in ARM_JOINTS):
            if all(name in self._positions for name in ARM_JOINTS):
                self._arm_position.publish(Float64MultiArray(
                    data=[self._positions[name] for name in ARM_JOINTS]))
            self._gripper_effort.publish(Float64MultiArray(data=[0.0, 0.0]))
            return

        arm_positions = []
        for index, name in enumerate(ARM_JOINTS):
            self._slewed_targets[name] = slew_target(
                self._slewed_targets[name], self._targets[name],
                ARM_MAX_VELOCITY[index], dt)
            arm_positions.append(self._slewed_targets[name])
        self._arm_position.publish(Float64MultiArray(data=arm_positions))

        gripper_efforts = []
        for name, lower, upper in (
            (GRIPPER_JOINT, GRIPPER_LOWER, GRIPPER_UPPER),
            (GRIPPER_MIMIC_JOINT, GRIPPER_MIMIC_LOWER, GRIPPER_MIMIC_UPPER),
        ):
            if name not in self._positions:
                gripper_efforts.append(0.0)
                continue
            self._slewed_targets[name] = slew_target(
                self._slewed_targets[name], self._targets[name],
                GRIPPER_MAX_VELOCITY, dt)
            target = clamp(self._slewed_targets[name], lower, upper)
            effort, integral = effort_pd(
                target, self._positions[name], self._velocities.get(name, 0.0),
                self._integrals[name], dt, GRIPPER_KP, GRIPPER_KD, GRIPPER_KI,
                GRIPPER_EFFORT_LIMIT)
            self._integrals[name] = integral
            gripper_efforts.append(effort)
        self._gripper_effort.publish(Float64MultiArray(data=gripper_efforts))



def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboEffortController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
