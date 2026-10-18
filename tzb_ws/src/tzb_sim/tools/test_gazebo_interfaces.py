#!/usr/bin/env python3
"""Integration test for real Gazebo joint-state feedback and effort control."""
import sys
import time

import rclpy
from tf2_msgs.msg import TFMessage
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8


ARM = ["J1_joint", "J2_joint", "J3_joint", "J4_joint", "J5_joint", "J6_joint"]


class PhysicsProbe(Node):
    def __init__(self):
        super().__init__("p2_gazebo_physics_acceptance")
        self.states = {}
        self.block_pose = None
        self.contact_frames = 0
        self.state_sub = self.create_subscription(
            JointState, "/p2_gazebo/arm_joint_state", self.on_state, 20)
        self.pose_sub = self.create_subscription(
            TFMessage, "/tf", self.on_tf, 30)
        self.contact_sub = self.create_subscription(
            Contacts, "/p2_gazebo/blue_block/contacts", self.on_contacts, 20)
        self.arm_pub = self.create_publisher(
            JointState, "/p2_gazebo/arm_joint_command", 10)
        self.gripper_pub = self.create_publisher(
            Int8, "/p2_gazebo/gripper_command", 10)

    def on_state(self, msg):
        self.states.update(zip(msg.name, msg.position))

    def on_tf(self, msg):
        for transform in msg.transforms:
            frame = transform.child_frame_id
            if frame == "blue_block" or frame.endswith("::blue_block") or frame.endswith("blue_block::link"):
                self.block_pose = transform

    def on_contacts(self, msg):
        if msg.contacts:
            self.contact_frames += 1

    def spin_for(self, duration):
        end = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for(self, predicate, timeout, label):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                return True
        raise RuntimeError(f"timeout waiting for {label}")


def send_arm(probe, names, values):
    message = JointState()
    message.header.stamp = probe.get_clock().now().to_msg()
    message.name = list(names)
    message.position = list(values)
    probe.arm_pub.publish(message)


def main():
    rclpy.init()
    probe = PhysicsProbe()
    try:
        probe.wait_for(
            lambda: all(name in probe.states for name in ARM),
            30.0, "measured Gazebo arm JointState")
        probe.wait_for(lambda: probe.block_pose is not None, 30.0,
                       "Gazebo blue-block model pose")
        probe.wait_for(lambda: probe.contact_frames > 0, 20.0,
                       "physical blue-block/table contact")

        joint_excursions = {}
        for name in ARM:
            for delta in (0.08, -0.08):
                baseline = probe.states[name]
                target = baseline + delta
                print(f"TEST: {name} baseline={baseline:.4f} target={target:.4f}", flush=True)
                send_arm(probe, [name], [target])
                def moved_from_baseline(n=name, b=baseline, d=delta):
                    measured = probe.states.get(n, b)
                    return (measured - b >= 0.045 if d > 0.0
                            else b - measured >= 0.045)
                probe.wait_for(
                    moved_from_baseline, 8.0,
                    f"{name} {'positive' if delta > 0 else 'negative'} physical excursion")
                moved = probe.states[name]
                joint_excursions[(name, delta)] = (baseline, moved)
                send_arm(probe, [name], [baseline])
                probe.wait_for(
                    lambda n=name, b=baseline: abs(probe.states.get(n, b) - b) < 0.035,
                    8.0, f"{name} return from measured physics")
                print(f"PASS: measured {name} target {target:.4f}: "
                      f"{baseline:.4f} -> {moved:.4f} -> {probe.states[name]:.4f} rad.",
                      flush=True)

        # A closed/open cycle tests the physical mimic joint and does not rely
        # on a rendered gripper command or planner trajectory preview.
        probe.gripper_pub.publish(Int8(data=1))
        probe.wait_for(
            lambda: probe.states.get("Claw_left_joint", -1.0) > -0.10
            and probe.states.get("claw_right_joint", 1.0) < 0.10,
            6.0, "physical two-jaw gripper close")
        closed_left = probe.states["Claw_left_joint"]
        closed_right = probe.states.get("claw_right_joint")
        probe.gripper_pub.publish(Int8(data=-1))
        probe.wait_for(
            lambda: probe.states.get("Claw_left_joint", 0.0) < -0.45
            and probe.states.get("claw_right_joint", 0.0) > 0.40,
            6.0, "physical two-jaw gripper open")
        opened_left = probe.states["Claw_left_joint"]
        opened_right = probe.states.get("claw_right_joint")
        print(f"PASS: measured gripper close={closed_left:.4f} and open={opened_left:.4f} rad.", flush=True)
        if closed_right is not None and abs(closed_right + closed_left) > 0.12:
            raise RuntimeError("Gazebo mimic gripper did not follow its leader")
        if opened_right is not None and abs(opened_right + opened_left) > 0.12:
            raise RuntimeError("Gazebo mimic gripper did not reopen symmetrically")

        block = probe.block_pose.transform.translation
        print("PASS: arm position and gripper effort controllers drive Gazebo physics.")
        print("PASS: /p2_gazebo/arm_joint_state mirrors measured Gazebo joints.")
        print(f"PASS: six-axis measured excursions: {joint_excursions}.")
        print(f"PASS: gripper measured close={closed_left:.4f} "
              f"open={opened_left:.4f} rad (right mimic={opened_right}).")
        print(f"PASS: dynamic block pose x={block.x:.4f} y={block.y:.4f} "
              f"z={block.z:.4f}; table contact frames={probe.contact_frames}.")
    except Exception as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        return 1
    finally:
        probe.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
