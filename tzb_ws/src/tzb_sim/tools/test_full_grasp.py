#!/usr/bin/env python3
"""Observe the full P2 grasp passively; it never publishes control commands."""
from __future__ import annotations

import csv
import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import JointState
from std_msgs.msg import Int8
from tf2_msgs.msg import TFMessage
from yolo_node.msg import DetectionArray

ARM = ("J1_joint", "J2_joint", "J3_joint", "J4_joint", "J5_joint", "J6_joint")
OBSERVE = (0.0, 0.0, -1.85, 0.0, 1.40, 0.0)
# FK of the configured place joints with the measured object-to-TCP grasp transform.
EXPECTED_PLACE_WORLD = (-0.00656, -0.51066, 0.44316)


class FullGraspProbe(Node):
    def __init__(self) -> None:
        super().__init__("p2_full_physics_acceptance")
        self.started_at = time.monotonic()
        self.trace_rows = []
        self.last_trace_at = 0.0
        self.latest_sim_stamp = 0.0
        self.current_contact_force = 0.0
        self.max_contact_force = 0.0
        self.latest_gripper_effort = {}
        default_trace = (Path.home() / "tzb-study-output/issue48-p2-gazebo/logs" /
                         f"full_grasp_trace_{time.strftime('%Y%m%d_%H%M%S')}.csv")
        default_trace.parent.mkdir(parents=True, exist_ok=True)
        self.trace_path = Path(os.environ.get("TZB_P2_PROBE_CSV", str(default_trace)))
        self.latest_arm = {}
        self.latest_gripper = {}
        self.block = None
        self.initial_block = None
        self.max_block_z = float("-inf")
        self.block_trajectory = []
        self.detection_count = 0
        self.best_confidence = 0.0
        self.pose_count = 0
        self.pose_frame = ""
        self.first_pose = None
        self.last_pose = None
        self.finger_contact_frames = 0
        self.finger_contact = False
        self.had_finger_contact = False
        self.contact_released = False
        self.max_block_z_during_contact = float('-inf')
        self.max_block_z_during_bilateral_contact = float('-inf')
        self.last_contact_block_xyz = None
        self.block_at_release_xyz = None
        self.block_contact_surfaces = set()
        self.left_finger_contact_frames = 0
        self.right_finger_contact_frames = 0
        self.bilateral_contact_frames = 0
        self.contact_pairs_seen = set()
        self.both_finger_contact = False
        self.finger_contact_pairs = []
        self.result = None
        self.create_subscription(
            DetectionArray, "/p2_gazebo/yolo/detections", self.on_detection, 20)
        self.create_subscription(
            PoseStamped, "/p2_gazebo/pose_estimator/object_pose",
            self.on_pose, 20)
        self.create_subscription(
            JointState, "/p2_gazebo/arm_joint_state", self.on_arm, 20)
        self.create_subscription(TFMessage, "/tf", self.on_tf, 30)
        self.create_subscription(
            Contacts, "/p2_gazebo/blue_block/contacts", self.on_contacts,
            qos_profile_sensor_data)
        self.create_subscription(
            Int8, "/tzb_catch/result", self.on_result,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        command_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(
            JointState, "/joint_states", self.on_measured_joint_state, 50)
        self.create_subscription(
            JointState, "/p2_gazebo/arm_joint_command", self.on_arm_command,
            qos_profile_sensor_data)
        self.create_subscription(
            Int8, "/p2_gazebo/gripper_command", self.on_gripper_command,
            command_qos)

    def _record_trace(self, event: str, force: bool = False) -> None:
        now = time.monotonic()
        if not force and now - self.last_trace_at < 0.1:
            return
        self.last_trace_at = now
        xyz = (self.block.transform.translation if self.block is not None else None)
        row = {
            "elapsed_sec": now - self.started_at,
            "sim_stamp_sec": self.latest_sim_stamp,
            "event": event,
            "block_x": float(xyz.x) if xyz is not None else "",
            "block_y": float(xyz.y) if xyz is not None else "",
            "block_z": float(xyz.z) if xyz is not None else "",
            "finger_contact": int(self.finger_contact),
            "bilateral_contact": int(self.both_finger_contact),
            "contact_force_peak_n": self.current_contact_force,
        }
        for name in ARM:
            row[f"arm_{name}"] = self.latest_arm.get(name, "")
        for name in ("Claw_left_joint", "claw_right_joint"):
            row[f"{name}_position"] = self.latest_gripper.get(name, "")
            row[f"{name}_effort"] = self.latest_gripper_effort.get(name, "")
        self.trace_rows.append(row)

    def _write_trace(self) -> None:
        if not self.trace_rows:
            return
        fields = list(self.trace_rows[0])
        self.trace_path.parent.mkdir(parents=True, exist_ok=True)
        with self.trace_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(self.trace_rows)
        jaw_efforts = [abs(float(row[key])) for row in self.trace_rows
                       for key in ("Claw_left_joint_effort", "claw_right_joint_effort")
                       if row.get(key, "") not in ("", None)]
        peak_jaw_effort = max(jaw_efforts, default=0.0)
        print(f"TRACE_CSV={self.trace_path} rows={len(self.trace_rows)} "
              f"peak_contact_force_n={self.max_contact_force:.4f} "
              f"peak_measured_jaw_effort_nm={peak_jaw_effort:.4f}", flush=True)

    def on_measured_joint_state(self, msg: JointState) -> None:
        if len(msg.effort) == len(msg.name):
            self.latest_gripper_effort.update(zip(msg.name, msg.effort))
            self._record_trace("measured_joint_effort")

    def on_arm_command(self, msg: JointState) -> None:
        if msg.name and msg.position:
            self._record_trace("arm_command:" + ",".join(
                f"{name}={value:.4f}" for name, value in zip(msg.name, msg.position)),
                force=True)

    def on_gripper_command(self, msg: Int8) -> None:
        if msg.data == -1 and self.had_finger_contact and self.block is not None:
            p = self.block.transform.translation
            self.block_at_release_xyz = (float(p.x), float(p.y), float(p.z))
        self._record_trace(f"gripper_command:{int(msg.data)}", force=True)

    def on_detection(self, msg: DetectionArray) -> None:
        matches = [d for d in msg.detections if d.class_name == "blue_block"]
        if matches:
            self.detection_count += 1
            self.best_confidence = max(
                self.best_confidence, max(float(d.confidence) for d in matches))

    def on_pose(self, msg: PoseStamped) -> None:
        self.pose_count += 1
        self.pose_frame = msg.header.frame_id
        if self.first_pose is None:
            self.first_pose = msg
        self.last_pose = msg

    def on_arm(self, msg: JointState) -> None:
        self.latest_arm.update(zip(msg.name, msg.position))
        self.latest_gripper.update(zip(msg.name, msg.position))
        if len(msg.effort) == len(msg.name):
            self.latest_gripper_effort.update(zip(msg.name, msg.effort))
        self._record_trace("joint_state")

    @staticmethod
    def is_block(frame: str) -> bool:
        return (frame == "blue_block" or frame.endswith("::blue_block")
                or frame.endswith("blue_block::link"))

    def on_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if not self.is_block(transform.child_frame_id):
                continue
            self.block = transform
            self.latest_sim_stamp = (
                float(transform.header.stamp.sec) +
                float(transform.header.stamp.nanosec) * 1e-9)
            xyz = transform.transform.translation
            self.max_block_z = max(self.max_block_z, float(xyz.z))
            sample = (float(xyz.x), float(xyz.y), float(xyz.z))
            if not self.block_trajectory or sum(
                (sample[i] - self.block_trajectory[-1][i]) ** 2 for i in range(3)
            ) > 1e-7:
                self.block_trajectory.append(sample)
            if self.finger_contact:
                self.max_block_z_during_contact = max(
                    self.max_block_z_during_contact, sample[2])
                self.last_contact_block_xyz = sample
            if self.both_finger_contact:
                self.max_block_z_during_bilateral_contact = max(
                    self.max_block_z_during_bilateral_contact, sample[2])
            if self.initial_block is None:
                self.initial_block = sample
            self._record_trace("block_tf")

    def on_contacts(self, msg: Contacts) -> None:
        was_contact = self.finger_contact
        pairs = []
        frame_fingers = set()
        force_samples = []
        for contact in msg.contacts:
            for wrench in contact.wrenches:
                for measured in (wrench.body_1_wrench, wrench.body_2_wrench):
                    f = measured.force
                    force_samples.append((f.x * f.x + f.y * f.y + f.z * f.z) ** 0.5)
            names = (contact.collision1.name, contact.collision2.name)
            pairs.append(names)
            if not any("blue_block" in name for name in names):
                continue
            self.block_contact_surfaces.update(
                name for name in names if "blue_block" not in name)
            for name in names:
                if "Claw_left_link" in name:
                    frame_fingers.add("left")
                if "claw_right_link" in name:
                    frame_fingers.add("right")
        self.current_contact_force = max(force_samples, default=0.0)
        self.max_contact_force = max(self.max_contact_force, self.current_contact_force)
        self.finger_contact = bool(frame_fingers)
        self.both_finger_contact = frame_fingers == {"left", "right"}
        if self.finger_contact:
            self.finger_contact_frames += 1
            self.had_finger_contact = True
            self.finger_contact_pairs = list(pairs)
            self.contact_pairs_seen.update(
                name for pair in pairs for name in pair
                if "blue_block" in name or "Claw_left_link" in name or "claw_right_link" in name)
            if "left" in frame_fingers:
                self.left_finger_contact_frames += 1
            if "right" in frame_fingers:
                self.right_finger_contact_frames += 1
            if self.both_finger_contact:
                self.bilateral_contact_frames += 1
        elif was_contact:
            self.contact_released = True
        self._record_trace("contacts")

    def on_result(self, msg: Int8) -> None:
        self.result = int(msg.data)

    def observation_error(self) -> float | None:
        if not all(name in self.latest_arm for name in ARM):
            return None
        return max(abs(float(self.latest_arm[name]) - goal)
                   for name, goal in zip(ARM, OBSERVE))

    def run(self, timeout_sec: float) -> int:
        started = time.monotonic()
        next_report = started + 10.0
        while rclpy.ok() and time.monotonic() - started < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.monotonic()
            if now >= next_report:
                lift = (self.initial_block is not None and self.block is not None
                        and self.max_block_z > self.initial_block[2] + 0.03)
                print(
                    f"PROGRESS t={now-started:.0f}s detections={self.detection_count} "
                    f"poses={self.pose_count} lift={lift} "
                    f"finger_contact={self.finger_contact} result={self.result}",
                    flush=True)
                next_report = now + 10.0
            lift_height = (self.initial_block[2] + 0.03
                           if self.initial_block is not None else float("inf"))
            lifted_during_contact = self.max_block_z_during_contact > lift_height
            lifted_during_bilateral_contact = (
                self.max_block_z_during_bilateral_contact > lift_height)
            released = self.had_finger_contact and self.contact_released and not self.finger_contact
            arm_error = self.observation_error()
            jaws_open = (
                self.latest_gripper.get("Claw_left_joint", 0.0) < -0.40 and
                self.latest_gripper.get("claw_right_joint", 0.0) > 0.40)
            if (self.result is not None and jaws_open
                    and arm_error is not None and arm_error < 0.08):
                final_xyz = self.block.transform.translation
                displacement = sum(
                    (float(value) - self.initial_block[i]) ** 2
                    for i, value in enumerate((final_xyz.x, final_xyz.y, final_xyz.z))
                ) ** 0.5
                checks = {
                    "openvino_blue_block_detection": self.detection_count > 0,
                    "pose_estimator_optical_pose": (
                        self.pose_count > 0 and self.pose_frame == "sim_camera_optical_frame"
                        and self.first_pose is not None
                        and abs(float(self.first_pose.pose.position.z) - 0.714) < 0.35
                    ),
                    "gazebo_bilateral_finger_contact": (
                        self.left_finger_contact_frames > 0 and
                        self.right_finger_contact_frames > 0 and
                        self.bilateral_contact_frames > 0
                    ),
                    "physical_pickup_while_bilateral_contact": lifted_during_bilateral_contact,
                    "carried_to_place_before_release": (
                        self.block_at_release_xyz is not None and
                        sum((self.block_at_release_xyz[i] - EXPECTED_PLACE_WORLD[i]) ** 2
                            for i in range(3)) ** 0.5 < 0.12
                    ),
                    "physical_release_and_relocation": released and displacement > 0.10,
                    "placed_on_gazebo_pad": (
                        abs(float(final_xyz.z) - 0.035) < 0.012 and
                        abs(float(final_xyz.x) - 0.13) < 0.225 and
                        abs(float(final_xyz.y) + 0.54) < 0.095 and
                        any("place_pad" in name for name in self.block_contact_surfaces)
                    ),
                    "planner_returned_to_observation": arm_error < 0.08,
                    "gripper_open_feedback": jaws_open,
                    "planner_success_result": self.result == 1,
                }
                for name, passed in checks.items():
                    print(f"{'PASS' if passed else 'FAIL'}: {name}", flush=True)
                print(
                    f"physical block start={self.initial_block} final="
                    f"({final_xyz.x:.4f}, {final_xyz.y:.4f}, {final_xyz.z:.4f}) "
                    f"max_z={self.max_block_z:.4f} "
                    f"max_z_during_contact={self.max_block_z_during_contact:.4f} "
                    f"max_z_during_bilateral_contact={self.max_block_z_during_bilateral_contact:.4f} "
                    f"release_xyz={self.block_at_release_xyz} "
                    f"last_contact_xyz={self.last_contact_block_xyz} "
                    f"block_contact_surfaces={sorted(self.block_contact_surfaces)} "
                    f"path_samples={len(self.block_trajectory)} "
                    f"contact_frames={self.finger_contact_frames} "
                    f"left={self.left_finger_contact_frames} "
                    f"right={self.right_finger_contact_frames} "
                    f"bilateral={self.bilateral_contact_frames} "
                    f"pairs={sorted(self.contact_pairs_seen)}", flush=True)
                self._write_trace()
                print(
                    f"detections={self.detection_count} best_confidence="
                    f"{self.best_confidence:.3f}; poses={self.pose_count} "
                    f"frame={self.pose_frame}; observe_max_error={arm_error:.4f} rad",
                    flush=True)
                return 0 if all(checks.values()) else 1
        print("FAIL: timed out before successful physical pick/place/return.", file=sys.stderr)
        self._write_trace()
        print(
            f"state result={self.result} detected={self.detection_count} "
            f"poses={self.pose_count} finger_contact_frames={self.finger_contact_frames} "
            f"initial={self.initial_block} max_z={self.max_block_z} "
            f"max_z_during_contact={self.max_block_z_during_contact} "
            f"max_z_during_bilateral={self.max_block_z_during_bilateral_contact} "
            f"last_contact={self.last_contact_block_xyz} "
            f"left={self.left_finger_contact_frames} right={self.right_finger_contact_frames} "
            f"bilateral={self.bilateral_contact_frames} "
            f"released={self.contact_released} result={self.result} "
            f"arm_error={self.observation_error()}", file=sys.stderr)
        return 1


def main() -> int:
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
    rclpy.init()
    node = FullGraspProbe()
    try:
        return node.run(timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
