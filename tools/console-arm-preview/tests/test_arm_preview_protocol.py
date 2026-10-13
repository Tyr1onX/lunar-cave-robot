#!/usr/bin/env python3
"""Offline contract checks; no ROS graph or physical device is started."""

import math
import sys
import unittest
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
ROOT = TOOLS.parents[1]
sys.path.insert(0, str(TOOLS))

from arm_preview_protocol import gripper_positions, load_model_contract, validate_command  # noqa: E402


URDF = ROOT / "tzb_ws/src/tzb_description/urdf/tzb_description.urdf"
SRDF = ROOT / "tzb_ws/src/lite_motion_planner/config/tzb_description.srdf"


class ArmPreviewProtocolTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = load_model_contract(URDF, SRDF)

    def command(self, **updates):
        joints = {joint["name"]: 0.0 for joint in self.model["arm_joints"]}
        body = {"token": "a" * 32, "seq": 1, "joints": joints, "gripper_opening": 0.0}
        body.update(updates)
        return body

    def test_existing_srdf_chain_resolves_to_six_urdf_joints_and_assets(self):
        self.assertEqual(self.model["base_link"], "base_link")
        self.assertEqual(self.model["tip_link"], "J6_link")
        self.assertEqual([joint["name"] for joint in self.model["arm_joints"]], [
            "J1_joint", "J2_joint", "J3_joint", "J4_joint", "J5_joint", "J6_joint",
        ])
        self.assertEqual(self.model["mesh_count"], 10)
        self.assertEqual(set(self.model["gripper_joints"]), {"Claw_left_joint", "claw_right_joint"})
        self.assertEqual(self.model["srdf_hand_parent_link"], "J6_link")

    def test_camera_and_tcp_use_existing_unverified_fixed_transforms(self):
        camera = self.model["fixed_frames"]["camera_joint"]
        tcp = self.model["fixed_frames"]["tcp_joint"]
        self.assertEqual((camera["parent"], camera["child"]), ("J3_link", "camera_link"))
        self.assertEqual((tcp["parent"], tcp["child"]), ("J6_link", "tcp_link"))
        self.assertAlmostEqual(camera["xyz"][0], -0.03)
        self.assertAlmostEqual(camera["xyz"][1], -0.155288674811394)
        self.assertAlmostEqual(camera["xyz"][2], 0.274492525428163)
        self.assertAlmostEqual(tcp["xyz"][0], -0.015)
        self.assertAlmostEqual(tcp["xyz"][1], 0.001)
        self.assertAlmostEqual(tcp["xyz"][2], 0.11)

    def test_command_accepts_full_pose_only_within_urdf_bounds(self):
        body = self.command(gripper_opening=0.4)
        sequence, joints, opening = validate_command(
            body, token="a" * 32, last_sequence=0, model=self.model
        )
        self.assertEqual(sequence, 1)
        self.assertEqual(set(joints), {joint["name"] for joint in self.model["arm_joints"]})
        self.assertEqual(opening, 0.4)

    def test_rejects_stale_partial_nonfinite_boolean_and_out_of_range_commands(self):
        checks = []
        checks.append((self.command(), "a" * 32, 1))
        checks.append((self.command(joints={"J1_joint": 0.0}), "a" * 32, 0))
        checks.append((self.command(joints={**{j["name"]: 0.0 for j in self.model["arm_joints"]}, "J1_joint": float("nan")}), "a" * 32, 0))
        checks.append((self.command(joints={**{j["name"]: 0.0 for j in self.model["arm_joints"]}, "J1_joint": True}), "a" * 32, 0))
        checks.append((self.command(gripper_opening=1.01), "a" * 32, 0))
        checks.append((self.command(), "b" * 32, 0))
        invalid_joint = {joint["name"]: 0.0 for joint in self.model["arm_joints"]}
        invalid_joint["J2_joint"] = self.model["arm_joints"][1]["upper"] + 0.01
        checks.append((self.command(joints=invalid_joint), "a" * 32, 0))
        for body, token, last_sequence in checks:
            with self.subTest(body=body, last_sequence=last_sequence):
                with self.assertRaises(ValueError):
                    validate_command(body, token=token, last_sequence=last_sequence, model=self.model)

    def test_gripper_preview_is_mirrored_and_stays_inside_urdf_bounds(self):
        closed = gripper_positions(self.model, 0.0)
        half = gripper_positions(self.model, 0.5)
        opened = gripper_positions(self.model, 1.0)
        self.assertEqual(closed, {"Claw_left_joint": 0.0, "claw_right_joint": 0.0})
        self.assertAlmostEqual(half["Claw_left_joint"], -half["claw_right_joint"])
        self.assertLess(opened["Claw_left_joint"], 0.0)
        self.assertGreater(opened["claw_right_joint"], 0.0)
        specs = self.model["gripper_joints"]
        for pose in (closed, half, opened):
            for name, value in pose.items():
                self.assertTrue(specs[name]["lower"] <= value <= specs[name]["upper"])
                self.assertTrue(math.isfinite(value))


if __name__ == "__main__":
    unittest.main(verbosity=2)
