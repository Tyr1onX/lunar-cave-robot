import math
import unittest

from tzb_sim.gazebo_control import (
    ARM_EFFORT_LIMITS, ARM_JOINTS, ARM_LOWER, ARM_UPPER,
    GRIPPER_EFFORT_LIMIT, GRIPPER_LOWER, GRIPPER_UPPER, effort_pd,
    GRIPPER_MIMIC_LOWER, GRIPPER_MIMIC_UPPER,
    ARM_MAX_VELOCITY, GRIPPER_MAX_VELOCITY, slew_target, seed_hold_targets,
)


class GazeboControlUnitTests(unittest.TestCase):
    def test_arm_joint_order_matches_six_axis_model(self):
        self.assertEqual(len(ARM_JOINTS), 6)
        self.assertEqual(len(ARM_EFFORT_LIMITS), 6)
        self.assertEqual(len(ARM_LOWER), 6)
        self.assertEqual(len(ARM_UPPER), 6)

    def test_effort_saturates_at_urdf_joint_limit(self):
        for limit in (*ARM_EFFORT_LIMITS, GRIPPER_EFFORT_LIMIT):
            value, _ = effort_pd(100.0, 0.0, 0.0, 0.0, 0.01,
                                 1000.0, 0.0, 0.0, limit)
            self.assertAlmostEqual(value, limit)
            self.assertLessEqual(abs(value), limit)

    def test_anti_windup_and_finite_damping(self):
        value, integral = effort_pd(
            2.0, 0.0, 1.0, 1.9, 0.01, 100.0, 2.0, 10.0, 0.8)
        self.assertTrue(math.isfinite(value))
        self.assertLessEqual(abs(value), 0.8)
        self.assertLessEqual(abs(integral), 2.0)

    def test_first_feedback_seeds_position_hold_targets(self):
        targets = {"J1_joint": 0.0, "J2_joint": 0.0, "Claw_left_joint": -0.5236}
        measured = {"J1_joint": 0.42, "J2_joint": -0.31, "Claw_left_joint": -0.18}
        held = seed_hold_targets(
            targets, measured, ("J1_joint", "J2_joint", "Claw_left_joint"))
        self.assertEqual(held, measured)
        self.assertEqual(targets["J1_joint"], 0.0)

    def test_first_feedback_rejects_nonfinite_measurement(self):
        with self.assertRaises(ValueError):
            seed_hold_targets({"J1_joint": 0.0}, {"J1_joint": float("nan")},
                              ("J1_joint",))

    def test_invalid_number_rejected(self):
        with self.assertRaises(ValueError):
            effort_pd(float("nan"), 0.0, 0.0, 0.0, 0.01, 1, 1, 0, 1)

    def test_position_setpoint_slew_respects_urdf_speed_limits(self):
        target = slew_target(0.0, 1.0, ARM_MAX_VELOCITY[2], 0.01)
        self.assertAlmostEqual(target, 0.006)
        claw = slew_target(0.0, GRIPPER_LOWER, GRIPPER_MAX_VELOCITY, 0.01)
        self.assertAlmostEqual(claw, -0.025)

    def test_gripper_position_range_matches_production_urdf(self):
        self.assertAlmostEqual(GRIPPER_LOWER, -0.5236, places=4)
        self.assertAlmostEqual(GRIPPER_UPPER, 0.0, places=4)
        self.assertAlmostEqual(GRIPPER_MIMIC_LOWER, 0.0, places=4)
        self.assertAlmostEqual(GRIPPER_MIMIC_UPPER, 0.5236, places=4)


if __name__ == "__main__":
    unittest.main()



class GazeboCameraFrameTests(unittest.TestCase):
    def test_gazebo_camera_axes_convert_to_ros_optical_axes(self):
        from tzb_sim.gazebo_rgbd_optical_adapter import gazebo_to_optical_xyz
        self.assertEqual(gazebo_to_optical_xyz(0.8, -0.01, 0.07),
                         (0.01, -0.07, 0.8))
