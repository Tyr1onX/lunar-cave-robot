"""Physical effort-control helpers for the isolated Gazebo simulation."""
from __future__ import annotations

import math

ARM_JOINTS = (
    "J1_joint", "J2_joint", "J3_joint",
    "J4_joint", "J5_joint", "J6_joint",
)
GRIPPER_JOINT = "Claw_left_joint"
GRIPPER_MIMIC_JOINT = "claw_right_joint"
GRIPPER_JOINTS = (GRIPPER_JOINT, GRIPPER_MIMIC_JOINT)

ARM_EFFORT_LIMITS = (20.0, 20.0, 12.0, 3.5, 3.5, 3.5)
ARM_MAX_VELOCITY = (1.6666, 1.666, 0.6, 2.0, 2.0, 2.0)
GRIPPER_MAX_VELOCITY = 2.5
ARM_KP = (70.0, 70.0, 45.0, 12.0, 10.0, 8.0)
ARM_KD = (5.0, 5.0, 3.5, 2.2, 2.0, 1.8)
ARM_KI = (1.0, 1.5, 1.0, 0.15, 0.12, 0.10)
ARM_LOWER = (-1.5708, -1.1345, -2.6180, -2.8798, -1.5780, -2.8798)
ARM_UPPER = (3.6652, 1.1345, 2.6180, 2.8798, 1.5708, 2.8798)
GRIPPER_EFFORT_LIMIT = 0.8
GRIPPER_KP = 10.0
GRIPPER_KD = 0.35
GRIPPER_KI = 0.05
GRIPPER_LOWER = -0.5236
GRIPPER_UPPER = 0.0
GRIPPER_MIMIC_LOWER = 0.0
GRIPPER_MIMIC_UPPER = 0.5236
ARM_INITIAL_POSITIONS = (0.0, 0.8, -2.30, 0.0, 0.0, 0.0)


def gazebo_to_optical_xyz(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Gazebo camera (+X forward,+Y left,+Z up) to ROS optical (+Z forward)."""
    return -y, -z, x


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def effort_pd(
    target: float,
    position: float,
    velocity: float,
    integral: float,
    dt: float,
    kp: float,
    kd: float,
    ki: float,
    effort_limit: float,
    integral_limit: float = 2.0,
) -> tuple[float, float]:
    """Return a saturated torque and bounded integral for one simulated joint."""
    values = (target, position, velocity, integral, dt, kp, kd, ki, effort_limit)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("effort controller inputs must be finite")
    if effort_limit <= 0.0 or dt < 0.0:
        raise ValueError("effort limit must be positive and dt non-negative")

    safe_dt = clamp(dt, 0.0, 0.02)
    error = target - position
    candidate_integral = clamp(
        integral + error * safe_dt, -integral_limit, integral_limit)
    raw = kp * error - kd * velocity + ki * candidate_integral
    saturated = clamp(raw, -effort_limit, effort_limit)

    # Anti-windup: don't integrate farther into a saturated motor.
    if raw != saturated and error * raw > 0.0:
        candidate_integral = integral
        raw = kp * error - kd * velocity + ki * candidate_integral
        saturated = clamp(raw, -effort_limit, effort_limit)
    return saturated, candidate_integral



def seed_hold_targets(targets, measured_positions, joint_names):
    """Initialize command targets from measured physics state before any command arrives."""
    seeded = dict(targets)
    for name in joint_names:
        if name not in measured_positions:
            continue
        value = float(measured_positions[name])
        if not math.isfinite(value):
            raise ValueError(f"measured joint position for {name} must be finite")
        seeded[name] = value
    return seeded


def slew_target(current: float, target: float, max_velocity: float, dt: float) -> float:
    """Move a simulated position setpoint no faster than its URDF velocity limit."""
    values = (current, target, max_velocity, dt)
    if not all(math.isfinite(float(value)) for value in values):
        raise ValueError("slew inputs must be finite")
    if max_velocity <= 0.0 or dt < 0.0:
        raise ValueError("maximum velocity must be positive and dt non-negative")
    step = max_velocity * clamp(dt, 0.0, 0.02)
    return current + clamp(target - current, -step, step)
