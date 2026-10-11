#!/usr/bin/env python3
"""Offline forward-kinematics checks for the default tzb_description URDF."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

URDF = Path(__file__).resolve().parents[3] / "tzb_description" / "urdf" / "tzb_description.urdf"
JOINTS = [
    ("J1_joint", "base_link", "J1_link", (0.0, 0.0, 1.0), -1.5708, 3.6652),
    ("J2_joint", "J1_link", "J2_link", (1.0, 0.0, 0.0), -1.1345, 1.1345),
    ("J3_joint", "J2_link", "J3_link", (1.0, 0.0, 0.0), -2.618, 2.618),
    ("J4_joint", "J3_link", "J4_link", (0.0, 0.0, 1.0), -2.8798, 2.8798),
    ("J5_joint", "J4_link", "J5_link", (-1.0, 0.0, 0.0), -1.578, 1.5708),
    ("J6_joint", "J5_link", "J6_link", (0.0, 0.0, 1.0), -2.8798, 2.8798),
]


def ident():
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def mm(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, 0.0],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, 0.0],
        [-sp, cp * sr, cp * cr, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]


def origin_transform(node):
    origin = node.find("origin")
    t = ident()
    if origin is None:
        return t
    xyz = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    rpy = [float(v) for v in origin.get("rpy", "0 0 0").split()]
    r = rpy_matrix(*rpy)
    for i in range(3):
        r[i][3] = xyz[i]
    return r


def axis_rotation(axis, angle):
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1.0 - math.cos(angle)
    r = ident()
    r[0][:3] = [x * x * v + c, x * y * v - z * s, x * z * v + y * s]
    r[1][:3] = [y * x * v + z * s, y * y * v + c, y * z * v - x * s]
    r[2][:3] = [z * x * v - y * s, z * y * v + x * s, z * z * v + c]
    return r


def load_chain():
    root = ET.parse(URDF).getroot()
    joints = {j.get("name"): j for j in root.findall("joint")}
    chain = []
    for name, parent, child, axis, lower, upper in JOINTS:
        joint = joints[name]
        assert joint.get("type") == "revolute", name
        assert joint.find("parent").get("link") == parent, name
        assert joint.find("child").get("link") == child, name
        actual_axis = tuple(float(v) for v in joint.find("axis").get("xyz").split())
        assert actual_axis == axis, (name, actual_axis, axis)
        limits = joint.find("limit")
        assert abs(float(limits.get("lower")) - lower) < 1e-6, name
        assert abs(float(limits.get("upper")) - upper) < 1e-6, name
        assert abs(math.sqrt(sum(v * v for v in actual_axis)) - 1.0) < 1e-8, name
        chain.append((joint, actual_axis))
    return chain


def fk(chain, q):
    t = ident()
    for i, (joint, axis) in enumerate(chain):
        t = mm(t, origin_transform(joint))
        t = mm(t, axis_rotation(axis, q[i]))
    return t


def point(t, p):
    return [sum(t[i][j] * p[j] for j in range(3)) + t[i][3] for i in range(3)]


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def norm(v):
    return math.sqrt(sum(x * x for x in v))


def test_fk_poses_and_axis_signs():
    chain = load_chain()
    zero = fk(chain, [0.0] * 6)
    q = [0.25, -0.2, 0.4, -0.3, 0.15, 0.5]
    moved = fk(chain, q)
    assert all(math.isfinite(x) for row in moved for x in row)
    assert norm(sub([moved[i][3] for i in range(3)], [zero[i][3] for i in range(3)])) > 1e-3

    q0 = [0.0] * 6
    point_local = [0.13, 0.07, 0.11]
    eps = 1e-5
    for k, (joint, axis) in enumerate(chain):
        t = ident()
        joint_origin = axis_world = None
        for i, (item, local_axis) in enumerate(chain):
            t = mm(t, origin_transform(item))
            if i == k:
                joint_origin = [t[r][3] for r in range(3)]
                axis_world = [sum(t[r][c] * local_axis[c] for c in range(3)) for r in range(3)]
            t = mm(t, axis_rotation(local_axis, q0[i]))
        p0 = point(fk(chain, q0), point_local)
        q1 = list(q0)
        q1[k] = eps
        p1 = point(fk(chain, q1), point_local)
        numerical = [(p1[i] - p0[i]) / eps for i in range(3)]
        expected = cross(axis_world, sub(p0, joint_origin))
        assert norm(sub(numerical, expected)) < 2e-5, (joint.get("name"), numerical, expected)


if __name__ == "__main__":
    test_fk_poses_and_axis_signs()
    print("URDF FK checks passed: 6 axes, zero/nonzero poses, positive perturbation signs")
