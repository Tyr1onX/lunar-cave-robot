"""Strict SIMULATION-only contract for the offline arm pose viewer."""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path

HOST = "127.0.0.1"
PORT = 8767
FOXGLOVE_PORT = 8768
DOMAIN = 218
JOINT_TOPIC = "/arm_preview/joint_states"
MODE = "SIMULATION"
SOURCE = "isolated-arm-joint-state-simulation"
MAX_SEQUENCE = 2**53


def _number(value: str, label: str) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"invalid {label}") from exc
    if not math.isfinite(result):
        raise ValueError(f"invalid {label}")
    return result


def _vector(text: str | None, label: str) -> list[float]:
    if text is None:
        return [0.0, 0.0, 0.0]
    values = [_number(value, label) for value in text.split()]
    if len(values) != 3:
        raise ValueError(f"invalid {label}")
    return values


def load_model_contract(urdf_path: Path, srdf_path: Path) -> dict:
    """Read joint order, bounds, fixed frames and meshes from the existing model."""
    urdf_bytes = urdf_path.read_bytes()
    urdf = ET.fromstring(urdf_bytes)
    srdf = ET.parse(srdf_path).getroot()
    links = {element.get("name") for element in urdf.findall("link")}
    if not links or urdf.get("name") != "tzb_description":
        raise ValueError("unexpected robot description")

    joints = {element.get("name"): element for element in urdf.findall("joint")}
    children = {element.find("child").get("link"): element for element in joints.values()}
    groups = {element.get("name"): element for element in srdf.findall("group")}
    arm = groups.get("arm")
    chain = arm.find("chain") if arm is not None else None
    if chain is None:
        raise ValueError("SRDF arm chain is missing")
    base_link = chain.get("base_link")
    tip_link = chain.get("tip_link")
    if base_link not in links or tip_link not in links:
        raise ValueError("SRDF arm chain references an unknown link")

    arm_joints = []
    link = tip_link
    while link != base_link:
        joint = children.get(link)
        if joint is None:
            raise ValueError("URDF does not connect the SRDF arm chain")
        if joint.find("parent").get("link") not in links:
            raise ValueError("URDF joint references an unknown parent link")
        arm_joints.append(joint)
        link = joint.find("parent").get("link")
        if len(arm_joints) > len(joints):
            raise ValueError("URDF arm chain contains a cycle")
    arm_joints.reverse()
    if len(arm_joints) != 6 or any(joint.get("type") != "revolute" for joint in arm_joints):
        raise ValueError("the SRDF arm group must resolve to six revolute joints")

    joint_specs = []
    for joint in arm_joints:
        limit = joint.find("limit")
        axis = _vector(joint.find("axis").get("xyz") if joint.find("axis") is not None else None, "joint axis")
        lower = _number(limit.get("lower"), "lower limit") if limit is not None else math.nan
        upper = _number(limit.get("upper"), "upper limit") if limit is not None else math.nan
        if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
            raise ValueError(f"invalid limits for {joint.get('name')}")
        if abs(math.sqrt(sum(component * component for component in axis)) - 1.0) > 1e-6:
            raise ValueError(f"non-unit axis for {joint.get('name')}")
        joint_specs.append({
            "name": joint.get("name"),
            "lower": lower,
            "upper": upper,
            "axis": axis,
        })

    gripper = groups.get("zhua")
    gripper_joint_names = [element.get("name") for element in gripper.findall("joint")] if gripper is not None else []
    if len(gripper_joint_names) != 2 or any(name not in joints for name in gripper_joint_names):
        raise ValueError("SRDF gripper group must reference two existing joints")
    gripper_specs = {}
    for name in gripper_joint_names:
        joint = joints[name]
        limit = joint.find("limit")
        if joint.get("type") != "revolute" or limit is None:
            raise ValueError(f"invalid SRDF gripper joint {name}")
        gripper_specs[name] = {
            "lower": _number(limit.get("lower"), "gripper lower limit"),
            "upper": _number(limit.get("upper"), "gripper upper limit"),
        }

    end_effector = next((item for item in srdf.findall("end_effector") if item.get("name") == "hand"), None)
    if end_effector is None or end_effector.get("group") != "zhua":
        raise ValueError("SRDF hand end effector is missing")

    meshes = set()
    for mesh in urdf.findall(".//mesh"):
        uri = mesh.get("filename", "")
        prefix = "package://tzb_description/meshes/"
        if not uri.startswith(prefix):
            raise ValueError(f"unexpected mesh URI {uri}")
        relative = uri.removeprefix(prefix)
        if not relative or "/" in relative or "\\" in relative:
            raise ValueError(f"unexpected mesh path {uri}")
        mesh_path = urdf_path.parent.parent / "meshes" / relative
        if not mesh_path.is_file() or mesh_path.stat().st_size <= 84:
            raise ValueError(f"missing or invalid STL asset {relative}")
        meshes.add(relative)

    fixed_frames = {}
    for name in ("camera_joint", "tcp_joint"):
        joint = joints.get(name)
        if joint is None or joint.get("type") != "fixed":
            raise ValueError(f"URDF fixed frame {name} is missing")
        origin = joint.find("origin")
        fixed_frames[name] = {
            "parent": joint.find("parent").get("link"),
            "child": joint.find("child").get("link"),
            "xyz": _vector(origin.get("xyz") if origin is not None else None, f"{name} xyz"),
            "rpy": _vector(origin.get("rpy") if origin is not None else None, f"{name} rpy"),
        }

    return {
        "robot_name": urdf.get("name"),
        "urdf_sha256": hashlib.sha256(urdf_bytes).hexdigest(),
        "base_link": base_link,
        "tip_link": tip_link,
        "arm_joints": joint_specs,
        "gripper_joints": gripper_specs,
        "srdf_hand_parent_link": end_effector.get("parent_link"),
        "fixed_frames": fixed_frames,
        "mesh_count": len(meshes),
        "mesh_names": sorted(meshes),
    }


def _finite_axis(value, label: str) -> float:
    if type(value) not in (int, float):
        raise ValueError(f"invalid {label}")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"invalid {label}")
    return result


def validate_command(body: object, *, token: str | None, last_sequence: int,
                     model: dict) -> tuple[int, dict[str, float], float]:
    """Validate an entire pose update; reject partial, stale or out-of-limit data."""
    if not isinstance(body, dict) or set(body) != {"token", "seq", "joints", "gripper_opening"}:
        raise ValueError("invalid command shape")
    if not isinstance(body["token"], str) or not token or body["token"] != token:
        raise ValueError("invalid simulation token")
    sequence = body["seq"]
    if type(sequence) is not int or not last_sequence < sequence <= MAX_SEQUENCE:
        raise ValueError("invalid or stale sequence")
    positions = body["joints"]
    expected = {joint["name"] for joint in model["arm_joints"]}
    if not isinstance(positions, dict) or set(positions) != expected:
        raise ValueError("invalid joint set")
    checked = {}
    for joint in model["arm_joints"]:
        value = _finite_axis(positions[joint["name"]], joint["name"])
        if not joint["lower"] <= value <= joint["upper"]:
            raise ValueError(f"{joint['name']} exceeds URDF limits")
        checked[joint["name"]] = value
    opening = _finite_axis(body["gripper_opening"], "gripper opening")
    if not 0.0 <= opening <= 1.0:
        raise ValueError("gripper opening outside 0..1")
    return sequence, checked, opening


def gripper_positions(model: dict, opening: float) -> dict[str, float]:
    """Map the preview slider to symmetric motion within the existing SRDF jaws."""
    names = model["gripper_joints"]
    left = names.get("Claw_left_joint")
    right = names.get("claw_right_joint")
    if left is None or right is None:
        raise ValueError("expected left and right gripper joints")
    amplitude = min(left["upper"] - left["lower"], right["upper"] - right["lower"])
    # Existing URDF bounds are [-0.5236, 0] and [0, 0.5236]. The mirrored
    # movement direction is a preview convention; physical open/close polarity
    # has not been verified on the actual gripper.
    return {
        "Claw_left_joint": max(left["lower"], min(left["upper"], -opening * amplitude)),
        "claw_right_joint": max(right["lower"], min(right["upper"], opening * amplitude)),
    }
