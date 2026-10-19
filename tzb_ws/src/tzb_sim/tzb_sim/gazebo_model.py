"""Build a simulation-only robot description from the production URDF."""
from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from pathlib import Path

from .gazebo_control import (
    ARM_JOINTS, ARM_INITIAL_POSITIONS, GRIPPER_JOINT, GRIPPER_LOWER,
    GRIPPER_MIMIC_JOINT, GRIPPER_MIMIC_UPPER,
)


def _add(parent, tag, text=None, **attributes):
    item = ET.SubElement(parent, tag, attributes)
    if text is not None:
        item.text = str(text)
    return item


def build_gazebo_urdf(
    source_urdf: str,
    controller_yaml: str,
    *,
    camera_frame: str = "sim_camera_link",
    image_topic: str = "/p2_gazebo/rgbd_camera",
    preserve_package_meshes: bool = False,
) -> str:
    """Keep original geometry/joints and add only P2 simulation systems."""
    root = ET.parse(source_urdf).getroot()
    if root.tag != "robot":
        raise ValueError("expected a robot root in source URDF")

    # Gazebo's URDF parser cannot reliably resolve ROS package:// URLs when the
    # model is spawned as a string, so make only the generated simulation copy
    # use absolute asset paths. The source URDF remains unchanged.
    share = Path(source_urdf).resolve().parent.parent
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename", "")
        prefix = "package://tzb_description/"
        if filename.startswith(prefix) and not preserve_package_meshes:
            mesh.set("filename", str((share / filename[len(prefix):]).resolve()))

    if root.find(f"link[@name='{camera_frame}']") is None:
        _add(root, "link", name=camera_frame)
        camera_joint = _add(root, "joint", name="tzb_sim_camera_joint", type="fixed")
        _add(camera_joint, "origin", xyz="0 0 0.60", rpy="0 0.910174 0.463648")
        _add(camera_joint, "parent", link="base_link")
        _add(camera_joint, "child", link=camera_frame)

    optical_frame = "sim_camera_optical_frame"
    if root.find(f"link[@name='{optical_frame}']") is None:
        _add(root, "link", name=optical_frame)
        optical_joint = _add(
            root, "joint", name="tzb_sim_camera_optical_joint", type="fixed")
        _add(optical_joint, "origin", xyz="0 0 0",
             rpy="-1.57079632679 0 -1.57079632679")
        _add(optical_joint, "parent", link=camera_frame)
        _add(optical_joint, "child", link=optical_frame)

    gazebo_sensor = _add(root, "gazebo", reference=camera_frame)
    sensor = _add(gazebo_sensor, "sensor", name="p2_rgbd_camera", type="rgbd_camera")
    _add(sensor, "always_on", "true")
    _add(sensor, "update_rate", "10")
    _add(sensor, "visualize", "true")
    _add(sensor, "topic", image_topic)
    _add(sensor, "ignition_frame_id", camera_frame)
    camera = _add(sensor, "camera")
    _add(camera, "horizontal_fov", "1.047")
    image = _add(camera, "image")
    _add(image, "width", "640")
    _add(image, "height", "480")
    _add(camera, "clip")
    clip = camera.find("clip")
    _add(clip, "near", "0.05")
    _add(clip, "far", "4.0")

    # Add modest motor/joint damping to the simulation-only copy. The source URDF
    # has no actuator damping and its very light wrist links otherwise oscillate
    # against effort saturation under lunar gravity.
    for joint_name, damping in zip(ARM_JOINTS, (0.05, 0.05, 0.05, 0.10, 0.10, 0.10)):
        source_joint = root.find(f"joint[@name='{joint_name}']")
        if source_joint is None:
            raise ValueError(f"URDF has no arm joint {joint_name}")
        dynamics = source_joint.find("dynamics")
        if dynamics is None:
            dynamics = _add(source_joint, "dynamics")
        dynamics.set("damping", f"{damping:.3f}")

    # Raise gripper/cube friction in the simulation copy only. Contacts still
    # resolve through Gazebo's physics engine; no attach/teleport plugin exists.
    for link_name in ("Claw_left_link", "claw_right_link"):
        friction = _add(root, "gazebo", reference=link_name)
        _add(friction, "mu1", "1.8")
        _add(friction, "mu2", "1.8")

    control = _add(root, "ros2_control", name="TZBGazeboSystem", type="system")
    hardware = _add(control, "hardware")
    _add(hardware, "plugin", "gz_ros2_control/GazeboSimSystem")

    limits = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if limit is not None:
            limits[name] = limit.attrib

    initial_positions = dict(zip(ARM_JOINTS, ARM_INITIAL_POSITIONS))
    initial_positions[GRIPPER_JOINT] = GRIPPER_LOWER
    initial_positions[GRIPPER_MIMIC_JOINT] = GRIPPER_MIMIC_UPPER
    for joint_name in (*ARM_JOINTS, GRIPPER_JOINT, GRIPPER_MIMIC_JOINT):
        if joint_name not in limits:
            raise ValueError(f"URDF has no joint limit for {joint_name}")
        if joint_name == "claw_right_joint":
            # Fortress does not enforce URDF mimic constraints in physics. Keep
            # the production source untouched and drive the mirrored virtual jaw
            # with its own effort interface and measured joint feedback.
            source_joint = root.find(f"joint[@name='{joint_name}']")
            mimic = source_joint.find("mimic") if source_joint is not None else None
            if mimic is not None:
                source_joint.remove(mimic)
        sim_joint = _add(control, "joint", name=joint_name)
        limit = limits[joint_name]
        if joint_name in ARM_JOINTS:
            command = _add(sim_joint, "command_interface", name="position")
            _add(command, "param", limit["lower"], name="min")
            _add(command, "param", limit["upper"], name="max")
        else:
            command = _add(sim_joint, "command_interface", name="effort")
            _add(command, "param", str(-float(limit["effort"])), name="min")
            _add(command, "param", limit["effort"], name="max")
        for interface in ("position", "velocity", "effort"):
            state = _add(sim_joint, "state_interface", name=interface)
            if interface == "position":
                _add(state, "param", f"{initial_positions[joint_name]:.6f}", name="initial_value")

    gazebo_plugin = _add(root, "gazebo")
    plugin = _add(
        gazebo_plugin, "plugin",
        name="gz_ros2_control::GazeboSimROS2ControlPlugin",
        filename="libgz_ros2_control-system.so")
    _add(plugin, "parameters", str(Path(controller_yaml).resolve()))
    _add(plugin, "position_proportional_gain", "0.10")
    _add(plugin, "hold_joints", "true")
    _add(plugin, "robot_param_node", "robot_state_publisher")
    _add(plugin, "robot_param", "robot_description")

    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"
