#!/usr/bin/env python3
"""Publish bounded fake JointState data for the isolated Foxglove arm preview."""

from __future__ import annotations

import json
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState

from arm_preview_protocol import (
    DOMAIN,
    HOST,
    JOINT_TOPIC,
    MODE,
    PORT,
    SOURCE,
    gripper_positions,
    load_model_contract,
    validate_command,
)

ROOT = Path(__file__).resolve().parents[2]
URDF = ROOT / "tzb_ws/src/tzb_description/urdf/tzb_description.urdf"
SRDF = ROOT / "tzb_ws/src/lite_motion_planner/config/tzb_description.srdf"


class ArmPreview(Node):
    def __init__(self, model: dict):
        super().__init__("lunar_console_arm_preview", enable_rosout=False, start_parameter_services=False)
        self.model = model
        self._lock = threading.RLock()
        self._token: str | None = None
        self._sequence = 0
        self._opening = 0.0
        self._positions = {joint["name"]: 0.0 for joint in model["arm_joints"]}
        self._publisher = self.create_publisher(JointState, JOINT_TOPIC, 10)
        self._timer = self.create_timer(0.05, self.publish_joint_state)

    def publish_joint_state(self) -> None:
        with self._lock:
            positions = dict(self._positions)
            positions.update(gripper_positions(self.model, self._opening))
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(positions)
        message.position = list(positions.values())
        self._publisher.publish(message)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "schema_version": 1,
                "source": SOURCE,
                "mode": MODE,
                "domain_id": DOMAIN,
                "localhost_only": True,
                "ready": True,
                "joint_topic": JOINT_TOPIC,
                "urdf_sha256": self.model["urdf_sha256"],
                "base_link": self.model["base_link"],
                "tip_link": self.model["tip_link"],
                "joints": [
                    {**joint, "position": self._positions[joint["name"]]}
                    for joint in self.model["arm_joints"]
                ],
                "gripper_opening": self._opening,
                "gripper_joints": self.model["gripper_joints"],
                "srdf_hand_parent_link": self.model["srdf_hand_parent_link"],
                "fixed_frames": self.model["fixed_frames"],
                "mesh_count": self.model["mesh_count"],
                "mesh_names": self.model["mesh_names"],
                "note": "URDF preview only; model zero, camera pose, TCP, and gripper direction are not physically calibrated.",
            }

    def claim(self) -> str:
        with self._lock:
            self._positions = {joint["name"]: 0.0 for joint in self.model["arm_joints"]}
            self._opening = 0.0
            self._token = secrets.token_hex(16)
            self._sequence = 0
            return self._token

    def command(self, body: object) -> int:
        with self._lock:
            sequence, positions, opening = validate_command(
                body, token=self._token, last_sequence=self._sequence, model=self.model
            )
            self._positions = positions
            self._opening = opening
            self._sequence = sequence
            return sequence

    def release(self, body: object) -> None:
        if not isinstance(body, dict) or set(body) != {"token"}:
            raise ValueError("invalid release shape")
        with self._lock:
            if not self._token or body["token"] != self._token:
                raise ValueError("invalid simulation token")
            self._positions = {joint["name"]: 0.0 for joint in self.model["arm_joints"]}
            self._opening = 0.0
            self._token = None
            self._sequence = 0


def make_handler(node: ArmPreview):
    class Handler(BaseHTTPRequestHandler):
        server_version = "LunarArmPreview/1"

        def log_message(self, _format, *_args):
            return

        def _send(self, status: int, value: dict) -> None:
            data = json.dumps(value, separators=(",", ":")).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("Connection", "close")
            self.end_headers()
            self.wfile.write(data)

        def _body(self) -> object:
            raw_length = self.headers.get("Content-Length", "")
            if not raw_length.isascii() or not raw_length.isdigit():
                raise ValueError("invalid content length")
            length = int(raw_length)
            if not 1 <= length <= 8192:
                raise ValueError("invalid request size")
            raw = self.rfile.read(length)
            try:
                return json.loads(raw)
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise ValueError("invalid JSON") from exc

        def do_GET(self):
            if self.path != "/v1/arm-preview/state":
                self._send(404, {"error": "not found"})
                return
            self._send(200, node.snapshot())

        def do_POST(self):
            routes = {
                "/v1/arm-preview/claim",
                "/v1/arm-preview/command",
                "/v1/arm-preview/release",
            }
            if self.path not in routes:
                self._send(404, {"error": "not found"})
                return
            try:
                body = self._body()
                if self.path.endswith("/claim"):
                    if body != {}:
                        raise ValueError("claim body must be empty")
                    self._send(200, {"token": node.claim(), "mode": MODE})
                elif self.path.endswith("/command"):
                    sequence = node.command(body)
                    self._send(200, {"applied": True, "seq": sequence, "mode": MODE})
                else:
                    node.release(body)
                    self._send(200, {"released": True, "mode": MODE})
            except ValueError as exc:
                self._send(400, {"error": str(exc)})

        def do_PUT(self):
            self._send(405, {"error": "method not allowed"})

        def do_DELETE(self):
            self._send(405, {"error": "method not allowed"})

        def do_OPTIONS(self):
            self._send(405, {"error": "method not allowed"})

    return Handler


def main() -> int:
    if rclpy.ok():
        raise RuntimeError("arm preview must run in its own ROS process")
    rclpy.init(args=None)
    model = load_model_contract(URDF, SRDF)
    node = ArmPreview(model)
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, name="arm-preview-ros", daemon=True)
    ros_thread.start()
    server = ThreadingHTTPServer((HOST, PORT), make_handler(node))
    server.daemon_threads = True
    print(
        f"SIMULATION only: ROS_DOMAIN_ID={DOMAIN}, ROS_LOCALHOST_ONLY=1, "
        f"publishing {JOINT_TOPIC} on 127.0.0.1:{PORT}",
        flush=True,
    )
    try:
        server.serve_forever(poll_interval=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
