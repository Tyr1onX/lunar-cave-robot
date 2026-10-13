"""Loopback-only V0.3a bridge around the existing P6 CaveSimulator. No hardware."""
import json
import math
import secrets
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from p6_autonomous_mission.cave_simulator import CaveSimulator

from sim_protocol import (HOST, PORT, DOMAIN, COMMAND, ODOM, MAP, SCAN,
                          MAX_LINEAR, MAX_TURN, require_isolation, validate_command)


class SimBridge(Node):
    def __init__(self):
        super().__init__("console_sim_bridge", enable_rosout=False, start_parameter_services=False)
        self._lock = threading.Lock()
        self._token = None
        self._seq = 0
        self._odom = None
        self._map = None
        self._last_odom = 0.0
        self._trail = []
        self.pub = self.create_publisher(Twist, COMMAND, 1)
        self.create_subscription(Odometry, ODOM, self._on_odom, 10)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, MAP, self._on_map, qos)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        value = {"x": msg.pose.pose.position.x, "y": msg.pose.pose.position.y,
                 "yaw": yaw, "linear": msg.twist.twist.linear.x,
                 "angular": msg.twist.twist.angular.z}
        with self._lock:
            self._odom = value
            self._last_odom = time.monotonic()
            point = [value["x"], value["y"]]
            if not self._trail or math.dist(point, self._trail[-1]) >= 0.02:
                self._trail.append(point)
                del self._trail[:-800]

    def _on_map(self, msg):
        with self._lock:
            self._map = {"width": msg.info.width, "height": msg.info.height,
                         "resolution": msg.info.resolution,
                         "origin_x": msg.info.origin.position.x,
                         "origin_y": msg.info.origin.position.y,
                         "cells": list(msg.data)}

    def _publish(self, forward, turn):
        msg = Twist()
        msg.linear.x = MAX_LINEAR * forward
        msg.angular.z = MAX_TURN * turn
        self.pub.publish(msg)

    def claim(self):
        with self._lock:
            self._publish(0, 0)
            self._token = secrets.token_hex(16)
            self._seq = 0
            return self._token

    def command(self, body):
        token, seq, forward, turn = validate_command(body)
        with self._lock:
            if token != self._token or seq <= self._seq:
                return False
            self._seq = seq
            self._publish(forward, turn)
            return True

    def release(self, token):
        with self._lock:
            if not isinstance(token, str) or token != self._token:
                return False
            self._token = None
            self._publish(0, 0)
            return True

    def state(self):
        with self._lock:
            fresh = self._odom is not None and time.monotonic() - self._last_odom < 0.5
            return {"schema_version": 1, "source": "isolated-p6-cave-simulator",
                    "mode": "SIMULATION", "domain_id": int(DOMAIN),
                    "command_topic": COMMAND, "ready": fresh and self._map is not None,
                    "odom": dict(self._odom) if fresh else None,
                    "trail": [list(x) for x in self._trail] if fresh else [],
                    "has_map": self._map is not None}

    def map(self):
        with self._lock:
            return dict(self._map) if self._map is not None else None

    def safe_stop(self):
        with self._lock:
            self._token = None
            self._publish(0, 0)


class Handler(BaseHTTPRequestHandler):
    bridge = None

    def log_message(self, *_args):
        pass

    def _response(self, code, obj):
        data = json.dumps(obj, separators=(",", ":")).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/v1/simulation/state":
            self._response(200, self.bridge.state())
        elif self.path == "/v1/simulation/map":
            data = self.bridge.map()
            self._response(200 if data else 503, data or {"error": "map unavailable"})
        else:
            self._response(404, {"error": "unknown route"})

    def do_POST(self):
        if self.path not in ("/v1/simulation/claim", "/v1/simulation/command", "/v1/simulation/release"):
            self._response(404, {"error": "unknown route"})
            return
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if size < 2 or size > 512 or self.headers.get("Content-Type") != "application/json":
                raise ValueError("invalid request")
            body = json.loads(self.rfile.read(size))
            if self.path == "/v1/simulation/claim":
                if body != {}:
                    raise ValueError("claim expects empty object")
                self._response(200, {"token": self.bridge.claim()})
            elif self.path == "/v1/simulation/command":
                applied = self.bridge.command(body)
                self._response(200 if applied else 409, {"applied": applied})
            else:
                if not isinstance(body, dict) or set(body) != {"token"}:
                    raise ValueError("release requires token")
                released = self.bridge.release(body["token"])
                self._response(200 if released else 409, {"released": released})
        except (ValueError, KeyError, TypeError):
            self._response(400, {"error": "invalid simulation command"})

    def do_PUT(self):
        self._response(405, {"error": "method not allowed"})

    do_DELETE = do_PUT
    do_PATCH = do_PUT


def main():
    require_isolation()
    rclpy.init(args=["--ros-args",
                     "-p", f"command_topic:={COMMAND}", "-p", f"odom_topic:={ODOM}",
                     "-p", f"map_topic:={MAP}", "-p", f"scan_topic:={SCAN}",
                     "-p", "world_frame:=console_sim_map",
                     "-p", "odom_frame:=console_sim_odom",
                     "-p", "base_frame:=console_sim_base_link"])
    simulator = CaveSimulator()
    if (simulator.command_topic, simulator.odom_topic, simulator.map_topic) != (COMMAND, ODOM, MAP):
        raise SystemExit("simulator ROS topic isolation failed")
    bridge = SimBridge()
    Handler.bridge = bridge
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(simulator)
    executor.add_node(bridge)
    worker = threading.Thread(target=executor.spin, daemon=True)
    worker.start()
    server = ThreadingHTTPServer((HOST, PORT), Handler)
    server.daemon_threads = True
    print(f"SIMULATION ONLY: {HOST}:{PORT} domain={DOMAIN} command={COMMAND}", flush=True)
    try:
        server.serve_forever(poll_interval=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.safe_stop()
        server.server_close()
        executor.shutdown()
        simulator.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()
        worker.join(timeout=2)


if __name__ == "__main__":
    main()
