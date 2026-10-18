#!/usr/bin/env python3
"""Exercise the read-only telemetry observer with isolated ROS 2 pipeline data.

This runner is intended for the dedicated localhost-only CI container. It creates
only synthetic publishers and never starts any project or hardware launch file.
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Callable


DOMAIN_ID = "47"
AGENT_URL = "http://127.0.0.1:8765"
PIPELINE_TOPICS = {
    "/livox/lidar": "sensor_msgs/msg/PointCloud2",
    "/livox/imu": "sensor_msgs/msg/Imu",
    "/imu/data": "sensor_msgs/msg/Imu",
    "/odometry": "nav_msgs/msg/Odometry",
    "/cloud_registered": "sensor_msgs/msg/PointCloud2",
    "/terrain_map": "sensor_msgs/msg/PointCloud2",
    "/scan": "sensor_msgs/msg/LaserScan",
    "/map": "nav_msgs/msg/OccupancyGrid",
}


def _publisher_main(control_path: Path) -> int:
    import rclpy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu, LaserScan, PointCloud2

    if os.environ.get("ROS_LOCALHOST_ONLY") != "1" or os.environ.get("ROS_DOMAIN_ID") not in {"47", "48"}:
        raise RuntimeError("Publisher requires localhost-only ROS Domain 47 or 48.")

    message_types: dict[str, Any] = {
        "/livox/lidar": PointCloud2,
        "/livox/imu": Imu,
        "/imu/data": Imu,
        "/odometry": Odometry,
        "/cloud_registered": PointCloud2,
        "/terrain_map": PointCloud2,
        "/scan": LaserScan,
        "/map": OccupancyGrid,
    }
    rclpy.init()
    node_name = "lunar_ci_domain48_source" if os.environ["ROS_DOMAIN_ID"] == "48" else "lunar_ci_pipeline_source"
    node = rclpy.create_node(node_name)
    publishers = {
        topic: node.create_publisher(message_type, topic, qos_profile_sensor_data)
        for topic, message_type in message_types.items()
    }

    def stamp(message: Any) -> None:
        if hasattr(message, "header"):
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = "synthetic_pipeline_frame"

    def publish() -> None:
        try:
            suppressed = set(control_path.read_text(encoding="utf-8").split()) if control_path.exists() else set()
        except OSError:
            suppressed = set(PIPELINE_TOPICS)
        for topic, publisher in publishers.items():
            if topic in suppressed:
                continue
            message = message_types[topic]()
            stamp(message)
            if isinstance(message, PointCloud2):
                message.height = 1
                message.width = 1
                message.point_step = 16
                message.row_step = 16
                message.is_dense = True
                message.data = bytes(16)
            elif isinstance(message, LaserScan):
                message.angle_increment = 0.1
                message.range_min = 0.1
                message.range_max = 10.0
                message.ranges = [1.0]
            elif isinstance(message, OccupancyGrid):
                message.info.width = 1
                message.info.height = 1
                message.data = [0]
            publisher.publish(message)

    node.create_timer(0.05, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
    return 0


def _http_json(path: str) -> dict[str, Any] | None:
    try:
        with urllib.request.urlopen(f"{AGENT_URL}{path}", timeout=1.5) as response:
            return json.loads(response.read())
    except (OSError, urllib.error.URLError, json.JSONDecodeError):
        return None


def _stream(telemetry: dict[str, Any] | None, topic: str) -> dict[str, Any] | None:
    if telemetry is None:
        return None
    return next((entry for entry in telemetry.get("streams", []) if entry.get("topic") == topic), None)


def _wait_for(predicate: Callable[[], bool], description: str, timeout: float = 25.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.25)
    raise TimeoutError(f"Timed out waiting for {description}")


def _start_publisher(control_path: Path, domain: str = DOMAIN_ID) -> subprocess.Popen[Any]:
    env = os.environ.copy()
    env["ROS_DOMAIN_ID"] = domain
    env["ROS_LOCALHOST_ONLY"] = "1"
    return subprocess.Popen(
        [sys.executable, str(Path(__file__).resolve()), "--publisher", "--control", str(control_path)],
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )


def _stop(process: subprocess.Popen[Any] | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        process.terminate()
        process.wait(timeout=5.0)


def run(report_path: Path) -> dict[str, Any]:
    if os.environ.get("ROS_DOMAIN_ID") != DOMAIN_ID or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Run only in the isolated ROS 2 container (Domain 47, ROS_LOCALHOST_ONLY=1).")

    report_path.parent.mkdir(parents=True, exist_ok=True)
    repo = Path(__file__).resolve().parents[3]
    env = os.environ.copy()
    env["LUNAR_CONSOLE_EXPECTED_ROS_DOMAIN_ID"] = DOMAIN_ID
    agent_python = str(repo / "tools" / "console-agent")
    env["PYTHONPATH"] = os.pathsep.join(value for value in (agent_python, env.get("PYTHONPATH", "")) if value)
    agent = subprocess.Popen([sys.executable, "-m", "console_agent"], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    control_path = report_path.parent / "synthetic-publisher-control.txt"
    publisher: subprocess.Popen[Any] | None = None
    alternate: subprocess.Popen[Any] | None = None
    observations: dict[str, Any] = {}
    fatal_error: str | None = None

    try:
        _wait_for(lambda: _http_json("/health") is not None, "Agent /health")
        publisher = _start_publisher(control_path)
        _wait_for(
            lambda: (data := _http_json("/v1/telemetry")) is not None and
            all(
                (_stream(data, topic) or {}).get("status") == "streaming"
                and (_stream(data, topic) or {}).get("received_at")
                and (_stream(data, topic) or {}).get("source_stamp")
                and (_stream(data, topic) or {}).get("receive_rate_hz") is not None
                for topic in PIPELINE_TOPICS
            ),
            "all synthetic streams with observed timestamps and receive rates",
        )
        baseline = _http_json("/v1/telemetry")
        observations["normal_streaming"] = {
            "passed": baseline is not None
            and baseline.get("domain_id") == int(DOMAIN_ID)
            and baseline.get("graph_status") == "ok"
            and baseline.get("graph_age_ms") is not None
            and all(
                (_stream(baseline, topic) or {}).get("status") == "streaming"
                and (_stream(baseline, topic) or {}).get("received_at")
                and (_stream(baseline, topic) or {}).get("source_stamp")
                and (_stream(baseline, topic) or {}).get("receive_rate_hz")
                and (_stream(baseline, topic) or {}).get("receive_rate_hz") > 0
                for topic in PIPELINE_TOPICS
            ),
            "domain_id": None if baseline is None else baseline.get("domain_id"),
            "graph_age_ms": None if baseline is None else baseline.get("graph_age_ms"),
            "graph_observed_at": None if baseline is None else baseline.get("graph_observed_at"),
            "source_times_and_rates_present": baseline is not None and all(
                (_stream(baseline, topic) or {}).get("received_at")
                and (_stream(baseline, topic) or {}).get("source_stamp")
                and (_stream(baseline, topic) or {}).get("receive_rate_hz") is not None
                for topic in PIPELINE_TOPICS
            ),
            "topics": {topic: _stream(baseline, topic) for topic in PIPELINE_TOPICS},
        }

        control_path.write_text("/scan\n", encoding="utf-8")
        _wait_for(
            lambda: (_stream(_http_json("/v1/telemetry"), "/scan") or {}).get("status") == "stale",
            "LaserScan receive timeout while synthetic publisher remains present",
            timeout=12.0,
        )
        dropped = _http_json("/v1/telemetry")
        observations["scan_stream_timeout"] = {
            "passed": (_stream(dropped, "/scan") or {}).get("status") == "stale"
            and (_stream(dropped, "/scan") or {}).get("publisher_count") == 1,
            "stream": _stream(dropped, "/scan"),
            "upstream_still_streaming": all((_stream(dropped, topic) or {}).get("status") == "streaming" for topic in ("/livox/lidar", "/livox/imu", "/odometry", "/terrain_map")),
        }

        control_path.write_text("", encoding="utf-8")
        _wait_for(lambda: (_stream(_http_json("/v1/telemetry"), "/scan") or {}).get("status") == "streaming", "LaserScan recovery")
        observations["scan_recovery"] = {"passed": (_stream(_http_json("/v1/telemetry"), "/scan") or {}).get("status") == "streaming"}

        _stop(publisher)
        publisher = None
        _wait_for(
            lambda: (
                (snapshot := _http_json("/v1/snapshot")) is not None and
                not any(node.get("name") == "lunar_ci_pipeline_source" for node in snapshot.get("ros", {}).get("nodes", [])) and
                (_stream(_http_json("/v1/telemetry"), "/livox/lidar") or {}).get("status") == "unavailable"
            ),
            "synthetic publisher node exit and graph removal",
            timeout=20.0,
        )
        node_exit_snapshot = _http_json("/v1/snapshot")
        node_exit_telemetry = _http_json("/v1/telemetry")
        observations["publisher_node_exit"] = {
            "passed": not any(node.get("name") == "lunar_ci_pipeline_source" for node in (node_exit_snapshot or {}).get("ros", {}).get("nodes", [])),
            "domain_id": None if node_exit_snapshot is None else node_exit_snapshot.get("ros", {}).get("domain_id"),
            "livox_lidar": _stream(node_exit_telemetry, "/livox/lidar"),
        }

        publisher = _start_publisher(control_path)
        _wait_for(
            lambda: (data := _http_json("/v1/telemetry")) is not None and
            all((_stream(data, topic) or {}).get("status") == "streaming" for topic in PIPELINE_TOPICS),
            "full pipeline data recovery after node restart",
        )
        observations["pipeline_recovery"] = {"passed": True}

        _stop(publisher)
        publisher = None
        _wait_for(lambda: (_stream(_http_json("/v1/telemetry"), "/livox/lidar") or {}).get("publisher_count") == 0,
                  "Domain 47 publisher removal before isolated mismatch", timeout=20.0)
        alternate = _start_publisher(control_path, "48")
        time.sleep(3.0)
        mismatch = _http_json("/v1/telemetry")
        observations["separate_domain_invisible"] = {
            "passed": mismatch is not None and mismatch.get("domain_id") == 47 and (_stream(mismatch, "/livox/lidar") or {}).get("publisher_count") == 0,
            "agent_domain_id": None if mismatch is None else mismatch.get("domain_id"),
            "domain_48_publisher_count_in_domain_47": None if mismatch is None else (_stream(mismatch, "/livox/lidar") or {}).get("publisher_count"),
        }

    except Exception as error:
        fatal_error = f"{type(error).__name__}: {error}"
    finally:
        _stop(alternate)
        _stop(publisher)
        _stop(agent)

    failed = [name for name, record in observations.items() if not record.get("passed")]
    report = {
        "acceptance": "isolated_ros2_pipeline_synthetic_scenarios",
        "ros_distribution": os.environ.get("ROS_DISTRO", "unknown"),
        "agent_domain_id": int(DOMAIN_ID),
        "ros_localhost_only": True,
        "hardware_or_robot_launch_started": False,
        "scenario_results": observations,
        "passed": not failed and fatal_error is None,
        "failed_scenarios": failed,
        "fatal_error": fatal_error,
    }
    report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if fatal_error:
        raise RuntimeError("Synthetic pipeline scenarios failed before completion: " + fatal_error)
    if failed:
        raise RuntimeError("Synthetic pipeline scenarios failed: " + ", ".join(failed))
    return report


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--publisher", action="store_true")
    parser.add_argument("--control", type=Path)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    if args.publisher:
        if args.control is None:
            parser.error("--control is required with --publisher")
        return _publisher_main(args.control)
    if args.report is None:
        parser.error("--report is required")
    report = run(args.report)
    print(json.dumps({"passed": report["passed"], "scenarios": list(report["scenario_results"])}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
