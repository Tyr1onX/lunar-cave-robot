#!/usr/bin/env python3
"""Measure Agent overhead with an isolated synthetic ROS 2 PointCloud2 stream.

Run only inside the dedicated CI container configured by console-tests.yml. The
publisher and Agent share that container's loopback-only ROS domain; no robot
workspace or physical device is involved.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import statistics
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


DOMAIN_ID = "47"
TOPIC = "/livox/lidar"
POINT_COUNT = 60_000
POINT_STEP = 16
PUBLISH_HZ = 10.0
PAYLOAD_BYTES = POINT_COUNT * POINT_STEP
AGENT_URL = "http://127.0.0.1:8765"


def _publisher_main() -> int:
    if os.environ.get("ROS_DOMAIN_ID") != DOMAIN_ID or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Synthetic ROS publisher requires ROS_DOMAIN_ID=47 and ROS_LOCALHOST_ONLY=1")
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node("lunar_ci_synthetic_pointcloud")
    publisher = node.create_publisher(PointCloud2, TOPIC, qos_profile_sensor_data)
    payload = bytes(PAYLOAD_BYTES)
    message = PointCloud2()
    message.height = 1
    message.width = POINT_COUNT
    message.point_step = POINT_STEP
    message.row_step = PAYLOAD_BYTES
    message.is_dense = True
    message.data = payload

    def publish() -> None:
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "synthetic_ci_lidar"
        publisher.publish(message)

    node.create_timer(1.0 / PUBLISH_HZ, publish)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_publisher(publisher)
        node.destroy_node()
        rclpy.shutdown()
    return 0


def _http_json(path: str) -> dict[str, Any] | None:
    try:
        with urllib.request.urlopen(f"{AGENT_URL}{path}", timeout=2.0) as response:
            return json.loads(response.read())
    except (OSError, urllib.error.URLError, json.JSONDecodeError):
        return None


def _proc_cpu_seconds(pid: int) -> float | None:
    try:
        raw = Path(f"/proc/{pid}/stat").read_text(encoding="ascii")
        fields = raw[raw.rfind(")") + 2 :].split()
        ticks = int(fields[11]) + int(fields[12])
        return ticks / os.sysconf("SC_CLK_TCK")
    except (OSError, ValueError, IndexError):
        return None


def _proc_rss_bytes(pid: int) -> int | None:
    try:
        for line in Path(f"/proc/{pid}/status").read_text(encoding="ascii").splitlines():
            if line.startswith("VmRSS:"):
                return int(line.split()[1]) * 1024
    except (OSError, ValueError, IndexError):
        return None
    return None


def _percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, math.ceil(fraction * len(ordered)) - 1))
    return round(ordered[index], 2)


def _sample_phase(pid: int, duration: float, *, poll_seconds: float = 0.25) -> dict[str, Any]:
    started = time.monotonic()
    previous_time = started
    previous_cpu = _proc_cpu_seconds(pid)
    cpu_intervals: list[float] = []
    rss_samples: list[int] = []
    graph_ages: list[float] = []
    receive_rates: list[float] = []
    statuses: list[str] = []
    graph_times: set[str] = set()
    receive_counts: list[int] = []
    domains: set[int] = set()
    observations = 0

    while time.monotonic() - started < duration:
        now = time.monotonic()
        current_cpu = _proc_cpu_seconds(pid)
        if previous_cpu is not None and current_cpu is not None and now > previous_time:
            cpu_intervals.append(max(0.0, current_cpu - previous_cpu) / (now - previous_time) * 100.0)
        if current_cpu is not None:
            previous_cpu = current_cpu
            previous_time = now
        rss = _proc_rss_bytes(pid)
        if rss is not None:
            rss_samples.append(rss)
        telemetry = _http_json("/v1/telemetry")
        if telemetry is not None:
            observations += 1
            domain = telemetry.get("domain_id")
            if isinstance(domain, int):
                domains.add(domain)
            graph_time = telemetry.get("graph_observed_at")
            graph_age = telemetry.get("graph_age_ms")
            if isinstance(graph_time, str):
                graph_times.add(graph_time)
            if isinstance(graph_age, (int, float)):
                graph_ages.append(float(graph_age))
            stream = next((item for item in telemetry.get("streams", []) if item.get("topic") == TOPIC), None)
            if stream is not None:
                status = stream.get("status")
                if isinstance(status, str):
                    statuses.append(status)
                rate = stream.get("receive_rate_hz")
                if isinstance(rate, (int, float)):
                    receive_rates.append(float(rate))
                count = stream.get("receive_count")
                if isinstance(count, int):
                    receive_counts.append(count)
        time.sleep(poll_seconds)

    total = max(0.001, time.monotonic() - started)
    cpu_average = sum(cpu_intervals) / len(cpu_intervals) if cpu_intervals else 0.0
    return {
        "duration_seconds": round(total, 2),
        "agent_cpu_percent_of_one_core_average": round(cpu_average, 2),
        "agent_cpu_percent_of_one_core_p95_interval": _percentile(cpu_intervals, 0.95),
        "agent_rss_bytes_median": int(statistics.median(rss_samples)) if rss_samples else None,
        "agent_rss_bytes_peak": max(rss_samples) if rss_samples else None,
        "graph_age_ms_p95": _percentile(graph_ages, 0.95),
        "graph_age_ms_peak": round(max(graph_ages), 2) if graph_ages else None,
        "distinct_graph_observations": len(graph_times),
        "pointcloud_receive_rate_hz_median": _percentile(receive_rates, 0.5),
        "pointcloud_receive_rate_hz_p05": _percentile(receive_rates, 0.05),
        "pointcloud_receive_count_peak": max(receive_counts) if receive_counts else 0,
        "stream_statuses": sorted(set(statuses)),
        "http_observations": observations,
        "observed_ros_domains": sorted(domains),
    }


def _wait_for_health(process: subprocess.Popen[Any], timeout: float = 20.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"Agent exited before readiness with status {process.returncode}")
        if _http_json("/health") is not None:
            return
        time.sleep(0.2)
    raise TimeoutError("Agent did not become healthy within the readiness window")


def run_benchmark(report_path: Path, baseline_seconds: float, load_seconds: float) -> dict[str, Any]:
    if os.environ.get("ROS_DOMAIN_ID") != DOMAIN_ID or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise RuntimeError("Run this benchmark only in the isolated ROS 2 container (Domain 47, localhost-only).")

    repo_root = Path(__file__).resolve().parents[3]
    agent_env = os.environ.copy()
    agent_env["PYTHONPATH"] = str(repo_root / "tools" / "console-agent")
    agent = subprocess.Popen(
        [sys.executable, "-m", "console_agent"],
        env=agent_env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    publisher: subprocess.Popen[Any] | None = None
    try:
        _wait_for_health(agent)
        time.sleep(2.0)
        baseline = _sample_phase(agent.pid, baseline_seconds)
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps({
            "benchmark": "isolated_ros2_pointcloud2_agent_load",
            "phase": "baseline_complete",
            "ros_domain_id": int(DOMAIN_ID),
            "baseline": baseline,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")

        publisher = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "--publisher"],
            env=os.environ.copy(),
            stdout=subprocess.DEVNULL,
        )
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if publisher.poll() is not None:
                raise RuntimeError(f"Synthetic publisher exited with status {publisher.returncode}")
            telemetry = _http_json("/v1/telemetry")
            if telemetry is not None:
                stream = next((item for item in telemetry.get("streams", []) if item.get("topic") == TOPIC), None)
                if stream and stream.get("status") == "streaming":
                    break
            time.sleep(0.25)
        else:
            raise TimeoutError("Synthetic PointCloud2 data did not reach the Agent")

        load = _sample_phase(agent.pid, load_seconds)
        publisher.send_signal(signal.SIGINT)
        try:
            publisher.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            publisher.terminate()
            publisher.wait(timeout=5.0)
        recovery_deadline = time.monotonic() + 8.0
        recovered_to_unavailable = False
        while time.monotonic() < recovery_deadline:
            telemetry = _http_json("/v1/telemetry")
            if telemetry is not None:
                stream = next((item for item in telemetry.get("streams", []) if item.get("topic") == TOPIC), None)
                if stream and stream.get("status") == "unavailable" and stream.get("received_at") is None:
                    recovered_to_unavailable = True
                    break
            time.sleep(0.25)

        rss_baseline = baseline.get("agent_rss_bytes_median")
        rss_load = load.get("agent_rss_bytes_peak")
        rss_increase = None if rss_baseline is None or rss_load is None else max(0, rss_load - rss_baseline)
        graph_p95 = load.get("graph_age_ms_p95")
        receive_p05 = load.get("pointcloud_receive_rate_hz_p05")
        report = {
            "benchmark": "isolated_ros2_pointcloud2_agent_load",
            "ros_distribution": os.environ.get("ROS_DISTRO", "unknown"),
            "ros_domain_id": int(DOMAIN_ID),
            "ros_localhost_only": True,
            "topic": TOPIC,
            "message_type": "sensor_msgs/msg/PointCloud2",
            "synthetic_payload_bytes_per_message": PAYLOAD_BYTES,
            "synthetic_points_per_message": POINT_COUNT,
            "synthetic_point_step_bytes": POINT_STEP,
            "synthetic_publish_rate_hz": PUBLISH_HZ,
            "estimated_payload_mib_per_second": round(PAYLOAD_BYTES * PUBLISH_HZ / 1024 / 1024, 2),
            "baseline": baseline,
            "load": load,
            "impact": {
                "agent_cpu_delta_percent_of_one_core": round(
                    load["agent_cpu_percent_of_one_core_average"] - baseline["agent_cpu_percent_of_one_core_average"], 2
                ),
                "agent_rss_peak_increase_bytes_vs_baseline_median": rss_increase,
                "graph_age_p95_ms_during_load": graph_p95,
                "graph_observation_count_during_load": load["distinct_graph_observations"],
                "synthetic_stream_rate_p05_hz": receive_p05,
                "stream_returns_to_unavailable_after_publisher_stop": recovered_to_unavailable,
            },
            "acceptance": {
                "agent_cpu_average_below_one_core": load["agent_cpu_percent_of_one_core_average"] < 100.0,
                "agent_rss_increase_below_128_mib": rss_increase is not None and rss_increase < 128 * 1024 * 1024,
                "graph_age_p95_below_5000_ms": graph_p95 is not None and graph_p95 < 5_000,
                "graph_observed_at_least_5_times": load["distinct_graph_observations"] >= 5,
                "pointcloud_receive_rate_p05_at_least_5_hz": receive_p05 is not None and receive_p05 >= 5.0,
                "stream_unavailable_after_publisher_stop": recovered_to_unavailable,
            },
        }
        report["passed"] = all(report["acceptance"].values())
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2, sort_keys=True))
        if not report["passed"]:
            raise RuntimeError("ROS 2 synthetic load benchmark did not meet its observation/resource guardrails")
        return report
    finally:
        if publisher is not None and publisher.poll() is None:
            publisher.terminate()
            try:
                publisher.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                publisher.kill()
        if agent.poll() is None:
            agent.terminate()
            try:
                agent.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                agent.kill()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--publisher", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--report", type=Path, default=Path("/tmp/lunar-link-telemetry-benchmark.json"))
    parser.add_argument("--baseline-seconds", type=float, default=6.0)
    parser.add_argument("--load-seconds", type=float, default=12.0)
    args = parser.parse_args()
    if args.publisher:
        return _publisher_main()
    run_benchmark(args.report, args.baseline_seconds, args.load_seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
