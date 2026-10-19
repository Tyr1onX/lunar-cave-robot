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
import tempfile
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
MIN_ACCEPTED_RATE_HZ = 5.0


def _write_json_atomic(path: str | None, value: dict[str, Any]) -> None:
    if not path:
        return
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_suffix(target.suffix + ".tmp")
    temporary.write_text(json.dumps(value, sort_keys=True), encoding="utf-8")
    temporary.replace(target)


def _event_rate_hz(times: list[float], start: float, end: float) -> float:
    selected = [stamp for stamp in times if start <= stamp <= end]
    if len(selected) < 2 or selected[-1] <= selected[0]:
        return 0.0
    return round((len(selected) - 1) / (selected[-1] - selected[0]), 3)


def _count_events(times: list[float], start: float, end: float) -> int:
    return sum(start <= stamp <= end for stamp in times)


def _summarize_deliveries(
    publisher_times: list[float],
    probe_times: list[float],
    *,
    agent_received_count: int,
    start: float,
    end: float,
    probe_sequences: list[int] | None = None,
) -> dict[str, Any]:
    publisher_count = _count_events(publisher_times, start, end)
    probe_count = _count_events(probe_times, start, end)
    publisher_probe_delta = publisher_count - probe_count
    probe_agent_delta = probe_count - agent_received_count
    in_window_sequences = {
        sequence
        for sequence, stamp in enumerate(publisher_times, start=1)
        if start <= stamp <= end
    }
    observed_sequences = set(probe_sequences or [])
    delivered_sequence_count = len(in_window_sequences & observed_sequences)
    return {
        "publisher_count": publisher_count,
        "publisher_rate_hz": _event_rate_hz(publisher_times, start, end),
        "probe_count": probe_count,
        "probe_rate_hz": _event_rate_hz(probe_times, start, end),
        "agent_count": agent_received_count,
        "publisher_probe_event_count_delta": publisher_probe_delta,
        "publisher_probe_sequence_delivered_count": delivered_sequence_count,
        "publisher_probe_sequence_missing_count": max(0, len(in_window_sequences) - delivered_sequence_count),
        "probe_agent_event_count_delta_estimate": probe_agent_delta,
    }


def _diagnose_pipeline(
    publisher_rate_hz: float,
    dds_probe_rate_hz: float,
    agent_rate_p05_hz: float | None,
    *,
    ci_pressure_observed: bool,
    publisher_cpu_percent: float | None = None,
    probe_cpu_percent: float | None = None,
    agent_cpu_percent: float | None = None,
) -> dict[str, Any]:
    """Classify the first measured rate boundary without attributing unsupported root cause."""
    publisher_ok = publisher_rate_hz >= MIN_ACCEPTED_RATE_HZ
    probe_ok = dds_probe_rate_hz >= MIN_ACCEPTED_RATE_HZ
    agent_ok = agent_rate_p05_hz is not None and agent_rate_p05_hz >= MIN_ACCEPTED_RATE_HZ
    if not publisher_ok:
        stage = "message_generation_below_5_hz"
    elif not probe_ok:
        stage = "dds_delivery_or_ci_pressure_below_5_hz"
    elif not agent_ok:
        stage = "agent_receive_path_below_5_hz"
    else:
        stage = "all_observed_stages_at_or_above_5_hz"
    if publisher_ok and probe_ok and agent_ok:
        suspected_cause = "none_observed"
    elif not publisher_ok:
        suspected_cause = (
            "ci_resource_pressure_or_publisher_scheduling"
            if ci_pressure_observed or (publisher_cpu_percent is not None and publisher_cpu_percent >= 80.0)
            else "publisher_generation_or_timer"
        )
    elif not probe_ok:
        suspected_cause = (
            "independent_probe_or_ci_resource_pressure"
            if ci_pressure_observed or (probe_cpu_percent is not None and probe_cpu_percent >= 80.0)
            else "dds_delivery_path"
        )
    else:
        suspected_cause = (
            "agent_processing_or_ci_resource_pressure"
            if ci_pressure_observed or (agent_cpu_percent is not None and agent_cpu_percent >= 80.0)
            else "agent_receive_path_or_subscription"
        )
    return {
        "minimum_rate_hz": MIN_ACCEPTED_RATE_HZ,
        "publisher_stage_at_least_5_hz": publisher_ok,
        "independent_dds_probe_at_least_5_hz": probe_ok,
        "agent_receive_p05_at_least_5_hz": agent_ok,
        "first_stage_below_threshold": stage,
        "suspected_cause_category": suspected_cause,
        "ci_resource_pressure_observed": ci_pressure_observed,
        "interpretation": (
            "CI CPU/resource pressure evidence was observed during the measurement; low rates cannot be attributed to the Agent alone."
            if ci_pressure_observed and stage != "all_observed_stages_at_or_above_5_hz"
            else "The first below-threshold measurement localizes the stage; it does not by itself prove a root cause."
        ),
    }


class _ReceiveCounter:
    """Accumulate Agent callback counts across subscription generations."""

    def __init__(self) -> None:
        self._generation: int | None = None
        self._count: int | None = None
        self.total = 0

    def observe(self, generation: int, count: int) -> None:
        if self._generation is None:
            self._generation, self._count = generation, count
            return
        if generation != self._generation:
            self.total += max(0, count)
        elif self._count is not None and count >= self._count:
            self.total += count - self._count
        else:
            self.total += max(0, count)
        self._generation, self._count = generation, count


def _publisher_main(stats_path: str | None = None) -> int:
    if os.environ.get("ROS_DOMAIN_ID") != DOMAIN_ID or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Synthetic ROS publisher requires ROS_DOMAIN_ID=47 and ROS_LOCALHOST_ONLY=1")
    import rclpy
    from rclpy.executors import ExternalShutdownException
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

    published_times: list[float] = []
    timer_times: list[float] = []
    next_stats_write = time.monotonic()
    matched_subscription_count = 0

    def stats() -> dict[str, Any]:
        intervals = [right - left for left, right in zip(published_times, published_times[1:])]
        timer_intervals = [right - left for left, right in zip(timer_times, timer_times[1:])]
        duration = max(0.0, published_times[-1] - published_times[0]) if len(published_times) > 1 else 0.0
        return {
            "requested_publish_rate_hz": PUBLISH_HZ,
            "timer_callback_count": len(timer_times),
            "published_message_count": len(published_times),
            "publish_monotonic_seconds": list(published_times),
            "timer_callback_monotonic_seconds": list(timer_times),
            "measured_publish_rate_hz": round((len(published_times) - 1) / duration, 3) if duration > 0 else 0.0,
            "publish_interval_ms_p95": _percentile([value * 1000 for value in intervals], 0.95),
            "timer_callback_interval_ms_p95": _percentile([value * 1000 for value in timer_intervals], 0.95),
            "matched_subscription_count": matched_subscription_count,
            "last_sequence": len(published_times),
        }

    def publish() -> None:
        nonlocal matched_subscription_count, next_stats_write
        timer_now = time.monotonic()
        timer_times.append(timer_now)
        matched_subscription_count = publisher.get_subscription_count()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = f"synthetic_ci_lidar:{len(published_times) + 1}"
        publisher.publish(message)
        published_times.append(time.monotonic())
        if time.monotonic() >= next_stats_write:
            _write_json_atomic(stats_path, stats())
            next_stats_write = time.monotonic() + 1.0

    timer = node.create_timer(1.0 / PUBLISH_HZ, publish)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        _write_json_atomic(stats_path, stats())
        if rclpy.ok():
            node.destroy_timer(timer)
            node.destroy_publisher(publisher)
            node.destroy_node()
            rclpy.shutdown()
    return 0


def _dds_probe_main(stats_path: str | None = None) -> int:
    """Observe the same ROS topic independently of the Agent callback path."""
    if os.environ.get("ROS_DOMAIN_ID") != DOMAIN_ID or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Synthetic DDS probe requires ROS_DOMAIN_ID=47 and ROS_LOCALHOST_ONLY=1")
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import PointCloud2

    rclpy.init()
    node = rclpy.create_node("lunar_ci_pointcloud_dds_probe")
    receive_times: list[float] = []
    sequence_numbers: list[int] = []
    next_stats_write = time.monotonic()

    def stats() -> dict[str, Any]:
        duration = max(0.0, receive_times[-1] - receive_times[0]) if len(receive_times) > 1 else 0.0
        return {
            "ready": True,
            "receive_count": len(receive_times),
            "receive_monotonic_seconds": list(receive_times),
            "sequence_numbers": list(sequence_numbers),
            "measured_receive_rate_hz": round((len(receive_times) - 1) / duration, 3) if duration > 0 else 0.0,
            "last_sequence": sequence_numbers[-1] if sequence_numbers else None,
        }

    def receive(message: Any) -> None:
        nonlocal next_stats_write
        receive_times.append(time.monotonic())
        frame_id = str(getattr(message.header, "frame_id", ""))
        try:
            sequence_numbers.append(int(frame_id.rsplit(":", 1)[1]))
        except (IndexError, ValueError):
            sequence_numbers.append(-1)
        if receive_times[-1] >= next_stats_write:
            _write_json_atomic(stats_path, stats())
            next_stats_write = receive_times[-1] + 1.0

    subscription = node.create_subscription(PointCloud2, TOPIC, receive, qos_profile_sensor_data)
    _write_json_atomic(stats_path, stats())
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        _write_json_atomic(stats_path, stats())
        if rclpy.ok():
            node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
    return 0


def _read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
        return value if isinstance(value, dict) else {}
    except (OSError, json.JSONDecodeError):
        return {}


def _cpu_cgroup_snapshot() -> dict[str, Any]:
    """Read container CPU quota/throttling counters when cgroup telemetry is available."""
    result: dict[str, Any] = {
        "cpu_count": os.cpu_count(),
        "load_average": None,
        "quota_cores": None,
        "throttled_usec": None,
        "throttled_periods": None,
        "total_periods": None,
    }
    try:
        result["load_average"] = [round(value, 2) for value in os.getloadavg()]
    except (AttributeError, OSError):
        pass
    try:
        quota, period = Path("/sys/fs/cgroup/cpu.max").read_text(encoding="ascii").split()[:2]
        if quota != "max":
            result["quota_cores"] = round(int(quota) / int(period), 3)
        cpu_stat = Path("/sys/fs/cgroup/cpu.stat").read_text(encoding="ascii").splitlines()
        values = dict(line.split() for line in cpu_stat if len(line.split()) == 2)
        result["throttled_usec"] = int(values["throttled_usec"]) if "throttled_usec" in values else None
        result["throttled_periods"] = int(values["nr_throttled"]) if "nr_throttled" in values else None
        result["total_periods"] = int(values["nr_periods"]) if "nr_periods" in values else None
    except (OSError, ValueError, IndexError, KeyError):
        try:
            quota = int(Path("/sys/fs/cgroup/cpu/cpu.cfs_quota_us").read_text(encoding="ascii"))
            period = int(Path("/sys/fs/cgroup/cpu/cpu.cfs_period_us").read_text(encoding="ascii"))
            if quota > 0 and period > 0:
                result["quota_cores"] = round(quota / period, 3)
            cpu_stat = Path("/sys/fs/cgroup/cpu/cpu.stat").read_text(encoding="ascii").splitlines()
            values = dict(line.split() for line in cpu_stat if len(line.split()) == 2)
            if "throttled_time" in values:
                result["throttled_usec"] = int(values["throttled_time"]) // 1000
            if "nr_throttled" in values:
                result["throttled_periods"] = int(values["nr_throttled"])
            if "nr_periods" in values:
                result["total_periods"] = int(values["nr_periods"])
        except (OSError, ValueError, IndexError, KeyError):
            pass
    return result


def _cpu_throttling_delta(before: dict[str, Any], after: dict[str, Any], duration: float) -> dict[str, Any]:
    start = before.get("throttled_usec")
    end = after.get("throttled_usec")
    delta = None if not isinstance(start, int) or not isinstance(end, int) else max(0, end - start)
    fraction = None if delta is None or duration <= 0 else round(delta / (duration * 1_000_000), 4)
    throttle_start = before.get("throttled_periods")
    throttle_end = after.get("throttled_periods")
    periods_start = before.get("total_periods")
    periods_end = after.get("total_periods")
    throttled_periods = (
        None if not isinstance(throttle_start, int) or not isinstance(throttle_end, int)
        else max(0, throttle_end - throttle_start)
    )
    total_periods = (
        None if not isinstance(periods_start, int) or not isinstance(periods_end, int)
        else max(0, periods_end - periods_start)
    )
    throttled_period_ratio = (
        None if throttled_periods is None or total_periods is None or total_periods == 0
        else round(throttled_periods / total_periods, 4)
    )
    quota = after.get("quota_cores") or after.get("cpu_count")
    load_average = after.get("load_average")
    load_ratio = (
        round(load_average[0] / quota, 3)
        if isinstance(load_average, list) and load_average and isinstance(quota, (int, float)) and quota > 0
        else None
    )
    pressure = (fraction is not None and fraction >= 0.05) or (
        throttled_period_ratio is not None and throttled_period_ratio >= 0.05
    ) or (load_ratio is not None and load_ratio >= 1.0)
    return {
        "throttled_usec_delta": delta,
        "throttled_time_fraction_estimate": fraction,
        "throttled_periods_delta": throttled_periods,
        "total_periods_delta": total_periods,
        "throttled_period_ratio": throttled_period_ratio,
        "load_average_to_available_cpu_ratio": load_ratio,
        "pressure_observed": pressure,
        "pressure_evidence": [
            name for condition, name in (
                (fraction is not None and fraction >= 0.05, "cgroup_throttled_time"),
                (throttled_period_ratio is not None and throttled_period_ratio >= 0.05, "cgroup_throttled_periods"),
                (load_ratio is not None and load_ratio >= 1.0, "load_average_at_or_above_cpu_capacity"),
            ) if condition
        ],
    }


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


def _sample_phase(
    pid: int,
    duration: float,
    *,
    poll_seconds: float = 0.25,
    companion_pids: dict[str, int] | None = None,
) -> dict[str, Any]:
    started = time.monotonic()
    previous_time = started
    previous_cpu = _proc_cpu_seconds(pid)
    receive_counter = _ReceiveCounter()
    cpu_intervals: list[float] = []
    rss_samples: list[int] = []
    graph_ages: list[float] = []
    receive_rates: list[float] = []
    statuses: list[str] = []
    graph_times: set[str] = set()
    receive_counts: list[int] = []
    domains: set[int] = set()
    subscription_generations: set[int] = set()
    observations = 0
    process_samples: dict[str, dict[str, list[float | int]]] = {
        "agent": {"cpu_intervals": [], "rss": []},
    }
    tracked = {"agent": pid, **(companion_pids or {})}
    tracked_previous = {name: (_proc_cpu_seconds(process_pid), started) for name, process_pid in tracked.items()}

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
            process_samples["agent"]["rss"].append(rss)
        for name, process_pid in tracked.items():
            if name not in process_samples:
                process_samples[name] = {"cpu_intervals": [], "rss": []}
            current_process_cpu = _proc_cpu_seconds(process_pid)
            previous_process_cpu, previous_process_time = tracked_previous[name]
            if current_process_cpu is not None and previous_process_cpu is not None and now > previous_process_time:
                process_samples[name]["cpu_intervals"].append(
                    max(0.0, current_process_cpu - previous_process_cpu) / (now - previous_process_time) * 100.0
                )
                tracked_previous[name] = (current_process_cpu, now)
            elif current_process_cpu is not None:
                tracked_previous[name] = (current_process_cpu, now)
            process_rss = _proc_rss_bytes(process_pid)
            if process_rss is not None and name != "agent":
                process_samples[name]["rss"].append(process_rss)
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
                    generation = stream.get("subscription_generation")
                    if isinstance(generation, int):
                        subscription_generations.add(generation)
                        receive_counter.observe(generation, count)
        time.sleep(poll_seconds)

    total = max(0.001, time.monotonic() - started)
    cpu_average = sum(cpu_intervals) / len(cpu_intervals) if cpu_intervals else 0.0
    process_report: dict[str, dict[str, Any]] = {}
    for name, samples in process_samples.items():
        values = [float(value) for value in samples["cpu_intervals"]]
        rss_values = [int(value) for value in samples["rss"]]
        process_report[name] = {
            "cpu_percent_of_one_core_average": round(sum(values) / len(values), 2) if values else None,
            "cpu_percent_of_one_core_p95_interval": _percentile(values, 0.95),
            "rss_bytes_peak": max(rss_values) if rss_values else None,
        }
    return {
        "duration_seconds": round(total, 2),
        "started_monotonic_seconds": started,
        "finished_monotonic_seconds": time.monotonic(),
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
        "pointcloud_agent_receive_count_delta": receive_counter.total,
        "pointcloud_subscription_generations_observed": sorted(subscription_generations),
        "stream_statuses": sorted(set(statuses)),
        "tracked_processes": process_report,
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
    agent_python_path = str(repo_root / "tools" / "console-agent")
    ros_python_path = agent_env.get("PYTHONPATH", "")
    agent_env["PYTHONPATH"] = os.pathsep.join(path for path in (agent_python_path, ros_python_path) if path)
    agent = subprocess.Popen(
        [sys.executable, "-m", "console_agent"],
        env=agent_env,
    )
    publisher: subprocess.Popen[Any] | None = None
    probe: subprocess.Popen[Any] | None = None
    diagnostics_dir = Path(tempfile.mkdtemp(prefix="lunar-pointcloud-benchmark-"))
    publisher_stats_path = diagnostics_dir / "publisher.json"
    probe_stats_path = diagnostics_dir / "dds-probe.json"
    try:
        _wait_for_health(agent)
        time.sleep(2.0)
        baseline = _sample_phase(agent.pid, baseline_seconds)
        baseline_telemetry = _http_json("/v1/telemetry")
        report_path.parent.mkdir(parents=True, exist_ok=True)
        report_path.write_text(json.dumps({
            "benchmark": "isolated_ros2_pointcloud2_agent_load",
            "phase": "baseline_complete",
            "ros_domain_id": int(DOMAIN_ID),
            "baseline": baseline,
            "baseline_telemetry": baseline_telemetry,
        }, indent=2, sort_keys=True) + "\n", encoding="utf-8")

        probe = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "--probe", "--stats", str(probe_stats_path)],
            env=os.environ.copy(),
            stdout=subprocess.DEVNULL,
        )
        probe_ready_deadline = time.monotonic() + 10.0
        while time.monotonic() < probe_ready_deadline:
            if probe.poll() is not None:
                raise RuntimeError(f"Independent DDS probe exited with status {probe.returncode}")
            if _read_json(probe_stats_path).get("ready") is True:
                break
            time.sleep(0.1)
        else:
            raise TimeoutError("Independent DDS probe did not become ready")

        publisher = subprocess.Popen(
            [sys.executable, str(Path(__file__).resolve()), "--publisher", "--stats", str(publisher_stats_path)],
            env=os.environ.copy(),
            stdout=subprocess.DEVNULL,
        )
        last_telemetry: dict[str, Any] | None = None
        stream_reached_streaming = False
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            if publisher.poll() is not None:
                raise RuntimeError(f"Synthetic publisher exited with status {publisher.returncode}")
            if probe.poll() is not None:
                raise RuntimeError(f"Independent DDS probe exited with status {probe.returncode}")
            telemetry = _http_json("/v1/telemetry")
            if telemetry is not None:
                last_telemetry = telemetry
                stream = next((item for item in telemetry.get("streams", []) if item.get("topic") == TOPIC), None)
                probe_stats = _read_json(probe_stats_path)
                if stream and stream.get("status") == "streaming" and probe_stats.get("receive_count", 0) > 0:
                    stream_reached_streaming = True
                    break
            time.sleep(0.25)

        ci_before = _cpu_cgroup_snapshot()
        load = _sample_phase(
            agent.pid,
            load_seconds,
            companion_pids={"publisher": publisher.pid, "independent_dds_probe": probe.pid},
        )
        ci_after = _cpu_cgroup_snapshot()
        publisher.send_signal(signal.SIGINT)
        try:
            publisher.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            publisher.terminate()
            publisher.wait(timeout=5.0)
        probe.send_signal(signal.SIGINT)
        try:
            probe.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            probe.terminate()
            probe.wait(timeout=5.0)
        publisher_shutdown_clean = publisher.returncode == 0
        probe_shutdown_clean = probe.returncode == 0
        publisher_stats = _read_json(publisher_stats_path)
        probe_stats = _read_json(probe_stats_path)
        load_start = float(load["started_monotonic_seconds"])
        load_end = float(load["finished_monotonic_seconds"])
        load_elapsed = max(0.001, load_end - load_start)
        publish_times = [float(item) for item in publisher_stats.get("publish_monotonic_seconds", [])]
        probe_times = [float(item) for item in probe_stats.get("receive_monotonic_seconds", [])]
        agent_received_in_window = int(load.get("pointcloud_agent_receive_count_delta") or 0)
        delivery = _summarize_deliveries(
            publish_times,
            probe_times,
            agent_received_count=agent_received_in_window,
            start=load_start,
            end=load_end,
            probe_sequences=[int(item) for item in probe_stats.get("sequence_numbers", [])],
        )
        published_in_window = delivery["publisher_count"]
        probe_received_in_window = delivery["probe_count"]
        publisher_rate = delivery["publisher_rate_hz"]
        dds_probe_rate = delivery["probe_rate_hz"]
        publisher_to_probe_difference = delivery["publisher_probe_event_count_delta"]
        probe_to_agent_difference = delivery["probe_agent_event_count_delta_estimate"]
        ci_throttling = _cpu_throttling_delta(ci_before, ci_after, load_elapsed)
        pipeline_diagnosis = _diagnose_pipeline(
            publisher_rate,
            dds_probe_rate,
            load.get("pointcloud_receive_rate_hz_p05"),
            ci_pressure_observed=ci_throttling["pressure_observed"],
            publisher_cpu_percent=(
                (load.get("tracked_processes", {}).get("publisher") or {}).get("cpu_percent_of_one_core_average")
            ),
            probe_cpu_percent=(
                (load.get("tracked_processes", {}).get("independent_dds_probe") or {}).get(
                    "cpu_percent_of_one_core_average"
                )
            ),
            agent_cpu_percent=load.get("agent_cpu_percent_of_one_core_average"),
        )
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
            "measurement_window_seconds": round(load_elapsed, 3),
            "stream_reached_streaming_before_measurement": stream_reached_streaming,
            "baseline": baseline,
            "load": load,
            "pipeline_observations": {
                "publisher": {
                    "requested_rate_hz": PUBLISH_HZ,
                    "timer_callback_count_total": publisher_stats.get("timer_callback_count", 0),
                    "published_message_count_total": publisher_stats.get("published_message_count", 0),
                    "published_message_count_in_window": published_in_window,
                    "actual_rate_hz_in_window": publisher_rate,
                    "overall_measured_rate_hz": publisher_stats.get("measured_publish_rate_hz"),
                    "publish_interval_ms_p95": publisher_stats.get("publish_interval_ms_p95"),
                    "timer_callback_interval_ms_p95": publisher_stats.get("timer_callback_interval_ms_p95"),
                    "matched_subscription_count_at_last_sample": publisher_stats.get("matched_subscription_count"),
                    "last_sequence": publisher_stats.get("last_sequence"),
                    "process_resources": load.get("tracked_processes", {}).get("publisher"),
                },
                "independent_dds_probe": {
                    "received_message_count_total": probe_stats.get("receive_count", 0),
                    "received_message_count_in_window": probe_received_in_window,
                    "actual_rate_hz_in_window": dds_probe_rate,
                    "overall_measured_rate_hz": probe_stats.get("measured_receive_rate_hz"),
                    "last_sequence": probe_stats.get("last_sequence"),
                    "process_resources": load.get("tracked_processes", {}).get("independent_dds_probe"),
                },
                "agent": {
                    "received_message_count_in_window": agent_received_in_window,
                    "receive_rate_p05_hz": receive_p05,
                    "receive_rate_median_hz": load.get("pointcloud_receive_rate_hz_median"),
                    "receive_count_peak_within_subscription_generation": load.get("pointcloud_receive_count_peak"),
                    "subscription_generations_observed": load.get("pointcloud_subscription_generations_observed"),
                    "stream_statuses": load.get("stream_statuses"),
                },
            },
            "loss_estimates": {
                "publisher_minus_probe_event_count_delta": publisher_to_probe_difference,
                "publisher_to_probe_sequence_delivered_count": delivery["publisher_probe_sequence_delivered_count"],
                "publisher_to_probe_sequence_missing_count": delivery["publisher_probe_sequence_missing_count"],
                "probe_minus_agent_event_count_delta_estimate": probe_to_agent_difference,
                "method_note": (
                    "Publisher-to-probe sequence loss counts messages published in the measurement window "
                    "and observed by the probe before shutdown. Probe and Agent count differences are estimates; "
                    "Agent callback count is differenced across polled telemetry snapshots and may have a boundary "
                    "error of up to one poll interval. The Agent endpoint does not expose per-message sequence history."
                ),
            },
            "ci_resources": {
                "before": ci_before,
                "after": ci_after,
                "resource_pressure": ci_throttling,
            },
            "pipeline_diagnosis": pipeline_diagnosis,
            "impact": {
                "agent_cpu_delta_percent_of_one_core": round(
                    load["agent_cpu_percent_of_one_core_average"] - baseline["agent_cpu_percent_of_one_core_average"], 2
                ),
                "agent_rss_peak_increase_bytes_vs_baseline_median": rss_increase,
                "graph_age_p95_ms_during_load": graph_p95,
                "graph_observation_count_during_load": load["distinct_graph_observations"],
                "synthetic_stream_rate_p05_hz": receive_p05,
                "publisher_actual_rate_hz": publisher_rate,
                "independent_dds_probe_rate_hz": dds_probe_rate,
                "publisher_to_probe_sequence_missing_count": delivery["publisher_probe_sequence_missing_count"],
                "probe_to_agent_event_count_delta_estimate": delivery["probe_agent_event_count_delta_estimate"],
                "stream_returns_to_unavailable_after_publisher_stop": recovered_to_unavailable,
            },
            "acceptance": {
                "agent_stream_reached_streaming": stream_reached_streaming,
                "publisher_shutdown_clean": publisher_shutdown_clean,
                "independent_dds_probe_shutdown_clean": probe_shutdown_clean,
                "agent_cpu_average_below_one_core": load["agent_cpu_percent_of_one_core_average"] < 100.0,
                "agent_rss_increase_below_128_mib": rss_increase is not None and rss_increase < 128 * 1024 * 1024,
                "graph_age_p95_below_5000_ms": graph_p95 is not None and graph_p95 < 5_000,
                "graph_observed_at_least_5_times": load["distinct_graph_observations"] >= 5,
                "publisher_actual_rate_at_least_5_hz": publisher_rate >= MIN_ACCEPTED_RATE_HZ,
                "independent_dds_probe_rate_at_least_5_hz": dds_probe_rate >= MIN_ACCEPTED_RATE_HZ,
                "pointcloud_receive_rate_p05_at_least_5_hz": (
                    receive_p05 is not None and receive_p05 >= MIN_ACCEPTED_RATE_HZ
                ),
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
        if probe is not None and probe.poll() is None:
            probe.terminate()
            try:
                probe.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                probe.kill()
        if agent.poll() is None:
            agent.terminate()
            try:
                agent.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                agent.kill()
        try:
            for diagnostics_file in diagnostics_dir.iterdir():
                diagnostics_file.unlink(missing_ok=True)
            diagnostics_dir.rmdir()
        except OSError:
            pass


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--publisher", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--probe", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--stats", type=Path, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--report", type=Path, default=Path("/tmp/lunar-link-telemetry-benchmark.json"))
    parser.add_argument("--baseline-seconds", type=float, default=6.0)
    parser.add_argument("--load-seconds", type=float, default=12.0)
    args = parser.parse_args()
    if args.publisher:
        return _publisher_main(str(args.stats) if args.stats else None)
    if args.probe:
        return _dds_probe_main(str(args.stats) if args.stats else None)
    run_benchmark(args.report, args.baseline_seconds, args.load_seconds)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
