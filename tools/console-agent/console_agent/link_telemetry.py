from __future__ import annotations

import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Any, Callable


TELEMETRY_STREAMS: dict[str, dict[str, Any]] = {
    "livox_lidar": {
        "label": "MID-360 点云",
        "topic": "/livox/lidar",
        "message_types": ["livox_ros_driver2/msg/CustomMsg", "sensor_msgs/msg/PointCloud2"],
        "sample_unit": "points",
        "stale_after_ms": 1_500,
    },
    "livox_imu": {
        "label": "MID-360 原始 IMU",
        "topic": "/livox/imu",
        "message_types": ["sensor_msgs/msg/Imu"],
        "sample_unit": None,
        "stale_after_ms": 1_500,
    },
    "imu_transformed": {
        "label": "坐标变换 IMU",
        "topic": "/livox/imu_aft_tf",
        "message_types": ["sensor_msgs/msg/Imu"],
        "sample_unit": None,
        "stale_after_ms": 1_500,
    },
    "imu_data": {
        "label": "Point-LIO IMU 输入",
        "topic": "/imu/data",
        "message_types": ["sensor_msgs/msg/Imu"],
        "sample_unit": None,
        "stale_after_ms": 1_500,
    },
    "pointlio_odometry": {
        "label": "Point-LIO 里程计输出",
        "topic": "/odometry",
        "message_types": ["nav_msgs/msg/Odometry"],
        "sample_unit": None,
        "stale_after_ms": 2_500,
    },
    "cloud_registered": {
        "label": "Point-LIO 注册点云输出",
        "topic": "/cloud_registered",
        "message_types": ["sensor_msgs/msg/PointCloud2"],
        "sample_unit": "points",
        "stale_after_ms": 2_500,
    },
    "terrain_map": {
        "label": "地形过滤点云",
        "topic": "/terrain_map",
        "message_types": ["sensor_msgs/msg/PointCloud2"],
        "sample_unit": "points",
        "stale_after_ms": 2_500,
    },
    "scan": {
        "label": "SLAM LaserScan 输入",
        "topic": "/scan",
        "message_types": ["sensor_msgs/msg/LaserScan"],
        "sample_unit": "ranges",
        "stale_after_ms": 2_500,
    },
    "map": {
        "label": "SLAM OccupancyGrid 输出",
        "topic": "/map",
        "message_types": ["nav_msgs/msg/OccupancyGrid"],
        "sample_unit": "cells",
        "stale_after_ms": 5_000,
    },
}

TOPIC_TO_STREAM = {definition["topic"]: stream_id for stream_id, definition in TELEMETRY_STREAMS.items()}
RATE_WINDOW_SECONDS = 5.0


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _stamp_epoch(stamp: Any) -> float | None:
    if stamp is None:
        return None
    try:
        seconds = int(stamp.sec)
        nanoseconds = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError):
        return None
    if seconds <= 0 or not 0 <= nanoseconds < 1_000_000_000:
        return None
    return seconds + nanoseconds / 1_000_000_000


def _stamp_iso(epoch: float | None) -> str | None:
    if epoch is None:
        return None
    try:
        return datetime.fromtimestamp(epoch, timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
    except (OverflowError, OSError, ValueError):
        return None


def _metadata(message: Any) -> tuple[str | None, str | None, int | None, str | None, float | None]:
    """Read only constant-time header and shape fields; never inspect point, range, or map arrays."""
    header = getattr(message, "header", None)
    frame_id = getattr(header, "frame_id", None) if header is not None else None
    source_epoch = _stamp_epoch(getattr(header, "stamp", None) if header is not None else None)
    sample_count: int | None = None
    unit: str | None = None

    point_num = getattr(message, "point_num", None)
    if isinstance(point_num, int) and point_num >= 0:
        sample_count, unit = point_num, "points"
    elif hasattr(message, "width") and hasattr(message, "height"):
        try:
            width, height = int(message.width), int(message.height)
            if width >= 0 and height >= 0:
                sample_count, unit = width * height, "points"
        except (TypeError, ValueError):
            pass
    elif hasattr(message, "ranges"):
        try:
            sample_count, unit = len(message.ranges), "ranges"
        except (TypeError, ValueError):
            pass
    elif hasattr(message, "info"):
        info = message.info
        try:
            width, height = int(info.width), int(info.height)
            if width >= 0 and height >= 0:
                sample_count, unit = width * height, "cells"
        except (AttributeError, TypeError, ValueError):
            pass

    return frame_id, _stamp_iso(source_epoch), sample_count, unit, source_epoch


class LinkTelemetryObserver:
    """Bounded, read-only receive metrics for a fixed ROS topic allowlist."""

    def __init__(
        self,
        *,
        monotonic: Callable[[], float] = time.monotonic,
        utc_now: Callable[[], str] = _utc_now,
    ) -> None:
        self._lock = threading.Lock()
        self._monotonic = monotonic
        self._utc_now = utc_now
        self._graph_status = "unknown"
        self._graph_reason = "ROS graph observer is starting"
        self._domain_id: int | None = None
        self._expected_domain_id: int | None = None
        self._expected_domain_error: str | None = None
        self._graph_observed_at: str | None = None
        self._graph_observed_mono: float | None = None
        self._available_message_types: set[str] = set()
        self._streams: dict[str, dict[str, Any]] = {
            stream_id: {
                "publisher_count": None,
                "publisher_endpoints": [],
                "topic_types": [],
                "subscription_type": None,
                "subscription_reason": "waiting for a matching ROS publisher",
                "subscription_generation": 0,
                "subscription_started_at": None,
                "subscription_started_mono": None,
                "previous_received_at": None,
                "previous_subscription_generation": None,
                "last_received_mono": None,
                "received_at": None,
                "source_stamp": None,
                "header_to_agent_ms": None,
                "frame_id": None,
                "sample_count": None,
                "sample_unit": definition["sample_unit"],
                "received_mono": deque(maxlen=128),
                "gap_events": 0,
                "last_gap_ms": None,
                "receive_count": 0,
            }
            for stream_id, definition in TELEMETRY_STREAMS.items()
        }

    def set_available_message_types(self, message_types: set[str]) -> None:
        with self._lock:
            self._available_message_types = set(message_types)

    def graph_available(
        self,
        *,
        domain_id: int | None,
        observed_at: str,
        expected_domain_id: int | None,
        expected_domain_error: str | None,
    ) -> None:
        with self._lock:
            self._graph_status = "ok"
            self._graph_reason = ""
            self._domain_id = domain_id
            self._expected_domain_id = expected_domain_id
            self._expected_domain_error = expected_domain_error
            self._graph_observed_at = observed_at
            self._graph_observed_mono = self._monotonic()

    def graph_unavailable(self, reason: str) -> None:
        with self._lock:
            self._graph_status = "unknown"
            self._graph_reason = reason

    def update_publishers(self, stream_id: str, endpoints: list[dict[str, str]] | None) -> None:
        if stream_id not in TELEMETRY_STREAMS:
            return
        if endpoints is None:
            with self._lock:
                stream = self._streams[stream_id]
                stream["publisher_count"] = None
                stream["publisher_endpoints"] = []
                stream["topic_types"] = []
                stream["subscription_reason"] = "publisher endpoint graph read failed"
            return
        cleaned = [
            {
                "name": str(endpoint.get("name") or ""),
                "namespace": str(endpoint.get("namespace") or ""),
                "topic_type": str(endpoint.get("topic_type") or ""),
            }
            for endpoint in endpoints
        ]
        topic_types = sorted({endpoint["topic_type"] for endpoint in cleaned if endpoint["topic_type"]})
        with self._lock:
            stream = self._streams[stream_id]
            stream["publisher_count"] = len(cleaned)
            stream["publisher_endpoints"] = sorted(cleaned, key=lambda item: (item["namespace"], item["name"], item["topic_type"]))
            stream["topic_types"] = topic_types
            if len(topic_types) != 1:
                stream["subscription_reason"] = "publisher type is missing or ambiguous" if cleaned else "no publisher is currently visible in this ROS Domain"
            elif topic_types[0] not in TELEMETRY_STREAMS[stream_id]["message_types"]:
                stream["subscription_reason"] = "publisher message type is not in the fixed telemetry allowlist"
            elif topic_types[0] not in self._available_message_types:
                stream["subscription_reason"] = "ROS message type support is not loaded in the Agent environment"
            else:
                stream["subscription_reason"] = "matching read-only subscription is being created"

    def subscription_ready(self, stream_id: str, message_type: str) -> None:
        if stream_id not in TELEMETRY_STREAMS:
            return
        with self._lock:
            stream = self._streams[stream_id]
            stream["subscription_type"] = message_type
            stream["subscription_reason"] = None

    def begin_subscription(
        self,
        stream_id: str,
        message_type: str,
        *,
        started_mono: float | None = None,
        started_at: str | None = None,
    ) -> int | None:
        """Start a fresh receive epoch and return its callback token."""
        if stream_id not in TELEMETRY_STREAMS:
            return None
        now_mono = self._monotonic() if started_mono is None else started_mono
        now_text = self._utc_now() if started_at is None else started_at
        with self._lock:
            stream = self._streams[stream_id]
            self._archive_current_receive(stream)
            stream["subscription_generation"] += 1
            stream["subscription_started_at"] = now_text
            stream["subscription_started_mono"] = now_mono
            stream["subscription_type"] = message_type
            stream["subscription_reason"] = None
            self._clear_current_receive(stream)
            return stream["subscription_generation"]

    @staticmethod
    def _archive_current_receive(stream: dict[str, Any]) -> None:
        if stream["received_at"] is not None:
            stream["previous_received_at"] = stream["received_at"]
            stream["previous_subscription_generation"] = stream["subscription_generation"]

    @staticmethod
    def _clear_current_receive(stream: dict[str, Any]) -> None:
        stream["last_received_mono"] = None
        stream["received_at"] = None
        stream["source_stamp"] = None
        stream["header_to_agent_ms"] = None
        stream["frame_id"] = None
        stream["sample_count"] = None
        stream["sample_unit"] = None
        stream["received_mono"].clear()
        stream["receive_count"] = 0
        stream["gap_events"] = 0
        stream["last_gap_ms"] = None

    def subscription_unavailable(self, stream_id: str, message_type: str | None, reason: str) -> None:
        if stream_id not in TELEMETRY_STREAMS:
            return
        with self._lock:
            stream = self._streams[stream_id]
            self._archive_current_receive(stream)
            self._clear_current_receive(stream)
            stream["subscription_type"] = None
            stream["subscription_started_at"] = None
            stream["subscription_started_mono"] = None
            stream["subscription_reason"] = reason

    def receive(
        self,
        stream_id: str,
        message: Any,
        message_type: str,
        *,
        received_mono: float | None = None,
        received_at: str | None = None,
        subscription_generation: int | None = None,
    ) -> None:
        if stream_id not in TELEMETRY_STREAMS:
            return
        now_mono = self._monotonic() if received_mono is None else received_mono
        now_text = self._utc_now() if received_at is None else received_at
        frame_id, source_stamp, sample_count, sample_unit, source_epoch = _metadata(message)
        try:
            received_epoch = datetime.fromisoformat(now_text.replace("Z", "+00:00")).timestamp()
        except (TypeError, ValueError):
            received_epoch = None
        latency = None if source_epoch is None or received_epoch is None else round((received_epoch - source_epoch) * 1000, 1)
        definition = TELEMETRY_STREAMS[stream_id]
        with self._lock:
            stream = self._streams[stream_id]
            if stream["subscription_type"] != message_type:
                return
            if subscription_generation is not None and subscription_generation != stream["subscription_generation"]:
                return
            previous = stream["last_received_mono"]
            if previous is not None:
                gap_ms = max(0.0, (now_mono - previous) * 1000)
                if gap_ms > definition["stale_after_ms"]:
                    stream["gap_events"] += 1
                    stream["last_gap_ms"] = round(gap_ms)
            stream["last_received_mono"] = now_mono
            stream["received_at"] = now_text
            stream["source_stamp"] = source_stamp
            stream["header_to_agent_ms"] = latency
            stream["frame_id"] = str(frame_id) if frame_id else None
            stream["sample_count"] = sample_count
            stream["sample_unit"] = sample_unit or definition["sample_unit"]
            stream["receive_count"] += 1
            stream["received_mono"].append(now_mono)
            stream["subscription_type"] = message_type
            stream["subscription_reason"] = None

    def snapshot(self, *, now_mono: float | None = None, collected_at: str | None = None) -> dict[str, Any]:
        now = self._monotonic() if now_mono is None else now_mono
        collected = self._utc_now() if collected_at is None else collected_at
        with self._lock:
            graph_status = self._graph_status
            graph_age_ms = None if self._graph_observed_mono is None else round(max(0.0, now - self._graph_observed_mono) * 1000)
            domain_matches = None
            if self._expected_domain_error is None and self._expected_domain_id is not None and self._domain_id is not None:
                domain_matches = self._expected_domain_id == self._domain_id
            result = {
                "schema_version": 1,
                "status": "ok" if graph_status == "ok" else "unknown",
                "source": "nuc-ros2-fixed-topic-read-only-subscriber",
                "collected_at": collected,
                "domain_id": self._domain_id,
                "expected_domain_id": self._expected_domain_id,
                "expected_domain_error": self._expected_domain_error,
                "domain_matches_expectation": domain_matches,
                "graph_status": graph_status,
                "graph_observed_at": self._graph_observed_at,
                "graph_age_ms": graph_age_ms,
                "graph_reason": self._graph_reason or None,
                "workspace_attribution": "unknown",
                "clock_note": "header_to_agent_ms compares the ROS source stamp with Agent wall time; clock synchronization is not verified",
                "streams": [],
            }
            for stream_id, definition in TELEMETRY_STREAMS.items():
                stream = self._streams[stream_id]
                publishers = stream["publisher_count"]
                topic_types = list(stream["topic_types"])
                subscription_type = stream["subscription_type"]
                last_received = stream["last_received_mono"]
                age_ms = None if last_received is None else round(max(0.0, now - last_received) * 1000)
                if graph_status != "ok":
                    status, reason = "unknown", self._graph_reason or "ROS Graph evidence is unavailable"
                elif publishers is None:
                    status, reason = "unknown", "publisher endpoint evidence is unavailable"
                elif publishers == 0:
                    status, reason = "unavailable", "ROS Graph currently reports no publisher in the Agent Domain; this is not a hardware-health conclusion"
                elif len(topic_types) != 1:
                    status, reason = "unknown", "publisher message type is missing or ambiguous"
                elif topic_types[0] not in definition["message_types"]:
                    status, reason = "unsupported", f"publisher type {topic_types[0]} is outside the fixed telemetry allowlist"
                elif topic_types[0] not in self._available_message_types:
                    status, reason = "unsupported", "ROS message type support is not loaded in the Agent environment"
                elif subscription_type != topic_types[0]:
                    status, reason = "waiting", stream["subscription_reason"] or "waiting for the fixed read-only subscriber"
                elif last_received is None:
                    status, reason = "waiting", "publisher is visible but no message has reached the Agent subscriber"
                elif age_ms is not None and age_ms > definition["stale_after_ms"]:
                    status, reason = "stale", f"no message received within {definition['stale_after_ms']} ms"
                else:
                    status, reason = "streaming", None

                rate_times = [value for value in stream["received_mono"] if now - value <= RATE_WINDOW_SECONDS]
                rate = None
                if len(rate_times) >= 2 and rate_times[-1] > rate_times[0]:
                    rate = round((len(rate_times) - 1) / (rate_times[-1] - rate_times[0]), 2)
                result["streams"].append({
                    "id": stream_id,
                    "label": definition["label"],
                    "topic": definition["topic"],
                    "topic_type": topic_types[0] if len(topic_types) == 1 else None,
                    "topic_types": topic_types,
                    "status": status,
                    "reason": reason,
                    "publisher_count": publishers,
                    "publisher_endpoints": list(stream["publisher_endpoints"]),
                    "workspace_attribution": "unknown",
                    "subscription_generation": stream["subscription_generation"],
                    "subscription_started_at": stream["subscription_started_at"],
                    "previous_received_at": stream["previous_received_at"],
                    "previous_subscription_generation": stream["previous_subscription_generation"],
                    "received_at": stream["received_at"],
                    "source_stamp": stream["source_stamp"],
                    "age_ms": age_ms,
                    "receive_rate_hz": rate,
                    "header_to_agent_ms": stream["header_to_agent_ms"],
                    "frame_id": stream["frame_id"],
                    "sample_count": stream["sample_count"],
                    "sample_unit": stream["sample_unit"],
                    "gap_events": stream["gap_events"],
                    "last_gap_ms": stream["last_gap_ms"],
                    "stale_after_ms": definition["stale_after_ms"],
                    "receive_count": stream["receive_count"],
                })
            return result
