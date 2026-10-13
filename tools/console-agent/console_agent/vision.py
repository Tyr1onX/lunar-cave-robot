from __future__ import annotations

import struct
import threading
import time
import zlib
from collections import deque
from datetime import datetime, timezone
from typing import Any


STREAMS: dict[str, dict[str, str]] = {
    "rgb": {"label": "RGB", "topic": "/camera/color/image_raw"},
    "depth": {"label": "深度", "topic": "/camera/depth/image_rect_raw"},
    "yolo": {"label": "YOLO", "topic": "/yolo/annotated_image"},
}
IMAGE_TYPE = "sensor_msgs/msg/Image"
STALE_AFTER_SECONDS = 1.5
MAX_WIDTH = 4096
MAX_HEIGHT = 4096
MAX_DATA_BYTES = 32 * 1024 * 1024
ENCODING_BYTES = {"rgb8": 3, "bgr8": 3, "rgba8": 4, "bgra8": 4, "mono8": 1, "8uc1": 1, "16uc1": 2, "mono16": 2}


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _stamp_iso(seconds: int, nanoseconds: int) -> str | None:
    if seconds <= 0 or not 0 <= nanoseconds < 1_000_000_000:
        return None
    try:
        return datetime.fromtimestamp(seconds + nanoseconds / 1_000_000_000, timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
    except (OverflowError, OSError, ValueError):
        return None


def _png_chunk(kind: bytes, data: bytes) -> bytes:
    payload = kind + data
    return struct.pack(">I", len(data)) + payload + struct.pack(">I", zlib.crc32(payload) & 0xFFFFFFFF)


def _rows(data: bytes, width: int, height: int, step: int, bytes_per_pixel: int) -> list[bytes]:
    minimum_step = width * bytes_per_pixel
    if step < minimum_step or len(data) < step * height:
        raise ValueError("Image data is shorter than its declared dimensions")
    return [data[row * step:row * step + minimum_step] for row in range(height)]


def encode_image_png(frame: dict[str, Any]) -> tuple[bytes, str | None]:
    """Convert supported ROS Image encodings to a bounded, dependency-free PNG."""
    width = int(frame["width"])
    height = int(frame["height"])
    step = int(frame["step"])
    data = frame["data"]
    encoding = str(frame["encoding"]).lower()
    if not 0 < width <= MAX_WIDTH or not 0 < height <= MAX_HEIGHT:
        raise ValueError("Image dimensions are outside the supported range")
    if len(data) > MAX_DATA_BYTES:
        raise ValueError("Image data exceeds the supported size")

    note = None
    if encoding in {"rgb8", "bgr8", "rgba8", "bgra8"}:
        channels = 4 if encoding in {"rgba8", "bgra8"} else 3
        source_rows = _rows(data, width, height, step, channels)
        rows: list[bytes] = []
        for source in source_rows:
            row = bytearray(width * 3)
            for pixel in range(width):
                offset = pixel * channels
                red, green, blue = source[offset:offset + 3]
                if encoding in {"bgr8", "bgra8"}:
                    red, blue = blue, red
                target = pixel * 3
                row[target:target + 3] = bytes((red, green, blue))
            rows.append(bytes(row))
        color_type = 2
    elif encoding in {"mono8", "8uc1"}:
        rows = _rows(data, width, height, step, 1)
        color_type = 0
    elif encoding in {"16uc1", "mono16"}:
        source_rows = _rows(data, width, height, step, 2)
        byte_order = "big" if frame.get("is_bigendian") else "little"
        values = [int.from_bytes(source[index:index + 2], byte_order) for source in source_rows for index in range(0, len(source), 2)]
        valid = [value for value in values if value > 0]
        low, high = (min(valid), max(valid)) if valid else (0, 0)
        span = max(1, high - low)
        rows = []
        for source in source_rows:
            row = bytearray(width)
            for pixel in range(width):
                value = int.from_bytes(source[pixel * 2:pixel * 2 + 2], byte_order)
                row[pixel] = 0 if value == 0 else max(0, min(255, round((value - low) * 255 / span)))
            rows.append(bytes(row))
        color_type = 0
        note = "16 位深度按当前帧有效范围拉伸为灰度；亮度不可跨帧比较，也不是距离刻度"
    else:
        raise ValueError(f"Unsupported ROS image encoding: {frame['encoding']}")

    scanlines = b"".join(b"\x00" + row for row in rows)
    header = struct.pack(">IIBBBBB", width, height, 8, color_type, 0, 0, 0)
    return b"\x89PNG\r\n\x1a\n" + _png_chunk(b"IHDR", header) + _png_chunk(b"IDAT", zlib.compress(scanlines, 1)) + _png_chunk(b"IEND", b""), note


class VisionObserver:
    """Keeps the latest frames from fixed, read-only ROS Image topics in memory."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._publishers: dict[str, tuple[int, list[str]]] = {key: (0, []) for key in STREAMS}
        self._frames: dict[str, dict[str, Any]] = {}
        self._received: dict[str, deque[float]] = {key: deque(maxlen=120) for key in STREAMS}
        self._gap_events: dict[str, int] = {key: 0 for key in STREAMS}
        self._last_received_mono: dict[str, float] = {}
        self._receive_errors: dict[str, str] = {}
        self._graph_status = "starting"
        self._graph_reason = "ROS graph observer is starting"

    def update_publishers(self, stream_id: str, types: list[str], publisher_count: int | None = None) -> None:
        if stream_id not in STREAMS:
            return
        with self._lock:
            self._publishers[stream_id] = (len(types) if publisher_count is None else publisher_count, sorted(set(types)))

    def graph_available(self) -> None:
        with self._lock:
            self._graph_status = "ok"
            self._graph_reason = ""

    def graph_unavailable(self, reason: str) -> None:
        with self._lock:
            self._graph_status = "unknown"
            self._graph_reason = reason

    def receive(self, stream_id: str, message: Any) -> None:
        if stream_id not in STREAMS:
            return
        width, height, step = int(message.width), int(message.height), int(message.step)
        source_data = message.data
        encoding = str(message.encoding)
        bytes_per_pixel = ENCODING_BYTES.get(encoding.lower())
        if not 0 < width <= MAX_WIDTH or not 0 < height <= MAX_HEIGHT:
            return
        if len(source_data) > MAX_DATA_BYTES:
            with self._lock:
                self._receive_errors[stream_id] = "图像帧超过 32 MiB 安全上限"
            return
        if bytes_per_pixel is None:
            with self._lock:
                self._receive_errors[stream_id] = f"不支持的 ROS 图像编码：{encoding}"
            return
        if step < width * bytes_per_pixel or len(source_data) < step * height:
            with self._lock:
                self._receive_errors[stream_id] = "图像数据长度与尺寸/步长不一致"
            return
        raw = bytes(source_data)
        seconds = int(message.header.stamp.sec)
        nanoseconds = int(message.header.stamp.nanosec)
        now_mono = time.monotonic()
        now_wall = time.time()
        with self._lock:
            previous = self._last_received_mono.get(stream_id)
            if previous is not None and now_mono - previous > 1.5:
                self._gap_events[stream_id] += 1
            self._last_received_mono[stream_id] = now_mono
            self._received[stream_id].append(now_mono)
            self._frames[stream_id] = {
                "width": width,
                "height": height,
                "step": step,
                "encoding": encoding,
                "is_bigendian": bool(getattr(message, "is_bigendian", False)),
                "data": raw,
                "stamp_seconds": seconds,
                "stamp_nanoseconds": nanoseconds,
                "source_stamp": _stamp_iso(seconds, nanoseconds),
                "received_at": _utc_now(),
                "received_epoch": now_wall,
                "received_mono": now_mono,
            }
            self._receive_errors.pop(stream_id, None)

    def snapshot(self) -> dict[str, Any]:
        now_mono = time.monotonic()
        readings: list[dict[str, Any]] = []
        with self._lock:
            graph_status = self._graph_status
            graph_reason = self._graph_reason
            for stream_id, definition in STREAMS.items():
                publisher_count, publisher_types = self._publishers[stream_id]
                frame = self._frames.get(stream_id)
                if graph_status != "ok":
                    status, reason = "unknown", graph_reason or "ROS graph unavailable"
                elif publisher_count == 0:
                    status, reason = "unavailable", "ROS Topic 当前没有发布者"
                elif IMAGE_TYPE not in publisher_types:
                    status, reason = "unsupported", f"Topic 类型不兼容：{', '.join(publisher_types)}"
                elif frame is None:
                    status, reason = ("unsupported", self._receive_errors[stream_id]) if stream_id in self._receive_errors else ("waiting", "Topic 有发布者，尚未收到图像帧")
                elif stream_id in self._receive_errors:
                    status, reason = "unsupported", self._receive_errors[stream_id]
                else:
                    age = max(0.0, now_mono - frame["received_mono"])
                    status = "streaming" if age <= STALE_AFTER_SECONDS else "stale"
                    reason = None if status == "streaming" else "图像帧已超时，实时画面已停止更新"
                visible_frame = frame if status in {"streaming", "stale"} and stream_id not in self._receive_errors else None

                samples = self._received[stream_id]
                while samples and now_mono - samples[0] > 5.0:
                    samples.popleft()
                fps = None
                if visible_frame is not None and len(samples) >= 2 and samples[-1] > samples[0]:
                    fps = round((len(samples) - 1) / (samples[-1] - samples[0]), 2)
                age_ms = None
                stamp_delta_ms = None
                if visible_frame is not None:
                    age_ms = round(max(0.0, now_mono - visible_frame["received_mono"]) * 1000)
                    stamp = visible_frame["stamp_seconds"] + visible_frame["stamp_nanoseconds"] / 1_000_000_000
                    delta = (visible_frame["received_epoch"] - stamp) * 1000
                    if visible_frame["source_stamp"] is not None and -1000 <= delta <= 120_000:
                        stamp_delta_ms = round(delta, 2)

                readings.append({
                    "id": stream_id,
                    "label": definition["label"],
                    "topic": definition["topic"],
                    "topic_type": IMAGE_TYPE if IMAGE_TYPE in publisher_types else (publisher_types[0] if publisher_types else None),
                    "status": status,
                    "reason": reason,
                    "publisher_count": publisher_count,
                    "width": visible_frame["width"] if visible_frame else None,
                    "height": visible_frame["height"] if visible_frame else None,
                    "encoding": visible_frame["encoding"] if visible_frame else None,
                    "frame_rate_hz": fps,
                    "frame_age_ms": age_ms,
                    "source_stamp": visible_frame["source_stamp"] if visible_frame else None,
                    "header_to_agent_ms": stamp_delta_ms,
                    "received_at": visible_frame["received_at"] if visible_frame else None,
                    "gap_events": self._gap_events[stream_id],
                })
        return {"status": graph_status, "source": "nuc-ros2-read-only-subscriber", "collected_at": _utc_now(), "reason": graph_reason or None, "streams": readings}

    def frame_png(self, stream_id: str) -> tuple[bytes | None, str | None, int | None]:
        if stream_id not in STREAMS:
            return None, "unknown visual source", 404
        with self._lock:
            frame = self._frames.get(stream_id)
            publisher_count, publisher_types = self._publishers[stream_id]
            age = None if frame is None else max(0.0, time.monotonic() - frame["received_mono"])
            graph_status = self._graph_status
            graph_reason = self._graph_reason
            if graph_status != "ok":
                return None, graph_reason or "ROS graph unavailable", 503
            if publisher_count == 0:
                return None, "ROS Topic 当前没有发布者", 503
            if IMAGE_TYPE not in publisher_types:
                return None, "ROS Topic 类型不受支持", 415
            if stream_id in self._receive_errors:
                return None, self._receive_errors[stream_id], 415
            if frame is None:
                return None, "尚未收到图像帧", 503
            if age is not None and age > STALE_AFTER_SECONDS:
                return None, "图像帧已超时", 503
            current = dict(frame)
        try:
            png, _note = encode_image_png(current)
            return png, None, None
        except (KeyError, TypeError, ValueError, OverflowError) as exc:
            return None, str(exc), 415
