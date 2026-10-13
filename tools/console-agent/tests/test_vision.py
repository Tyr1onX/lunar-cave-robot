from __future__ import annotations

import struct
import time
import tracemalloc
import unittest
import zlib
from datetime import datetime, timezone
from types import SimpleNamespace

from console_agent.vision import IMAGE_TYPE, VisionObserver, encode_image_png


def decode_png(image: bytes) -> tuple[int, int, int, bytes]:
    if not image.startswith(b"\x89PNG\r\n\x1a\n"):
        raise AssertionError("not a PNG")
    offset = 8
    compressed = bytearray()
    width = height = bit_depth = color_type = 0
    while offset < len(image):
        length = struct.unpack(">I", image[offset:offset + 4])[0]
        kind = image[offset + 4:offset + 8]
        payload = image[offset + 8:offset + 8 + length]
        offset += 12 + length
        if kind == b"IHDR":
            width, height, bit_depth, color_type, _, _, _ = struct.unpack(">IIBBBBB", payload)
        elif kind == b"IDAT":
            compressed.extend(payload)
        elif kind == b"IEND":
            break
    return width, height, bit_depth, zlib.decompress(compressed)


def image_message(encoding: str, data: bytes, *, width: int = 1, height: int = 1, step: int | None = None) -> SimpleNamespace:
    if step is None:
        step = len(data) // height
    now = datetime.now(timezone.utc).timestamp()
    seconds = int(now)
    nanoseconds = int((now - seconds) * 1_000_000_000)
    return SimpleNamespace(
        width=width,
        height=height,
        step=step,
        encoding=encoding,
        data=data,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=seconds, nanosec=nanoseconds), frame_id="lunar-v0.2a-synthetic/test"),
    )


class VisionEncodingTests(unittest.TestCase):
    def test_rgb_png_preserves_pixels_and_ignores_row_padding(self) -> None:
        png, note = encode_image_png({"width": 1, "height": 1, "step": 4, "encoding": "rgb8", "data": b"\x01\x02\x03\xff"})
        width, height, bit_depth, pixels = decode_png(png)
        self.assertEqual((width, height, bit_depth), (1, 1, 8))
        self.assertEqual(pixels, b"\x00\x01\x02\x03")
        self.assertIsNone(note)

    def test_bgr_png_swizzles_to_display_rgb(self) -> None:
        png, _ = encode_image_png({"width": 1, "height": 1, "step": 3, "encoding": "bgr8", "data": b"\x03\x02\x01"})
        self.assertEqual(decode_png(png)[3], b"\x00\x01\x02\x03")

    def test_depth_preview_is_explicitly_normalized(self) -> None:
        values = (1000).to_bytes(2, "little") + (3000).to_bytes(2, "little")
        png, note = encode_image_png({"width": 2, "height": 1, "step": 4, "encoding": "16UC1", "data": values})
        self.assertEqual(decode_png(png)[:3], (2, 1, 8))
        self.assertEqual(decode_png(png)[3], b"\x00\x00\xff")
        self.assertIn("不可跨帧比较", note or "")

    def test_large_depth_preview_is_bounded_without_large_pixel_lists(self) -> None:
        # Capture only allocations made during conversion, not the caller-owned ROS payload.
        frame = {"width": 2048, "height": 2048, "step": 4096,
                 "encoding": "16UC1", "data": b"\xe8\x03" * (2048 * 2048)}
        tracemalloc.start()
        try:
            png, _ = encode_image_png(frame)
            _, peak = tracemalloc.get_traced_memory()
        finally:
            tracemalloc.stop()
        width, height, bit_depth, scanlines = decode_png(png)
        self.assertEqual((width, height, bit_depth), (1024, 1024, 8))
        self.assertEqual(scanlines, (b"\x00" + b"\x00" * 1024) * 1024)
        self.assertLess(peak, 16 * 1024 * 1024, "Depth conversion must not materialize pixel-sized Python integer lists")

    def test_big_endian_depth_preserves_normalized_values(self) -> None:
        png, _ = encode_image_png({"width": 2, "height": 1, "step": 4,
                                   "encoding": "mono16", "is_bigendian": True,
                                   "data": b"\x03\xe8\x0b\xb8"})
        self.assertEqual(decode_png(png)[3], b"\x00\x00\xff")

    def test_unsupported_encodings_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "Unsupported ROS image encoding"):
            encode_image_png({"width": 1, "height": 1, "step": 4, "encoding": "yuv422", "data": b"\x00" * 4})


class VisionObserverTests(unittest.TestCase):
    def setUp(self) -> None:
        self.observer = VisionObserver()
        self.observer.graph_available()

    def test_absent_source_is_unavailable_and_has_no_fake_metrics(self) -> None:
        reading = self.observer.snapshot()["streams"][0]
        self.assertEqual(reading["status"], "unavailable")
        self.assertEqual(reading["topic_type"], None)
        self.assertIsNone(reading["frame_rate_hz"])
        self.assertIsNone(reading["frame_age_ms"])
        self.assertIsNone(reading["source_stamp"])
        self.assertEqual(reading["publisher_count"], 0)

    def test_frame_is_reported_only_after_a_ros_image_arrives(self) -> None:
        self.observer.update_publishers("rgb", [IMAGE_TYPE], publisher_count=1)
        self.observer.receive("rgb", image_message("rgb8", b"\x10\x20\x30"))
        reading = self.observer.snapshot()["streams"][0]
        self.assertEqual(reading["status"], "streaming")
        self.assertEqual(reading["encoding"], "rgb8")
        self.assertEqual((reading["width"], reading["height"]), (1, 1))
        self.assertIsNotNone(reading["source_stamp"])
        self.assertEqual(reading["frame_id"], "lunar-v0.2a-synthetic/test")
        self.assertEqual(decode_png(self.observer.frame_png("rgb")[0] or b"")[3], b"\x00\x10\x20\x30")

    def test_stale_frame_is_not_returned_as_live_image(self) -> None:
        self.observer.update_publishers("rgb", [IMAGE_TYPE], publisher_count=1)
        self.observer.receive("rgb", image_message("rgb8", b"\x10\x20\x30"))
        with self.observer._lock:
            self.observer._frames["rgb"]["received_mono"] = time.monotonic() - 2.0
        self.assertEqual(self.observer.snapshot()["streams"][0]["status"], "stale")
        frame, reason, status = self.observer.frame_png("rgb")
        self.assertIsNone(frame)
        self.assertEqual(status, 503)
        self.assertIn("超时", reason or "")

    def test_source_loss_clears_cached_frame_metadata(self) -> None:
        self.observer.update_publishers("rgb", [IMAGE_TYPE], publisher_count=1)
        self.observer.receive("rgb", image_message("rgb8", b"\x10\x20\x30"))
        self.observer.update_publishers("rgb", [], publisher_count=0)
        reading = self.observer.snapshot()["streams"][0]
        self.assertEqual(reading["status"], "unavailable")
        self.assertIsNone(reading["width"])
        self.assertIsNone(reading["frame_rate_hz"])
        self.assertIsNone(reading["frame_age_ms"])
        self.assertIsNone(reading["source_stamp"])
        self.observer.update_publishers("rgb", [IMAGE_TYPE], publisher_count=1)
        self.assertEqual(self.observer.snapshot()["streams"][0]["status"], "waiting")
        self.assertEqual(self.observer.frame_png("rgb")[2], 503, "Reappearing publisher must not expose an earlier image")

    def test_unsupported_new_frame_does_not_fall_back_to_previous_image(self) -> None:
        self.observer.update_publishers("rgb", [IMAGE_TYPE], publisher_count=1)
        self.observer.receive("rgb", image_message("rgb8", b"\x10\x20\x30"))
        self.observer.receive("rgb", image_message("yuv422", b"\x00" * 4))
        self.assertEqual(self.observer.snapshot()["streams"][0]["status"], "unsupported")
        frame, _reason, status = self.observer.frame_png("rgb")
        self.assertIsNone(frame)
        self.assertEqual(status, 415)

    def test_unknown_stream_does_not_select_an_arbitrary_ros_topic(self) -> None:
        frame, reason, status = self.observer.frame_png("../../etc/passwd")
        self.assertIsNone(frame)
        self.assertEqual(status, 404)
        self.assertEqual(reason, "unknown visual source")


if __name__ == "__main__":
    unittest.main()
