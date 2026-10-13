#!/usr/bin/env python3
"""Start the installed YOLO ROS node and verify its image/message round trip."""

from __future__ import annotations

import argparse
import math
import os
import random
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from offline_test_data import synthetic_blue_block_bgr


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True, help="External OpenVINO model bundle directory")
    parser.add_argument("--timeout", type=float, default=35.0)
    args = parser.parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(random.randint(16, 90))
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import numpy as np
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image
    from yolo_node.msg import DetectionArray
    from ament_index_python.packages import get_package_prefix

    image = synthetic_blue_block_bgr()
    height, width = image.shape[:2]
    topic_suffix = f"/p9i_{os.getpid()}"
    image_topic = topic_suffix + "/image"
    detections_topic = topic_suffix + "/detections"
    annotated_topic = topic_suffix + "/annotated"
    env = os.environ.copy()
    model_tmp = tempfile.TemporaryDirectory(prefix="lunar-p9i-appledouble-model-")
    model_bundle = Path(model_tmp.name) / "blue_block"
    model_bundle.mkdir()
    model_root = Path(args.model).expanduser().resolve()
    for model_file in ("metadata.yaml", "yoloe-26l-seg.xml", "yoloe-26l-seg.bin"):
        (model_bundle / model_file).symlink_to(model_root / model_file)
    # A common macOS archive sidecar must never be selected as the IR XML.
    (model_bundle / "._decoy.xml").write_text("not an OpenVINO model", encoding="utf-8")
    (model_bundle / "._decoy.bin").write_bytes(b"not model weights")
    node_cmd = [
        str(Path(get_package_prefix("yolo_node")) / "lib/yolo_node/yolo_detector.py"), "--ros-args",
        "-p", f"model_path:={model_bundle}",
        "-p", f"image_topic:={image_topic}",
        "-p", f"detection_topic:={detections_topic}",
        "-p", f"annotated_image_topic:={annotated_topic}",
        "-p", "conf_threshold:=0.0005",
        "-p", "publish_conf_threshold:=0.0005",
        "-p", "publish_hold_seconds:=0.0",
        "-p", "timing_log_enable:=true",
        "-p", "timing_log_throttle_sec:=0.0",
    ]

    rclpy.init()
    node = rclpy.create_node("p9i_offline_yolo_test")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=1)
    image_pub = node.create_publisher(Image, image_topic, qos)
    results: dict[str, object] = {}
    node.create_subscription(DetectionArray, detections_topic, lambda m: results.setdefault("detections", m), 10)
    node.create_subscription(Image, annotated_topic, lambda m: results.setdefault("annotated", m), 10)
    log = tempfile.TemporaryFile(mode="w+t")
    child = subprocess.Popen(node_cmd, env=env, stdout=log, stderr=subprocess.STDOUT, text=True)
    try:
        deadline = time.monotonic() + args.timeout
        while image_pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError("yolo_detector.py exited before subscribing to the synthetic image")
            rclpy.spin_once(node, timeout_sec=0.1)
        if image_pub.get_subscription_count() == 0:
            raise TimeoutError("YOLO ROS node did not subscribe to the isolated test topic")

        msg = Image()
        now = node.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = "synthetic_rgb_optical_frame"
        msg.height = height
        msg.width = width
        msg.encoding = "bgr8"
        msg.is_bigendian = False
        msg.step = width * 3
        msg.data = np.ascontiguousarray(image).tobytes()
        sent_mono = time.monotonic()

        while time.monotonic() < deadline and not {"detections", "annotated"}.issubset(results):
            if child.poll() is not None:
                raise RuntimeError("yolo_detector.py exited during synthetic inference")
            # Re-publish until the first result so a discovery/first-sample race
            # cannot turn a smoke test into a flaky one-frame transport test.
            image_pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.1)
        if not {"detections", "annotated"}.issubset(results):
            raise TimeoutError("did not receive both DetectionArray and annotated Image")

        det = results["detections"]
        annotated = results["annotated"]
        assert isinstance(det, DetectionArray) and isinstance(annotated, Image)
        if (det.header.stamp.sec, det.header.stamp.nanosec, det.header.frame_id) != (
                msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id):
            raise AssertionError("DetectionArray did not preserve input stamp/frame_id")
        if (annotated.header.stamp.sec, annotated.header.stamp.nanosec, annotated.header.frame_id) != (
                msg.header.stamp.sec, msg.header.stamp.nanosec, msg.header.frame_id):
            raise AssertionError("annotated image did not preserve input stamp/frame_id")
        if annotated.width != width or annotated.height != height or annotated.encoding != "bgr8":
            raise AssertionError("annotated image shape/encoding mismatch")
        if not det.detections:
            raise AssertionError("synthetic model test produced an empty DetectionArray")
        for item in det.detections:
            values = (item.x, item.y, item.width, item.height, item.confidence)
            if not all(math.isfinite(float(v)) for v in values):
                raise AssertionError(f"non-finite detection fields: {values}")
            if item.width <= 0 or item.height <= 0 or not (0.0 <= item.confidence <= 1.0):
                raise AssertionError(f"invalid detection bounds/confidence: {values}")
            if item.x < -2 or item.y < -2 or item.x + item.width > width + 2 or item.y + item.height > height + 2:
                raise AssertionError(f"detection extends outside input image: {values}")
            if item.class_id not in (0, 1):
                raise AssertionError(f"unexpected synthetic model class id: {item.class_id}")

        wall_ms = (time.monotonic() - sent_mono) * 1000.0
        print(f"synthetic_image={width}x{height} frame={msg.header.frame_id}")
        print(f"detections={len(det.detections)} wall_latency_ms={wall_ms:.1f}")
        print("detection_fields=" + repr([
            {"class_id": d.class_id, "class_name": d.class_name, "confidence": d.confidence,
             "xywh": [d.x, d.y, d.width, d.height]}
            for d in det.detections]))
        print(f"annotated={annotated.width}x{annotated.height} {annotated.encoding} stamp/frame preserved")
        child.terminate()
        child.wait(timeout=5.0)
        log.seek(0)
        timing = [line.strip() for line in log if "[YOLO TIMING]" in line]
        if timing:
            print("node_timing=" + timing[-1])
        return 0
    finally:
        if child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        if not {"detections", "annotated"}.issubset(results):
            log.seek(0)
            tail = log.read().splitlines()[-80:]
            if tail:
                print("yolo_node_log_tail:\n" + "\n".join(tail), file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()
        model_tmp.cleanup()


if __name__ == "__main__":
    raise SystemExit(main())
