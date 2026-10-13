#!/usr/bin/env python3
"""Exercise DetectionArray + organized PointCloud2 + CameraInfo -> PoseStamped.

All three inputs are generated offline from the packaged PCD template. This is
an interface/algorithm smoke test and is not a real-object accuracy result.
"""

from __future__ import annotations

import math
import os
import random
import subprocess
import sys
import tempfile
import time
from pathlib import Path

from offline_test_data import organized_model_cloud


def main() -> int:
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--model-pcd", required=True)
    parser.add_argument("--params", required=True, help="pose_estimator params.yaml from the tested source")
    parser.add_argument("--timeout", type=float, default=90.0)
    args = parser.parse_args()

    os.environ["ROS_DOMAIN_ID"] = str(random.randint(16, 90))
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, PointCloud2, PointField
    from geometry_msgs.msg import PoseStamped
    from yolo_node.msg import Detection, DetectionArray
    from ament_index_python.packages import get_package_prefix
    from pose_estimator.pose_estimation_node import (
        _InvalidPointCloudError, pc2_roi_numpy, pc2_to_numpy)

    model_path = Path(args.model_pcd).expanduser().resolve()
    xyz, fx, fy, cx, cy, bbox, expected_center, occupied = organized_model_cloud(str(model_path))
    height, width = xyz.shape[:2]
    frame_id = "synthetic_camera_color_optical_frame"
    suffix = f"/p9i_{os.getpid()}"
    topics = {
        "cloud": suffix + "/points",
        "info": suffix + "/camera_info",
        "detections": suffix + "/detections",
        "pose": suffix + "/pose",
    }
    child_cmd = [
        str(Path(get_package_prefix("pose_estimator")) / "lib/pose_estimator/pose_estimation_node"), "--ros-args",
        "--params-file", str(Path(args.params).expanduser().resolve()),
        "-p", f"model_path:={model_path}",
        "-p", f"pointcloud_topic:={topics['cloud']}",
        "-p", f"camera_info_topic:={topics['info']}",
        "-p", f"detection_topic:={topics['detections']}",
        "-p", f"object_pose_topic:={topics['pose']}",
        "-p", "target_class:=blue_block",
    ]

    rclpy.init()
    node = rclpy.create_node("p9i_offline_pose_test")
    cloud_pub = node.create_publisher(PointCloud2, topics["cloud"], qos_profile_sensor_data)
    info_pub = node.create_publisher(CameraInfo, topics["info"], 10)
    det_pub = node.create_publisher(DetectionArray, topics["detections"], 10)
    poses: list[PoseStamped] = []
    node.create_subscription(PoseStamped, topics["pose"], poses.append, 10)
    log = tempfile.TemporaryFile(mode="w+t")
    child = subprocess.Popen(child_cmd, stdout=log, stderr=subprocess.STDOUT, text=True)

    def make_header(stamp):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = stamp
        h.frame_id = frame_id
        return h

    def make_camera_info(stamp):
        msg = CameraInfo()
        msg.header = make_header(stamp)
        msg.width = width
        msg.height = height
        msg.distortion_model = "plumb_bob"
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return msg

    def make_detection(stamp, present=True):
        msg = DetectionArray()
        msg.header = make_header(stamp)
        if present:
            x1, y1, x2, y2 = bbox
            det = Detection()
            det.x, det.y = float(x1), float(y1)
            det.width, det.height = float(x2 - x1), float(y2 - y1)
            det.confidence = 0.99
            det.class_id = 0
            det.class_name = "blue_block"
            msg.detections.append(det)
        return msg

    def make_cloud(stamp, malformed=False):
        msg = PointCloud2()
        msg.header = make_header(stamp)
        msg.height = height if not malformed else 1
        msg.width = width if not malformed else 1
        msg.is_bigendian = False
        msg.is_dense = not malformed
        msg.point_step = 12
        msg.row_step = msg.width * msg.point_step
        msg.fields = []
        for name, offset in (("x", 0), ("y", 4), ("z", 8)):
            field = PointField()
            field.name, field.offset = name, offset
            field.datatype, field.count = PointField.FLOAT32, 1
            msg.fields.append(field)
        if malformed:
            msg.fields = msg.fields[:1]  # intentionally incomplete; should be rejected without killing the node
            msg.data = np.zeros((1, 3), dtype="<f4").tobytes()
        else:
            msg.data = np.ascontiguousarray(xyz, dtype="<f4").tobytes()
        return msg

    def verify_pointcloud_decoders():
        padded = PointCloud2()
        padded.width, padded.height = 2, 2
        padded.is_bigendian = False
        padded.point_step, padded.row_step = 16, 36
        padded.fields = []
        for name, offset in (("x", 0), ("y", 4), ("z", 8)):
            field = PointField()
            field.name, field.offset = name, offset
            field.datatype, field.count = PointField.FLOAT32, 1
            padded.fields.append(field)
        expected = np.arange(1, 13, dtype="<f4").reshape(4, 3)
        row_bytes = []
        for row in expected.reshape(2, 2, 3):
            point_bytes = b"".join(point.tobytes() + b"RSVD" for point in row)
            row_bytes.append(point_bytes + b"PAD!")
        padded.data = b"".join(row_bytes)
        np.testing.assert_allclose(pc2_to_numpy(padded), expected)
        np.testing.assert_allclose(pc2_roi_numpy(padded, 0, 2, 0, 2), expected)

        big_endian = PointCloud2()
        big_endian.width, big_endian.height = 1, 1
        big_endian.is_bigendian = True
        big_endian.point_step = big_endian.row_step = 12
        big_endian.fields = padded.fields[:3]
        big_endian.data = np.array([[-0.25, 0.5, 0.75]], dtype=">f4").tobytes()
        np.testing.assert_allclose(pc2_to_numpy(big_endian), [[-0.25, 0.5, 0.75]])

        malformed = make_cloud(node.get_clock().now().to_msg(), malformed=True)
        try:
            pc2_to_numpy(malformed)
        except _InvalidPointCloudError as exc:
            if "missing required XYZ fields: y, z" not in str(exc):
                raise
        else:
            raise AssertionError("PointCloud2 decoder accepted a message missing y/z")

    def spin_for(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            if child.poll() is not None:
                raise RuntimeError("pose_estimation_node exited during offline test")
            rclpy.spin_once(node, timeout_sec=0.05)

    try:
        verify_pointcloud_decoders()
        deadline = time.monotonic() + 20.0
        while (cloud_pub.get_subscription_count() == 0 or info_pub.get_subscription_count() == 0
               or det_pub.get_subscription_count() == 0) and time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError("pose_estimation_node exited before subscribing")
            rclpy.spin_once(node, timeout_sec=0.1)
        if min(cloud_pub.get_subscription_count(), info_pub.get_subscription_count(), det_pub.get_subscription_count()) == 0:
            raise TimeoutError("pose_estimation_node did not subscribe to all isolated test topics")

        stamp = node.get_clock().now().to_msg()
        info_pub.publish(make_camera_info(stamp))
        det_pub.publish(make_detection(stamp, present=False))
        cloud_pub.publish(make_cloud(stamp))
        spin_for(0.5)
        if poses:
            raise AssertionError("pose was published without a detection ROI")

        stamp = node.get_clock().now().to_msg()
        info_pub.publish(make_camera_info(stamp))
        det_pub.publish(make_detection(stamp, present=True))
        cloud_pub.publish(make_cloud(stamp, malformed=True))
        spin_for(0.5)
        if poses:
            raise AssertionError("pose was published for a malformed cloud missing y/z fields")

        stamp = node.get_clock().now().to_msg()
        info_pub.publish(make_camera_info(stamp))
        det_pub.publish(make_detection(stamp, present=True))
        sent_mono = time.monotonic()
        valid_info = make_camera_info(stamp)
        valid_detection = make_detection(stamp, present=True)
        valid_cloud = make_cloud(stamp)
        deadline = time.monotonic() + args.timeout
        while not poses and time.monotonic() < deadline:
            if child.poll() is not None:
                raise RuntimeError("pose_estimation_node exited during synthetic registration")
            # Detection and PointCloud2 use separate DDS subscriptions. Keep
            # publishing the same synthetic sample until both have crossed
            # the process boundary; a one-shot pair can arrive out of order.
            info_pub.publish(valid_info)
            det_pub.publish(valid_detection)
            cloud_pub.publish(valid_cloud)
            rclpy.spin_once(node, timeout_sec=0.1)
        if not poses:
            raise TimeoutError("no PoseStamped received for the perfectly aligned synthetic PCD cloud")

        pose = poses[0]
        p, q = pose.pose.position, pose.pose.orientation
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if not all(math.isfinite(float(x)) for x in values):
            raise AssertionError(f"non-finite pose: {values}")
        qnorm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        if abs(qnorm - 1.0) > 1e-3:
            raise AssertionError(f"non-unit quaternion: norm={qnorm}")
        if pose.header.frame_id != frame_id:
            raise AssertionError(f"pose frame_id={pose.header.frame_id!r}, expected {frame_id!r}")
        if (pose.header.stamp.sec, pose.header.stamp.nanosec) != (stamp.sec, stamp.nanosec):
            raise AssertionError("PoseStamped did not preserve the synthetic cloud timestamp")
        latency_ms = (time.monotonic() - sent_mono) * 1000.0
        print(f"synthetic_points={occupied} image={width}x{height} bbox={bbox} frame={frame_id}")
        print(f"pose_xyz=({p.x:.5f},{p.y:.5f},{p.z:.5f}) quaternion_norm={qnorm:.6f} latency_ms={latency_ms:.1f}")
        print(f"input_model_center={expected_center.tolist()} (synthetic, not an accuracy metric)")
        child.terminate()
        child.wait(timeout=5.0)
        log.seek(0)
        log_text = log.read()
        if "拒绝无效 PointCloud2，跳过本帧" not in log_text:
            raise AssertionError("malformed PointCloud2 was not explicitly rejected by the pose node")
        if "Traceback (most recent call last)" in log_text:
            raise AssertionError("offline pose node emitted an uncaught traceback")
        print("negative_cases=empty DetectionArray ignored; malformed PointCloud2 rejected; node remained alive")
        timing = [line.strip() for line in log_text.splitlines()
                  if "[POSE TIMING]" in line or "[GLOBAL]" in line or "[TRACK]" in line]
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
        if not poses:
            log.seek(0)
            tail = log.read().splitlines()[-100:]
            if tail:
                print("pose_node_log_tail:\n" + "\n".join(tail), file=sys.stderr)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        log.close()


if __name__ == "__main__":
    raise SystemExit(main())
