#!/usr/bin/env python3
"""Publish bounded, clearly synthetic RGB-D inputs for the P6 integration launch."""

from __future__ import annotations

import argparse
import time

import numpy as np

from offline_test_data import organized_model_cloud, synthetic_blue_block_bgr


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', choices=('normal', 'no_target', 'low_confidence', 'timeout', 'recovery'), default='normal')
    parser.add_argument('--model-pcd', required=True)
    parser.add_argument('--rate-hz', type=float, default=1.0)
    args = parser.parse_args()
    if not 0.2 <= args.rate_hz <= 2.0:
        parser.error('--rate-hz must be between 0.2 and 2.0 to bound synthetic publication load')

    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
    from std_msgs.msg import String

    xyz, fx, fy, cx, cy, bbox, _expected_center, occupied_points = organized_model_cloud(
        args.model_pcd, width=640, height=640)
    height, width = xyz.shape[:2]
    frame = 'synthetic_camera_color_optical_frame'
    source_image = synthetic_blue_block_bgr()
    image = np.full_like(source_image, 235)
    x1, y1, x2, y2 = bbox
    source = source_image[225:506, 205:439]
    import cv2
    image[y1:y2, x1:x2] = cv2.resize(source, (x2 - x1, y2 - y1))

    class InputPublisher(Node):
        def __init__(self):
            super().__init__('p6_synthetic_rgbd_inputs')
            self.image_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
            self.info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
            self.cloud_pub = self.create_publisher(PointCloud2, '/camera/depth/color/points', 10)
            self.started = time.monotonic()
            self.target_detection_started = None
            self.frame_count = 0
            if args.scenario == 'recovery':
                self.create_subscription(String, '/p6/mission_state', self.on_mission_state, 10)
            self.create_timer(1.0 / args.rate_hz, self.publish_frame)

        def on_mission_state(self, message):
            if message.data == 'DETECTING_TARGET' and self.target_detection_started is None:
                self.target_detection_started = time.monotonic()

        def _header(self, message, stamp):
            message.header.stamp = stamp
            message.header.frame_id = frame

        def publish_frame(self):
            self.frame_count += 1
            stamp = self.get_clock().now().to_msg()
            elapsed = time.monotonic() - self.started
            has_target = args.scenario != 'no_target'

            image_msg = Image()
            self._header(image_msg, stamp)
            image_msg.width, image_msg.height = width, height
            image_msg.encoding = 'bgr8'
            image_msg.is_bigendian = False
            image_msg.step = width * 3
            selected_image = image if has_target else np.full_like(image, 235)
            image_msg.data = np.ascontiguousarray(selected_image).tobytes()
            self.image_pub.publish(image_msg)

            info = CameraInfo()
            self._header(info, stamp)
            info.width, info.height = width, height
            info.distortion_model = 'plumb_bob'
            info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
            info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.info_pub.publish(info)

            cloud_available = args.scenario not in ('timeout',)
            if args.scenario == 'recovery':
                cloud_available = (
                    self.target_detection_started is not None
                    and time.monotonic() - self.target_detection_started >= 4.0
                )
            if cloud_available:
                cloud = PointCloud2()
                self._header(cloud, stamp)
                cloud.height, cloud.width = height, width
                cloud.is_bigendian = False
                cloud.is_dense = False
                cloud.point_step = 12
                cloud.row_step = width * cloud.point_step
                cloud.fields = []
                for name, offset in (('x', 0), ('y', 4), ('z', 8)):
                    field = PointField()
                    field.name, field.offset = name, offset
                    field.datatype, field.count = PointField.FLOAT32, 1
                    cloud.fields.append(field)
                cloud.data = np.ascontiguousarray(xyz, dtype='<f4').tobytes()
                self.cloud_pub.publish(cloud)

            if self.frame_count == 1:
                self.get_logger().info(
                    'SYNTHETIC INPUT ONLY: RGB-D fixture points=%d bbox=%s scenario=%s' %
                    (occupied_points, bbox, args.scenario))

    rclpy.init()
    node = InputPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
