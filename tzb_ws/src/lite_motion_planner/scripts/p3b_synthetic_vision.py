#!/usr/bin/env python3
"""Publish a synthetic camera-frame pose equivalent to a fixed base_link target."""

import math
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class SyntheticVision(Node):
    def __init__(self):
        super().__init__('p3b_synthetic_vision')
        self.declare_parameter('target_topic', '/p2_offline/pose_estimator/object_pose')
        self.declare_parameter('target_xyz_base', [-0.2616858316, -0.1753690719, 0.3573509015])
        self.declare_parameter('target_rpy_base', [-2.73, 0.0, 2.5])
        self._target_topic = str(self.get_parameter('target_topic').value)
        self._xyz = list(self.get_parameter('target_xyz_base').value)
        self._rpy = list(self.get_parameter('target_rpy_base').value)
        if len(self._xyz) != 3 or len(self._rpy) != 3:
            raise ValueError('target xyz and rpy must each have three values')
        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._publisher = self.create_publisher(PoseStamped, self._target_topic, 10)
        self._timer = self.create_timer(0.05, self._publish_target)
        self._last_log = self.get_clock().now()
        self._published = 0

    def _publish_target(self):
        try:
            tf = self._buffer.lookup_transform(
                'camera_color_optical_frame', 'base_link', Time(),
                timeout=Duration(seconds=0.05))
            base_pose = PoseStamped()
            base_pose.header.frame_id = 'base_link'
            base_pose.pose.position.x, base_pose.pose.position.y, base_pose.pose.position.z = self._xyz
            qx, qy, qz, qw = quaternion_from_rpy(*self._rpy)
            base_pose.pose.orientation.x = qx
            base_pose.pose.orientation.y = qy
            base_pose.pose.orientation.z = qz
            base_pose.pose.orientation.w = qw
            camera_pose = do_transform_pose_stamped(base_pose, tf)
            camera_pose.header.frame_id = 'camera_color_optical_frame'
            camera_pose.header.stamp = tf.header.stamp
            self._publisher.publish(camera_pose)
            self._published += 1
            now = self.get_clock().now()
            if self._published == 1 or (now - self._last_log).nanoseconds > 2_000_000_000:
                self.get_logger().info(
                    'Publishing synthetic optical target derived from fixed base_link target; '
                    f'count={self._published}, stamp={camera_pose.header.stamp.sec}.'
                    f'{camera_pose.header.stamp.nanosec:09d}')
                self._last_log = now
        except TransformException as exc:
            now = self.get_clock().now()
            if (now - self._last_log).nanoseconds > 2_000_000_000:
                self.get_logger().warning(f'Waiting for URDF/RSP TF: {exc}')
                self._last_log = now


def main():
    rclpy.init()
    node = SyntheticVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
