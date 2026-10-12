#!/usr/bin/env python3
"""Exercise the default-URDF robot_state_publisher using synthetic JointState."""

import math
import sys
import time
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, ExtrapolationException, TransformException, TransformListener

JOINT_NAMES = [
    'J1_joint', 'J2_joint', 'J3_joint', 'J4_joint', 'J5_joint', 'J6_joint',
    'Claw_left_joint',
]


def as_time(stamp, clock_type):
    return Time(nanoseconds=stamp.sec * 1_000_000_000 + stamp.nanosec, clock_type=clock_type)


def translation(transform):
    value = transform.transform.translation
    return (value.x, value.y, value.z)


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


class TfIntegrationTest(Node):
    def __init__(self):
        super().__init__('p3b_urdf_tf_integration_test')
        self.publisher = self.create_publisher(
            JointState, '/p2_offline/joint_states',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def sample(self, values):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(JOINT_NAMES)
        msg.position = list(values) + [0.0]
        for _ in range(3):
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
        stamp = as_time(msg.header.stamp, self.get_clock().clock_type)
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            try:
                self.buffer.lookup_transform(
                    'base_link', 'camera_color_optical_frame', stamp,
                    timeout=Duration(seconds=0.05))
                return msg.header.stamp
            except TransformException:
                rclpy.spin_once(self, timeout_sec=0.02)
        raise AssertionError('robot_state_publisher gave no TF at this JointState timestamp')

    def lookup(self, target, source, stamp):
        return self.buffer.lookup_transform(
            target, source, as_time(stamp, self.get_clock().clock_type),
            timeout=Duration(seconds=0.2))


def main():
    rclpy.init()
    node = TfIntegrationTest()
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline and not node.buffer.can_transform(
                'camera_link', 'camera_color_optical_frame', Time(), timeout=Duration(seconds=0.05)):
            rclpy.spin_once(node, timeout_sec=0.05)
        assert node.buffer.can_transform(
            'camera_link', 'camera_color_optical_frame', Time(), timeout=Duration(seconds=0.1)), \
            'synthetic Orbbec optical-frame fixture unavailable'

        stamps = []
        target_positions = []
        probe = PoseStamped()
        probe.header.frame_id = 'camera_color_optical_frame'
        probe.pose.position.x = 0.12
        probe.pose.position.y = -0.04
        probe.pose.position.z = 0.65
        probe.pose.orientation.w = 1.0

        stamps.append(node.sample([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        camera_a = node.lookup('base_link', 'camera_link', stamps[-1])
        optical_a = node.lookup('base_link', 'camera_color_optical_frame', stamps[-1])
        target_positions.append(do_transform_pose_stamped(probe, optical_a).pose.position)

        stamps.append(node.sample([0.25, 0.30, -0.65, 0.0, 0.0, 0.0]))
        camera_b = node.lookup('base_link', 'camera_link', stamps[-1])
        optical_b = node.lookup('base_link', 'camera_color_optical_frame', stamps[-1])
        target_positions.append(do_transform_pose_stamped(probe, optical_b).pose.position)
        movement_123 = distance(translation(camera_a), translation(camera_b))
        target_delta_123 = distance(
            (target_positions[0].x, target_positions[0].y, target_positions[0].z),
            (target_positions[1].x, target_positions[1].y, target_positions[1].z))
        assert movement_123 > 0.01, f'J1-J3 camera movement too small: {movement_123:.6f} m'
        assert target_delta_123 > 0.01, f'transformed target movement too small: {target_delta_123:.6f} m'
        exact_a_after_b = node.lookup('base_link', 'camera_color_optical_frame', stamps[0])
        assert distance(translation(exact_a_after_b), translation(camera_a)) < 1e-5
        assert distance(translation(exact_a_after_b), translation(camera_b)) > 0.01, \
            'old timestamp returned the latest camera pose instead of its matching pose'

        stamps.append(node.sample([0.0, 0.0, 0.0, 0.50, -0.35, 0.65]))
        camera_c = node.lookup('base_link', 'camera_link', stamps[-1])
        movement_456 = distance(translation(camera_a), translation(camera_c))
        assert movement_456 < 1e-5, f'J4-J6 moved J3-mounted camera: {movement_456:.9f} m'

        missing = Time(
            nanoseconds=(stamps[-1].sec + 2) * 1_000_000_000 + stamps[-1].nanosec,
            clock_type=node.get_clock().clock_type)
        try:
            node.buffer.lookup_transform(
                'base_link', 'camera_color_optical_frame', missing,
                timeout=Duration(seconds=0.1))
            raise AssertionError('missing timestamp unexpectedly returned a transform')
        except ExtrapolationException:
            pass

        camera_optical = node.buffer.lookup_transform(
            'camera_link', 'camera_color_optical_frame', Time(),
            timeout=Duration(seconds=0.2))
        q = camera_optical.transform.rotation
        expected = (-0.5, 0.5, -0.5, 0.5)
        qdot = abs(q.x * expected[0] + q.y * expected[1] + q.z * expected[2] + q.w * expected[3])
        assert abs(1.0 - qdot) < 1e-6, f'Orbbec optical convention mismatch: qdot={qdot}'

        print('P3B_TF_TEST=PASS')
        print('chain: base_link -> J1_link -> J2_link -> J3_link -> camera_link -> camera_color_optical_frame')
        print(f'J1-J3 camera translation change: {movement_123:.6f} m; transformed target change: {target_delta_123:.6f} m')
        print(f'J4-J6 camera translation change: {movement_456:.9f} m (expected 0)')
        print('timestamp samples: ' + ', '.join(f'{s.sec}.{s.nanosec:09d}' for s in stamps))
        print('missing timestamp: rejected; no latest-TF fallback')
        print(f'camera_link -> optical translation: {translation(camera_optical)} m (synthetic fixture)')
        return 0
    except Exception as exc:
        print(f'P3B_TF_TEST=FAIL: {exc}', file=sys.stderr)
        raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
