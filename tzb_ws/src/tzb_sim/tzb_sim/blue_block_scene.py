#!/usr/bin/env python3
"""Synthetic RGB-D scene of a blue cube on a table.
蓝色方块的仿真场景
Publishes a rendered RGB image, matching camera intrinsics, an organized
point cloud and the camera TF, so the YOLO + pose-estimation + arm pipeline
can run without an Orbbec camera or a physical robot.
可发布渲染的RGB图像、匹配的相机内参、有组织的点云和相机TF，因此YOLO + 姿态估计 + 机械臂管道
The rendered image and the ray-cast point cloud share the same camera model:
    camera at (0, 0, 0.60) m in base_link, optical +X forward, 42 deg down,
    fx = fy = 800 px, 640 x 480, cube 50 mm at (0.40, 0.12, 0.025).
"""

import math
import os
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation as Rotation
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker


def _as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _quat_from_matrix(matrix: np.ndarray) -> tuple[float, float, float, float]:
    x, y, z, w = Rotation.from_matrix(matrix).as_quat()
    return float(x), float(y), float(z), float(w)


def _camera_rotation(pitch_rad: float) -> np.ndarray:
    """R_base_cam for an optical frame looking along +X and tilted down."""
    x_axis = np.array([0.0, -1.0, 0.0])
    z_axis = np.array([math.cos(pitch_rad), 0.0, -math.sin(pitch_rad)])
    y_axis = np.cross(z_axis, x_axis)
    return np.column_stack((x_axis, y_axis, z_axis))


def _rotation_z(angle: float) -> np.ndarray:
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class BlueBlockScene(Node):
    def __init__(self) -> None:
        super().__init__('blue_block_scene')

        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('frame_id', 'sim_camera_link')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('pointcloud_topic', '/camera/depth/color/points')
        self.declare_parameter('image_path', '')
        self.declare_parameter('publish_image', True)
        self.declare_parameter('publish_gt_detections', False)
        self.declare_parameter('detection_topic', '/yolo/detections')
        self.declare_parameter('gt_pose_topic', '/sim/object_pose_gt')
        self.declare_parameter('marker_topic', '/sim/object_marker')

        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fx', 800.0)
        self.declare_parameter('fy', 800.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)

        self.declare_parameter('camera_x', 0.0)
        self.declare_parameter('camera_y', 0.0)
        self.declare_parameter('camera_z', 0.60)
        self.declare_parameter('camera_pitch_deg', 52.13)
        self.declare_parameter('camera_yaw_deg', 26.57)

        self.declare_parameter('object_x', 0.40)
        self.declare_parameter('object_y', 0.20)
        self.declare_parameter('object_z', 0.025)
        self.declare_parameter('object_size_x', 0.05)
        self.declare_parameter('object_size_y', 0.05)
        self.declare_parameter('object_size_z', 0.05)
        self.declare_parameter('object_yaw', 0.0)
        self.declare_parameter('noise_std', 0.0005)
        self.declare_parameter('timing_log', False)
        self.declare_parameter('publish_camera_tf', True)

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fx = float(self.get_parameter('fx').value)
        self.fy = float(self.get_parameter('fy').value)
        self.cx = float(self.get_parameter('cx').value)
        self.cy = float(self.get_parameter('cy').value)

        self.frame_id = str(self.get_parameter('frame_id').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.camera_pos = np.array([
            float(self.get_parameter('camera_x').value),
            float(self.get_parameter('camera_y').value),
            float(self.get_parameter('camera_z').value),
        ])
        self.camera_pitch = math.radians(
            float(self.get_parameter('camera_pitch_deg').value))
        self.camera_yaw = math.radians(
            float(self.get_parameter('camera_yaw_deg').value))
        self.R_base_cam = (
            _rotation_z(self.camera_yaw) @ _camera_rotation(self.camera_pitch))

        self.object_center = np.array([
            float(self.get_parameter('object_x').value),
            float(self.get_parameter('object_y').value),
            float(self.get_parameter('object_z').value),
        ])
        self.object_half = 0.5 * np.array([
            float(self.get_parameter('object_size_x').value),
            float(self.get_parameter('object_size_y').value),
            float(self.get_parameter('object_size_z').value),
        ])
        self.object_yaw = float(self.get_parameter('object_yaw').value)
        self.noise_std = max(0.0, float(self.get_parameter('noise_std').value))

        self.bridge = CvBridge()
        image_path = str(self.get_parameter('image_path').value).strip()
        if not image_path:
            image_path = os.path.join(
                get_package_share_directory('tzb_sim'),
                'config', 'blue_block_scene.png')
        self.image = self._load_image(image_path)

        self.image_pub = self.create_publisher(
            Image, str(self.get_parameter('image_topic').value),
            qos_profile_sensor_data)
        self.caminfo_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter('camera_info_topic').value), 10)
        self.cloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter('pointcloud_topic').value),
            qos_profile_sensor_data)
        self.gt_pose_pub = self.create_publisher(
            PoseStamped, str(self.get_parameter('gt_pose_topic').value), 10)
        self.marker_pub = self.create_publisher(
            Marker, str(self.get_parameter('marker_topic').value), 10)

        self.publish_gt_detections = _as_bool(
            self.get_parameter('publish_gt_detections').value)
        self.detection_pub = None
        if self.publish_gt_detections:
            from yolo_node.msg import Detection, DetectionArray
            self._Detection = Detection
            self._DetectionArray = DetectionArray
            self.detection_pub = self.create_publisher(
                DetectionArray,
                str(self.get_parameter('detection_topic').value), 3)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf = None
        if _as_bool(self.get_parameter('publish_camera_tf').value):
            # When the robot URDF already contains this frame (simulation
            # variant), let robot_state_publisher own the transform.
            self.static_tf = StaticTransformBroadcaster(self)
            self._publish_camera_tf()

        self._ray_dirs_cam = self._make_ray_dirs()
        self._cloud_points = self._ray_box_hit()
        self._camera_info = self._make_camera_info()
        self._bbox = self._projected_bbox()
        # The scene is static, so every message body is built exactly once.
        # Re-assigning multi-megabyte arrays every tick costs hundreds of ms
        # because rclpy converts Python sequences element by element.
        self._cloud_msg = self._make_cloud_msg(self.get_clock().now().to_msg())
        self._image_msg = self.bridge.cv2_to_imgmsg(self.image, encoding='bgr8')
        self._image_msg.header.frame_id = self.frame_id

        rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'Synthetic blue block scene: camera=({self.camera_pos.tolist()}) '
            f'object=({self.object_center.tolist()}) '
            f'size={(self.object_half * 2).tolist()} '
            f'image={self.width}x{self.height} rate={rate:.1f}Hz')

    def _load_image(self, path: str) -> np.ndarray:
        image = cv2.imread(path)
        if image is None:
            self.get_logger().warning(
                f'Scene image not found: {path}; publishing a procedural fallback')
            image = np.full((self.height, self.width, 3), 220, np.uint8)
            cv2.rectangle(image, (self.width // 3, self.height // 3),
                          (2 * self.width // 3, 2 * self.height // 3),
                          (200, 90, 20), -1)
        if image.shape[0] != self.height or image.shape[1] != self.width:
            image = cv2.resize(image, (self.width, self.height))
        return image

    def _make_camera_info(self) -> CameraInfo:
        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [self.fx, 0.0, self.cx,
                  0.0, self.fy, self.cy,
                  0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [self.fx, 0.0, self.cx, 0.0,
                  0.0, self.fy, self.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def _make_ray_dirs(self) -> np.ndarray:
        u, v = np.meshgrid(np.arange(self.width), np.arange(self.height))
        dirs = np.stack((
            (u - self.cx) / self.fx,
            (v - self.cy) / self.fy,
            np.ones_like(u, dtype=np.float64),
        ), axis=-1)
        return dirs.reshape(-1, 3)

    def _projected_bbox(self) -> tuple[float, float, float, float]:
        rot_z = _rotation_z(self.object_yaw)
        corners = []
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    p = self.object_center + rot_z @ (
                        self.object_half * np.array([sx, sy, sz]))
                    p_cam = self.R_base_cam.T @ (p - self.camera_pos)
                    if p_cam[2] <= 0.0:
                        continue
                    u = self.fx * p_cam[0] / p_cam[2] + self.cx
                    v = self.fy * p_cam[1] / p_cam[2] + self.cy
                    corners.append((u, v))
        if not corners:
            return (0.0, 0.0, float(self.width), float(self.height))
        arr = np.asarray(corners)
        return (
            float(arr[:, 0].min()), float(arr[:, 1].min()),
            float(arr[:, 0].max()), float(arr[:, 1].max()))

    def _ray_box_hit(self) -> np.ndarray:
        """Hit point in the camera frame for every pixel (NaN on miss)."""
        dirs_base = self._ray_dirs_cam @ self.R_base_cam.T
        rot_inv = _rotation_z(-self.object_yaw)
        origin_box = rot_inv @ (self.camera_pos - self.object_center)
        dirs_box = dirs_base @ rot_inv.T

        t_enter = np.full(len(dirs_box), -np.inf)
        t_exit = np.full(len(dirs_box), np.inf)
        with np.errstate(divide='ignore', invalid='ignore'):
            for axis in range(3):
                d = dirs_box[:, axis]
                o = origin_box[axis]
                h = self.object_half[axis]
                t1 = (-h - o) / d
                t2 = (h - o) / d
                t_enter = np.maximum(t_enter, np.minimum(t1, t2))
                t_exit = np.minimum(t_exit, np.maximum(t1, t2))

        hit_mask = (t_exit >= t_enter) & (t_enter > 0.0)
        hit = np.full((len(dirs_base), 3), np.nan, dtype=np.float64)
        if not np.any(hit_mask):
            return hit.astype(np.float32)

        t = t_enter[hit_mask]
        hit_box = origin_box + dirs_box[hit_mask] * t[:, None]
        rot_z = _rotation_z(self.object_yaw)
        hit_base = hit_box @ rot_z.T + self.object_center
        hit_cam = (hit_base - self.camera_pos) @ self.R_base_cam
        if self.noise_std > 0.0:
            hit_cam += np.random.normal(0.0, self.noise_std, hit_cam.shape)
        hit[hit_mask] = hit_cam
        return hit.astype(np.float32)

    def _publish_camera_tf(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.frame_id
        qx, qy, qz, qw = _quat_from_matrix(self.R_base_cam)
        transform.transform.translation.x = float(self.camera_pos[0])
        transform.transform.translation.y = float(self.camera_pos[1])
        transform.transform.translation.z = float(self.camera_pos[2])
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.static_tf.sendTransform(transform)

    def _make_cloud_msg(self, stamp) -> PointCloud2:
        points = self._cloud_points.reshape(self.height, self.width, 3)
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * self.width
        msg.is_dense = False
        msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        return msg

    def _publish_gt(self, stamp) -> None:
        rot_z = _rotation_z(self.object_yaw)
        qx, qy, qz, qw = _quat_from_matrix(rot_z)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.base_frame
        pose.pose.position.x = float(self.object_center[0])
        pose.pose.position.y = float(self.object_center[1])
        pose.pose.position.z = float(self.object_center[2])
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.gt_pose_pub.publish(pose)

        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.base_frame
        marker.ns = 'sim_target'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = float(self.object_half[0] * 2.0)
        marker.scale.y = float(self.object_half[1] * 2.0)
        marker.scale.z = float(self.object_half[2] * 2.0)
        marker.color.r = 0.05
        marker.color.g = 0.25
        marker.color.b = 0.95
        marker.color.a = 0.85
        self.marker_pub.publish(marker)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.base_frame
        tf_msg.child_frame_id = 'sim_blue_block'
        tf_msg.transform.translation.x = float(self.object_center[0])
        tf_msg.transform.translation.y = float(self.object_center[1])
        tf_msg.transform.translation.z = float(self.object_center[2])
        tf_msg.transform.rotation.x = qx
        tf_msg.transform.rotation.y = qy
        tf_msg.transform.rotation.z = qz
        tf_msg.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_gt_detection(self, stamp) -> None:
        x1, y1, x2, y2 = self._bbox
        det = self._Detection()
        det.x = x1
        det.y = y1
        det.width = max(0.0, x2 - x1)
        det.height = max(0.0, y2 - y1)
        det.confidence = 1.0
        det.class_id = 0
        det.class_name = 'blue_block'

        array = self._DetectionArray()
        array.header.stamp = stamp
        array.header.frame_id = self.frame_id
        array.detections = [det]
        self.detection_pub.publish(array)

    def _tick(self) -> None:
        tick_start = time.monotonic()
        stamp = self.get_clock().now().to_msg()
        self._camera_info.header.stamp = stamp

        if _as_bool(self.get_parameter('publish_image').value):
            self._image_msg.header.stamp = stamp
            self.image_pub.publish(self._image_msg)

        self._camera_info.header.stamp = stamp
        self.caminfo_pub.publish(self._camera_info)
        self._cloud_msg.header.stamp = stamp
        self.cloud_pub.publish(self._cloud_msg)
        self._publish_gt(stamp)
        if self.detection_pub is not None:
            self._publish_gt_detection(stamp)
        if _as_bool(self.get_parameter('timing_log').value):
            self.get_logger().info(
                f'tick {(time.monotonic() - tick_start) * 1000.0:.0f} ms')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BlueBlockScene()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
