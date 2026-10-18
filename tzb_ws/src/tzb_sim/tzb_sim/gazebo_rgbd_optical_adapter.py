"""Convert Gazebo X-forward depth points into ROS camera optical coordinates."""
from __future__ import annotations

import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, PointCloud2
from sensor_msgs_py import point_cloud2


def gazebo_to_optical_xyz(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Gazebo camera (+X forward,+Y left,+Z up) to ROS optical (+Z forward)."""
    return -y, -z, x


class GazeboRgbdOpticalAdapter(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_rgbd_optical_adapter")
        self.declare_parameter("input_cloud_topic", "/p2_gazebo/camera/depth/color/points")
        self.declare_parameter("output_cloud_topic", "/p2_gazebo/camera/depth/optical/points")
        self.declare_parameter("input_info_topic", "/p2_gazebo/camera/color/camera_info")
        self.declare_parameter("output_info_topic", "/p2_gazebo/camera/optical/camera_info")
        self.declare_parameter("optical_frame", "sim_camera_optical_frame")
        frame = str(self.get_parameter("optical_frame").value)
        self._frame = frame
        self._cloud_pub = self.create_publisher(
            PointCloud2, str(self.get_parameter("output_cloud_topic").value),
            qos_profile_sensor_data)
        self._info_pub = self.create_publisher(
            CameraInfo, str(self.get_parameter("output_info_topic").value), 10)
        self.create_subscription(
            PointCloud2, str(self.get_parameter("input_cloud_topic").value),
            self._on_cloud, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, str(self.get_parameter("input_info_topic").value),
            self._on_camera_info, 10)
        self.get_logger().info(
            "Gazebo RGB-D data will be rotated from +X-forward,+Y-left,+Z-up "
            f"to ROS optical frame {frame}; source messages remain untouched")

    def _on_camera_info(self, message: CameraInfo) -> None:
        adapted = copy.deepcopy(message)
        adapted.header.frame_id = self._frame
        self._info_pub.publish(adapted)

    def _on_cloud(self, message: PointCloud2) -> None:
        if not all(name in {field.name for field in message.fields}
                   for name in ("x", "y", "z")):
            self.get_logger().error("Gazebo depth cloud is missing x/y/z fields")
            return
        try:
            dtype = point_cloud2.dtype_from_fields(
                message.fields, point_step=message.point_step)
            points = np.frombuffer(message.data, dtype=dtype).copy()
            x = points["x"].copy()
            y = points["y"].copy()
            z = points["z"].copy()
            points["x"] = -y
            points["y"] = -z
            points["z"] = x
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(f"cannot convert Gazebo depth cloud: {exc}")
            return
        adapted = copy.deepcopy(message)
        adapted.header.frame_id = self._frame
        adapted.data = points.tobytes()
        self._cloud_pub.publish(adapted)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboRgbdOpticalAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
