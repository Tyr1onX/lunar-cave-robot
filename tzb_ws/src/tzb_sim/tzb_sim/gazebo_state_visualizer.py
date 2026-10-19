"""Visualize object pose and contact read only from Gazebo physics topics."""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from visualization_msgs.msg import Marker


class GazeboStateVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_state_visualizer")
        self.declare_parameter("tf_topic", "/tf")
        self.declare_parameter("contact_topic", "/p2_gazebo/blue_block/contacts")
        self.declare_parameter("object_marker_topic", "/p2_gazebo/physical_object_marker")
        self.declare_parameter("contact_status_topic", "/p2_gazebo/contact_status")
        self._marker_pub = self.create_publisher(
            Marker, str(self.get_parameter("object_marker_topic").value), 10)
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter("contact_status_topic").value), 10)
        self._finger_contact = False
        self._contact_frames = 0
        self.create_subscription(
            TFMessage, str(self.get_parameter("tf_topic").value),
            self._on_gazebo_tf, 30)
        self.create_subscription(
            Contacts, str(self.get_parameter("contact_topic").value),
            self._on_contacts, 20)
        self.get_logger().info(
            "RViz object marker is derived from Gazebo PosePublisher TF; "
            "no marker callback commands or alters simulated objects")

    @staticmethod
    def _is_block_frame(frame: str) -> bool:
        return frame == "blue_block" or frame.endswith("::blue_block") or \
            frame.endswith("blue_block::link")

    def _on_gazebo_tf(self, msg: TFMessage) -> None:
        for transform in msg.transforms:
            if not self._is_block_frame(transform.child_frame_id):
                continue
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            marker = Marker()
            marker.header = transform.header
            marker.ns = "gazebo_physical_target"
            marker.id = 0
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z = 0.05
            if self._finger_contact:
                marker.color.r = 0.1
                marker.color.g = 1.0
                marker.color.b = 0.15
            else:
                marker.color.r = 0.02
                marker.color.g = 0.16
                marker.color.b = 1.0
            marker.color.a = 0.85
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 300_000_000
            self._marker_pub.publish(marker)

    def _on_contacts(self, msg: Contacts) -> None:
        names = []
        for contact in msg.contacts:
            names.extend((contact.collision1.name, contact.collision2.name))
        unique = sorted(set(name for name in names if name))
        finger = any(
            "blue_block" in name and
            ("Claw_left_link" in name or "claw_right_link" in name)
            for name in unique)
        if finger:
            self._contact_frames += 1
        if finger != self._finger_contact:
            self._finger_contact = finger
            status = "FINGER_CONTACT" if finger else "NO_FINGER_CONTACT"
            self.get_logger().info(
                f"Gazebo physical contact state: {status}; "
                f"collision_pairs={unique}; frames={self._contact_frames}")
            self._status_pub.publish(String(data=status))

def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboStateVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
