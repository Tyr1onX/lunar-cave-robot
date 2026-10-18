"""Give Gazebo Fortress Pose_V transforms an explicit world parent frame."""
from __future__ import annotations

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


class GazeboPoseTfAdapter(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_pose_tf_adapter")
        self.declare_parameter("input_topic", "/p2_gazebo/gz_pose_tf")
        self.declare_parameter("output_topic", "/tf")
        self.declare_parameter("parent_frame", "world")
        self._parent = str(self.get_parameter("parent_frame").value)
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.RELIABLE)
        self._publisher = self.create_publisher(
            TFMessage, str(self.get_parameter("output_topic").value), qos)
        self.create_subscription(
            TFMessage, str(self.get_parameter("input_topic").value),
            self._on_pose_vector, qos)
        self.get_logger().info(
            f"Gazebo model poses will be published under {self._parent}; "
            "input is isolated Pose_V transport, output is TF only")

    def _on_pose_vector(self, message: TFMessage) -> None:
        resolved = TFMessage()
        for source in message.transforms:
            transform = copy.deepcopy(source)
            if not transform.header.frame_id:
                transform.header.frame_id = self._parent
            if not transform.child_frame_id or transform.child_frame_id == transform.header.frame_id:
                continue
            if transform.header.stamp.sec == 0 and transform.header.stamp.nanosec == 0:
                transform.header.stamp = self.get_clock().now().to_msg()
            resolved.transforms.append(transform)
        if resolved.transforms:
            self._publisher.publish(resolved)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboPoseTfAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
