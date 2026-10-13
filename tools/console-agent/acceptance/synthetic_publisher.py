"""Isolated ROS 2 Image publishers for Console V0.2a acceptance.

Never run on the robot ROS domain 0 or across LAN.
"""
import os
import time

if not os.getenv("ROS_DOMAIN_ID") or os.getenv("ROS_DOMAIN_ID") == "0" or os.getenv("ROS_LOCALHOST_ONLY") != "1":
    raise SystemExit("Set a nonzero ROS_DOMAIN_ID and ROS_LOCALHOST_ONLY=1")

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

TOPICS = (
    ("rgb", "/camera/color/image_raw", "rgb8", bytes((230, 35, 20)) * (64 * 48)),
    ("depth", "/camera/depth/image_rect_raw", "16UC1", b"\xe8\x03" * (64 * 48)),
    ("yolo", "/yolo/annotated_image", "rgb8", bytes((20, 220, 45)) * (64 * 48)),
)

def main() -> None:
    rclpy.init()
    node = rclpy.create_node("lunar_v02a_synthetic_publisher")
    publishers = [(name, encoding, data, node.create_publisher(Image, topic, qos_profile_sensor_data))
                  for name, topic, encoding, data in TOPICS]
    try:
        while rclpy.ok():
            stamp = node.get_clock().now().to_msg()
            for name, encoding, data, publisher in publishers:
                image = Image()
                image.header.stamp = stamp
                image.header.frame_id = f"lunar-v0.2a-synthetic/{name}"
                image.width, image.height = 64, 48
                image.encoding = encoding
                image.step = 64 * (2 if name == "depth" else 3)
                image.data = data
                publisher.publish(image)
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
