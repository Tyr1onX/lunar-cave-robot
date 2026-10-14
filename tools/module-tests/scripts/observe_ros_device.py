#!/usr/bin/env python3
"""Read-only ROS graph/status inspection for four fixed robot device profiles."""

from __future__ import annotations

import argparse
import time
from collections import defaultdict


PROFILES = {
    "camera": {
        "label": "camera",
        "topics": ("/camera/color/image_raw", "/camera/depth/image_rect_raw", "/camera/color/camera_info", "/camera/depth/camera_info", "/diagnostics"),
        "sample": (("/camera/color/camera_info", "sensor_msgs/msg/CameraInfo"),),
        "diagnostic_terms": ("camera", "orbbec", "gemini"),
    },
    "mid360": {
        "label": "mid360",
        "topics": ("/livox/lidar", "/livox/lidar/pointcloud", "/livox/imu", "/diagnostics"),
        "sample": (("/livox/imu", "sensor_msgs/msg/Imu"),),
        "diagnostic_terms": ("livox", "mid360", "mid-360", "lidar"),
    },
    "chassis": {
        "label": "chassis controller",
        "topics": ("/chassis/serial_link_status", "/diagnostics"),
        "sample": (("/chassis/serial_link_status", "std_msgs/msg/String"),),
        "diagnostic_terms": ("chassis", "stm32", "serial"),
    },
    "arm": {
        "label": "arm controller",
        "topics": ("/serial/arm_safety/status", "/joint_states", "/p2_offline/arm_joint_state", "/diagnostics"),
        "sample": (("/serial/arm_safety/status", "std_msgs/msg/String"), ("/joint_states", "sensor_msgs/msg/JointState")),
        "diagnostic_terms": ("arm", "serial", "safety", "joint"),
    },
}

OBSERVATION_SECONDS = 2.5


def _sample_type(type_name: str):
    if type_name == "std_msgs/msg/String":
        from std_msgs.msg import String
        return String
    if type_name == "sensor_msgs/msg/CameraInfo":
        from sensor_msgs.msg import CameraInfo
        return CameraInfo
    if type_name == "sensor_msgs/msg/Imu":
        from sensor_msgs.msg import Imu
        return Imu
    if type_name == "sensor_msgs/msg/JointState":
        from sensor_msgs.msg import JointState
        return JointState
    raise ValueError(f"unsupported fixed observer message type: {type_name}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", choices=tuple(PROFILES), required=True)
    args = parser.parse_args()
    profile = PROFILES[args.device]

    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data

    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        f"lunar_console_readonly_{args.device}",
        context=context,
        start_parameter_services=False,
        enable_rosout=False,
    )
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    samples: dict[str, list[str]] = defaultdict(list)
    received: dict[str, int] = defaultdict(int)
    diagnostics: list[str] = []
    subscriptions = []
    try:
        def topic_map() -> dict[str, list[str]]:
            return {name: sorted(types) for name, types in node.get_topic_names_and_types()}

        topics = topic_map()
        discovery_deadline = time.monotonic() + 0.75
        while time.monotonic() < discovery_deadline:
            executor.spin_once(timeout_sec=0.05)
            topics = topic_map()

        for topic, message_type in profile["sample"]:
            if message_type not in topics.get(topic, []):
                continue
            cls = _sample_type(message_type)

            def callback(message, selected=topic):
                received[selected] += 1
                text = getattr(message, "data", None)
                if text:
                    samples[selected].append(str(text)[:240])
                elif hasattr(message, "name"):
                    pairs = zip(message.name, getattr(message, "position", ()))
                    summary = ", ".join(f"{name}={position:.4f} rad" for name, position in pairs)
                    if summary:
                        samples[selected].append(summary[:240])
                elif hasattr(message, "width") and hasattr(message, "height"):
                    samples[selected].append(f"{message.width}x{message.height} {getattr(message, 'distortion_model', '')}")

            subscriptions.append(node.create_subscription(cls, topic, callback, qos_profile_sensor_data))

        diagnostics_subscribed = False
        if "diagnostic_msgs/msg/DiagnosticArray" in topics.get("/diagnostics", []):
            from diagnostic_msgs.msg import DiagnosticArray

            def on_diagnostics(message):
                for item in message.status:
                    haystack = f"{item.name} {item.hardware_id}".lower()
                    if any(term in haystack for term in profile["diagnostic_terms"]):
                        level = "OK" if int(item.level) == 0 else ("WARN" if int(item.level) == 1 else "ERROR")
                        diagnostics.append(f"{level} {item.name}: {item.message}"[:300])

            subscriptions.append(node.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostics, 10))
            diagnostics_subscribed = True

        endpoints: dict[str, list[tuple[str, str]]] = {}
        started = time.monotonic()
        deadline = time.monotonic() + OBSERVATION_SECONDS
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
            topics = topic_map()
        observed_seconds = max(0.001, time.monotonic() - started)
        for topic in profile["topics"]:
            if topic in topics:
                endpoints[topic] = [
                    (publisher.node_namespace, publisher.node_name)
                    for publisher in node.get_publishers_info_by_topic(topic)
                ]

        print(f"DEVICE={profile['label']}")
        print("ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=1 observation=read-only")
        found = False
        for topic in profile["topics"]:
            topic_types = topics.get(topic, [])
            publishers = endpoints.get(topic, [])
            found = found or bool(publishers)
            state = "PUBLISHED" if publishers else "NO_PUBLISHER"
            publisher_names = ",".join(f"{namespace.rstrip('/')}/{name}" for namespace, name in publishers) or "-"
            print(f"TOPIC {topic} state={state} types={','.join(topic_types) or '-'} publishers={len(publishers)} nodes={publisher_names}")

        for topic, count in sorted(received.items()):
            hz = count / observed_seconds
            print(f"FREQUENCY {topic} samples={count} window_s={observed_seconds:.1f} approx_hz={hz:.2f}")
        for topic, messages in sorted(samples.items()):
            for message in messages[-2:]:
                print(f"STATUS {topic} {message}")

        if diagnostics_subscribed:
            diag_deadline = time.monotonic() + 0.25
            while time.monotonic() < diag_deadline:
                executor.spin_once(timeout_sec=0.05)
        if diagnostics:
            for detail in diagnostics[-20:]:
                print(f"DIAGNOSTIC {detail}")
        else:
            print("DIAGNOSTIC no matching diagnostics message observed")
        print(f"CONNECTION={'DISCOVERED' if found else 'NO_PUBLISHERS'}")
        print("READONLY_CHECK=COMPLETE; this is a status snapshot, not hardware acceptance")
        return 0
    finally:
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        if context.ok():
            rclpy.shutdown(context=context)


if __name__ == "__main__":
    raise SystemExit(main())
