from __future__ import annotations

import json
import os
import threading
import tempfile
import unittest
from datetime import datetime, timezone
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from pathlib import Path
from types import SimpleNamespace
from typing import Any
from unittest.mock import patch

from console_agent.collectors import ROSGraphObserver, _disable_default_parameter_event_publisher, _sync_link_telemetry_subscription, collect_logs
from console_agent.link_telemetry import LinkTelemetryObserver
from console_agent.server import HOST, ReadOnlyHandler, main


class AgentHTTPTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.server = ThreadingHTTPServer(("127.0.0.1", 0), ReadOnlyHandler)
        cls.thread = threading.Thread(target=cls.server.serve_forever, daemon=True)
        cls.thread.start()
        cls.port = cls.server.server_address[1]

    @classmethod
    def tearDownClass(cls) -> None:
        cls.server.shutdown()
        cls.server.server_close()
        cls.thread.join(timeout=2)

    def request(self, method: str, path: str) -> tuple[int, dict[str, str], bytes]:
        connection = HTTPConnection("127.0.0.1", self.port, timeout=2)
        connection.request(method, path)
        response = connection.getresponse()
        result = (response.status, dict(response.getheaders()), response.read())
        connection.close()
        return result

    def test_health_is_read_only_json(self) -> None:
        status, headers, body = self.request("GET", "/health")
        self.assertEqual(status, 200)
        self.assertEqual(headers["Content-Type"], "application/json; charset=utf-8")
        self.assertTrue(json.loads(body)["ok"])

    def assert_matches_schema(self, value: Any, schema: dict[str, Any], root: dict[str, Any], path: str = "$") -> None:
        if "$ref" in schema:
            reference = str(schema["$ref"])
            target: Any = root
            for part in reference.removeprefix("#/").split("/"):
                target = target[part]
            self.assert_matches_schema(value, target, root, path)
            return

        if "const" in schema:
            self.assertEqual(value, schema["const"], path)
        if "enum" in schema:
            self.assertIn(value, schema["enum"], path)

        expected_types = schema.get("type")
        if expected_types is not None:
            if isinstance(expected_types, str):
                expected_types = [expected_types]
            matches = False
            for expected_type in expected_types:
                if expected_type == "object" and isinstance(value, dict):
                    matches = True
                elif expected_type == "array" and isinstance(value, list):
                    matches = True
                elif expected_type == "string" and isinstance(value, str):
                    matches = True
                elif expected_type == "boolean" and isinstance(value, bool):
                    matches = True
                elif expected_type == "integer" and isinstance(value, int) and not isinstance(value, bool):
                    matches = True
                elif expected_type == "number" and isinstance(value, (int, float)) and not isinstance(value, bool):
                    matches = True
                elif expected_type == "null" and value is None:
                    matches = True
            self.assertTrue(matches, f"{path} does not match type {expected_types!r}: {value!r}")

        if isinstance(value, dict):
            properties = schema.get("properties", {})
            for key in schema.get("required", []):
                self.assertIn(key, value, f"{path}.{key} is required")
            if schema.get("additionalProperties") is False:
                self.assertLessEqual(set(value), set(properties), f"{path} has unexpected keys")
            for key, child_schema in properties.items():
                if key in value:
                    self.assert_matches_schema(value[key], child_schema, root, f"{path}.{key}")
        elif isinstance(value, list) and "items" in schema:
            for index, item in enumerate(value):
                self.assert_matches_schema(item, schema["items"], root, f"{path}[{index}]")

    def test_snapshot_endpoint_returns_complete_v1_contract(self) -> None:
        status, headers, body = self.request("GET", "/v1/snapshot")
        self.assertEqual(status, 200)
        self.assertEqual(headers["Content-Type"], "application/json; charset=utf-8")
        schema_path = Path(__file__).resolve().parents[1] / "schema" / "snapshot-v1.schema.json"
        schema = json.loads(schema_path.read_text(encoding="utf-8"))
        snapshot = json.loads(body)
        self.assertIn("workspaces", snapshot)
        self.assert_matches_schema(snapshot, schema, schema)

    def test_log_lines_label_file_modification_time_not_event_time(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            home = Path(temporary)
            log_dir = home / ".ros" / "log" / "latest"
            log_dir.mkdir(parents=True)
            log_file = log_dir / "node.log"
            event_time = "2026-10-13T05:00:00Z"
            log_file.write_text(f"{event_time} [INFO] message\n", encoding="utf-8")
            modified_epoch = datetime(2026, 10, 12, 23, 45, tzinfo=timezone.utc).timestamp()
            os.utime(log_file, (modified_epoch, modified_epoch))

            with patch.object(Path, "home", return_value=home):
                reading = collect_logs()

        self.assertEqual(reading["status"], "ok")
        entry = reading["entries"][0]
        expected_modified = "2026-10-12T23:45:00Z"
        self.assertEqual(entry["file_modified_at"], expected_modified)
        self.assertNotEqual(entry["file_modified_at"], event_time)
        self.assertNotIn("timestamp", entry)

    def test_logs_limit_is_bounded(self) -> None:
        with patch("console_agent.server.collect_logs", return_value={"status": "ok", "entries": []}) as collector:
            status, _, _ = self.request("GET", "/v1/logs?limit=9000")
        self.assertEqual(status, 200)
        collector.assert_called_once_with(200)

    def test_vision_endpoint_reports_only_observed_source_state(self) -> None:
        payload = {"status": "ok", "source": "nuc-ros2-read-only-subscriber", "streams": [{"id": "rgb", "status": "unavailable", "frame_rate_hz": None}]}
        with patch("console_agent.server.collect_vision", return_value=payload):
            status, headers, body = self.request("GET", "/v1/vision")
        self.assertEqual(status, 200)
        self.assertEqual(headers["Content-Type"], "application/json; charset=utf-8")
        self.assertEqual(json.loads(body), payload)

    def test_link_telemetry_endpoint_returns_fixed_allowlist_observations(self) -> None:
        payload = {
            "schema_version": 1,
            "status": "ok",
            "source": "nuc-ros2-fixed-topic-read-only-subscriber",
            "collected_at": "2026-10-17T00:00:00.000Z",
            "domain_id": 187,
            "expected_domain_id": None,
            "expected_domain_error": None,
            "domain_matches_expectation": None,
            "graph_status": "ok",
            "graph_observed_at": "2026-10-17T00:00:00.000Z",
            "graph_age_ms": 20,
            "graph_reason": None,
            "workspace_attribution": "unknown",
            "clock_note": "source and Agent clocks are not confirmed synchronized",
            "streams": [],
        }
        with patch("console_agent.server.collect_link_telemetry", return_value=payload):
            status, headers, body = self.request("GET", "/v1/telemetry")
        self.assertEqual(status, 200)
        self.assertEqual(headers["Content-Type"], "application/json; charset=utf-8")
        self.assertEqual(headers["Cache-Control"], "no-store")
        self.assertEqual(json.loads(body), payload)

    def test_link_telemetry_contract_matches_its_versioned_schema(self) -> None:
        payload = LinkTelemetryObserver().snapshot()
        schema_path = Path(__file__).resolve().parents[1] / "schema" / "link-telemetry-v1.schema.json"
        schema = json.loads(schema_path.read_text(encoding="utf-8"))
        self.assert_matches_schema(payload, schema, schema)

    def test_link_telemetry_subscribes_only_fixed_topic_and_discovered_allowlisted_type(self) -> None:
        class NodeStub:
            def __init__(self) -> None:
                self.subscriptions = []
                self.destroyed = []

            def create_subscription(self, message_class, topic, callback, qos_profile):
                value = (message_class, topic, callback, qos_profile)
                self.subscriptions.append(value)
                return value

            def destroy_subscription(self, _subscription):
                self.destroyed.append(_subscription)

        class PointCloud2:
            pass

        node = NodeStub()
        observer = LinkTelemetryObserver()
        observer.set_available_message_types({"sensor_msgs/msg/PointCloud2"})
        observer.graph_available(
            domain_id=187,
            observed_at="2026-10-17T00:00:00.000Z",
            expected_domain_id=None,
            expected_domain_error=None,
        )
        subscriptions = {}
        endpoints = [{
            "name": "synthetic_lidar", "namespace": "/sim",
            "topic_type": "sensor_msgs/msg/PointCloud2", "endpoint_gid": "01",
        }]
        observer.update_publishers("livox_lidar", endpoints)
        _sync_link_telemetry_subscription(
            node, "livox_lidar", endpoints, {"sensor_msgs/msg/PointCloud2": PointCloud2}, object(), subscriptions, observer
        )
        self.assertEqual(len(node.subscriptions), 1)
        self.assertIs(node.subscriptions[0][0], PointCloud2)
        self.assertEqual(node.subscriptions[0][1], "/livox/lidar")

        message = SimpleNamespace(
            header=SimpleNamespace(
                stamp=SimpleNamespace(sec=1_797_589_600, nanosec=0),
                frame_id="synthetic_lidar",
            ),
            width=10,
            height=1,
        )
        old_callback = node.subscriptions[0][2]
        old_callback(message)
        old_stream = next(item for item in observer.snapshot()["streams"] if item["id"] == "livox_lidar")
        self.assertEqual(old_stream["status"], "streaming")
        self.assertEqual(old_stream["subscription_generation"], 1)

        # A publisher restart may keep its name, namespace, and type while receiving a new ROS endpoint GID.
        replacement = [{**endpoints[0], "endpoint_gid": "02"}]
        observer.update_publishers("livox_lidar", replacement)
        _sync_link_telemetry_subscription(
            node,
            "livox_lidar",
            replacement,
            {"sensor_msgs/msg/PointCloud2": PointCloud2},
            object(),
            subscriptions,
            observer,
        )
        self.assertEqual(len(node.subscriptions), 2)
        self.assertEqual(len(node.destroyed), 1)
        waiting = next(item for item in observer.snapshot()["streams"] if item["id"] == "livox_lidar")
        self.assertEqual(waiting["status"], "waiting")
        self.assertEqual(waiting["subscription_generation"], 2)
        self.assertIsNone(waiting["received_at"])
        self.assertIsNone(waiting["age_ms"])
        self.assertEqual(waiting["previous_subscription_generation"], 1)
        self.assertIsNotNone(waiting["previous_received_at"])

        # A callback already queued for the destroyed publisher is rejected after the endpoint switch.
        old_callback(message)
        self.assertEqual(next(item for item in observer.snapshot()["streams"] if item["id"] == "livox_lidar")["receive_count"], 0)
        node.subscriptions[1][2](message)
        recovered = next(item for item in observer.snapshot()["streams"] if item["id"] == "livox_lidar")
        self.assertEqual(recovered["status"], "streaming")
        self.assertEqual(recovered["receive_count"], 1)

        _sync_link_telemetry_subscription(
            node,
            "livox_lidar",
            [{"name": "unknown", "namespace": "/", "topic_type": "example_msgs/msg/Arbitrary"}],
            {"example_msgs/msg/Arbitrary": object},
            object(),
            subscriptions,
            observer,
        )
        self.assertEqual(len(node.subscriptions), 2, "Unknown message types must not create dynamic subscriptions.")

    def test_vision_frame_is_a_no_store_png(self) -> None:
        with patch("console_agent.server.collect_vision_frame", return_value=(b"\x89PNG\r\n\x1a\n", None, None)):
            status, headers, body = self.request("GET", "/v1/vision/rgb/frame.png")
        self.assertEqual(status, 200)
        self.assertEqual(headers["Content-Type"], "image/png")
        self.assertEqual(headers["Cache-Control"], "no-store")
        self.assertTrue(body.startswith(b"\x89PNG"))

    def test_vision_frame_reports_missing_data_without_fabricating_an_image(self) -> None:
        with patch("console_agent.server.collect_vision_frame", return_value=(None, "no frame", 503)):
            status, _, body = self.request("GET", "/v1/vision/depth/frame.png")
        self.assertEqual(status, 503)
        self.assertEqual(json.loads(body), {"status": "unavailable", "reason": "no frame"})

    def test_unknown_route_and_write_methods_are_rejected(self) -> None:
        self.assertEqual(self.request("GET", "/run?command=id")[0], 404)
        status, headers, _ = self.request("POST", "/v1/snapshot")
        self.assertEqual(status, 405)
        self.assertEqual(headers["Allow"], "GET")

    def test_invalid_log_limit_is_rejected(self) -> None:
        self.assertEqual(self.request("GET", "/v1/logs?limit=all")[0], 400)

    def test_listener_configuration_cannot_bind_off_loopback(self) -> None:
        self.assertEqual(HOST, "127.0.0.1")
        with patch.dict("os.environ", {"LUNAR_CONSOLE_BIND": "0.0.0.0"}):
            with self.assertRaises(SystemExit):
                main()

    def test_ros_observer_removes_implicit_parameter_event_writer(self) -> None:
        class NodeStub:
            _parameter_event_publisher = object()

            def __init__(self) -> None:
                self.destroyed = []

            def destroy_publisher(self, publisher: object) -> None:
                self.destroyed.append(publisher)

        node = NodeStub()
        _disable_default_parameter_event_publisher(node)
        self.assertEqual(len(node.destroyed), 1)
        self.assertIsNone(node._parameter_event_publisher)

    def test_ros_snapshot_reports_monotonic_graph_age_and_configured_expectations(self) -> None:
        observer = ROSGraphObserver.__new__(ROSGraphObserver)
        observer._lock = threading.Lock()
        observer._snapshot = {"status": "ok", "domain_id": 42, "nodes": [], "topics": []}
        observer._last_observed_at = "2026-10-14T05:00:00.000Z"
        observer._last_observed_mono = __import__("time").monotonic() - 6.0
        with patch.dict(os.environ, {
            "LUNAR_CONSOLE_EXPECTED_ROS_DOMAIN_ID": "42",
            "LUNAR_CONSOLE_EXPECTED_ROS_NODES": "/camera/camera_node, yolo_detector, /camera/camera_node",
        }):
            reading = observer.snapshot()

        self.assertEqual(reading["observed_at"], "2026-10-14T05:00:00.000Z")
        self.assertGreaterEqual(reading["graph_age_ms"], 6000)
        self.assertEqual(reading["expected_domain_id"], 42)
        self.assertEqual(reading["expected_nodes"], ["/camera/camera_node", "/yolo_detector"])
        self.assertIsNone(reading["expected_domain_error"])
        self.assertIsNone(reading["expected_nodes_error"])

    def test_ros_expectation_errors_are_reported_without_inventing_defaults(self) -> None:
        observer = ROSGraphObserver.__new__(ROSGraphObserver)
        observer._lock = threading.Lock()
        observer._snapshot = {"status": "unknown", "domain_id": 0, "nodes": [], "topics": []}
        observer._last_observed_at = None
        observer._last_observed_mono = None
        with patch.dict(os.environ, {
            "LUNAR_CONSOLE_EXPECTED_ROS_DOMAIN_ID": "999",
            "LUNAR_CONSOLE_EXPECTED_ROS_NODES": "/valid_node,/invalid-node",
        }):
            reading = observer.snapshot()

        self.assertIsNone(reading["expected_domain_id"])
        self.assertIsNotNone(reading["expected_domain_error"])
        self.assertEqual(reading["expected_nodes"], [])
        self.assertIsNotNone(reading["expected_nodes_error"])


if __name__ == "__main__":
    unittest.main()
