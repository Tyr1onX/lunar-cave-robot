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
from typing import Any
from unittest.mock import patch

from console_agent.collectors import _disable_default_parameter_event_publisher, collect_logs
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
        self.assert_matches_schema(json.loads(body), schema, schema)

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


if __name__ == "__main__":
    unittest.main()
