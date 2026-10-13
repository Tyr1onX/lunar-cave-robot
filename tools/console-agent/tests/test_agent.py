from __future__ import annotations

import json
import threading
import unittest
from http.client import HTTPConnection
from http.server import ThreadingHTTPServer
from unittest.mock import patch

from console_agent.collectors import _disable_default_parameter_event_publisher
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

    def test_snapshot_endpoint_returns_real_collector_contract(self) -> None:
        fake = {"schema_version": 1, "source": "nuc-read-only-agent", "git": {"status": "unknown"}}
        with patch("console_agent.server.collect_snapshot", return_value=fake):
            status, _, body = self.request("GET", "/v1/snapshot")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body), fake)

    def test_logs_limit_is_bounded(self) -> None:
        with patch("console_agent.server.collect_logs", return_value={"status": "ok", "entries": []}) as collector:
            status, _, _ = self.request("GET", "/v1/logs?limit=9000")
        self.assertEqual(status, 200)
        collector.assert_called_once_with(200)

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
