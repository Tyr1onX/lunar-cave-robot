from __future__ import annotations

import json
import sys
import tempfile
import threading
import time
import unittest
from http.client import HTTPConnection
from pathlib import Path

from console_agent.module_tests import ModuleTestHTTPServer, TestManager, TestSpec, serve


def spec(script: Path, timeout: int = 5) -> TestSpec:
    return TestSpec(
        id="safe_check", title="Safe check", description="test", mode="OFFLINE",
        input_source="generated test input", prerequisites=("Python",), timeout_seconds=timeout,
        command=lambda _repo: ([sys.executable, str(script)], {"input": "fixed"}, None),
        check=lambda _repo: [],
    )


class ModuleTestHTTPTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.script = self.root / "test_process.py"
        self.manager = TestManager(repo=self.root, state_root=self.root / "reports", specs=(spec(self.script),))
        self.server = ModuleTestHTTPServer(self.manager, ("127.0.0.1", 0))
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)
        self.thread.start()
        self.port = self.server.server_address[1]

    def tearDown(self) -> None:
        self.server.shutdown()
        self.server.server_close()
        self.thread.join(timeout=2)
        self.temporary.cleanup()

    def request(self, method: str, path: str, payload: dict[str, object] | None = None) -> tuple[int, dict[str, str], bytes]:
        connection = HTTPConnection("127.0.0.1", self.port, timeout=3)
        body = json.dumps(payload).encode("utf-8") if payload is not None else None
        headers = {"Content-Type": "application/json"} if body is not None else {}
        connection.request(method, path, body=body, headers=headers)
        response = connection.getresponse()
        result = response.status, dict(response.getheaders()), response.read()
        connection.close()
        return result

    def wait_for(self, run_id: str, statuses: set[str], timeout: float = 4.0, manager: TestManager | None = None) -> dict[str, object]:
        selected_manager = manager or self.manager
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            run = selected_manager.get(run_id)
            if run and run["status"] in statuses:
                return run
            time.sleep(0.02)
        self.fail(f"run {run_id} did not reach {statuses}; latest state: {run!r}")

    def test_only_allowlisted_id_and_no_extra_parameters_are_accepted(self) -> None:
        self.script.write_text("print('ran')\n", encoding="utf-8")
        status, _, body = self.request("POST", "/v1/tests/run", {"test_id": "safe_check", "command": "id"})
        self.assertEqual(status, 400)
        self.assertIn("only the allowlisted", json.loads(body)["error"])
        status, _, _ = self.request("POST", "/v1/tests/run", {"test_id": "../../anything"})
        self.assertEqual(status, 404)
        self.assertEqual(self.manager._runs, {})

    def test_report_persists_input_code_sha_parameters_and_logs(self) -> None:
        self.script.write_text("print('offline test passed')\n", encoding="utf-8")
        status, _, body = self.request("POST", "/v1/tests/run", {"test_id": "safe_check"})
        self.assertEqual(status, 202)
        run_id = json.loads(body)["run_id"]
        record = self.wait_for(run_id, {"passed"})
        self.assertEqual(record["mode"], "OFFLINE")
        self.assertEqual(record["input_source"], "generated test input")
        self.assertEqual(record["parameters"], {"input": "fixed"})
        self.assertIn("offline test passed", record["log_text"])
        self.assertTrue(Path(str(record["report_file"])).is_file())
        self.assertTrue(Path(str(record["log_file"])).is_file())
        self.assertFalse(self.manager.catalog()["busy"])
        report_status, _, report_body = self.request("GET", f"/v1/tests/{run_id}/report")
        self.assertEqual(report_status, 200)
        self.assertEqual(json.loads(report_body)["result"], "passed")

    def test_rejects_second_run_while_one_is_active(self) -> None:
        self.script.write_text("import time\nprint('started', flush=True)\ntime.sleep(2)\n", encoding="utf-8")
        status, _, body = self.request("POST", "/v1/tests/run", {"test_id": "safe_check"})
        self.assertEqual(status, 202)
        run_id = json.loads(body)["run_id"]
        status, _, body = self.request("POST", "/v1/tests/run", {"test_id": "safe_check"})
        self.assertEqual(status, 409)
        self.assertIn("already running", json.loads(body)["error"])
        self.wait_for(run_id, {"passed"})

    def test_cancellation_terminates_child_and_records_cancelled(self) -> None:
        self.script.write_text("import time\nprint('started', flush=True)\ntime.sleep(30)\n", encoding="utf-8")
        status, _, body = self.request("POST", "/v1/tests/run", {"test_id": "safe_check"})
        self.assertEqual(status, 202)
        run_id = json.loads(body)["run_id"]
        deadline = time.monotonic() + 2
        child = None
        while time.monotonic() < deadline:
            with self.manager._lock:
                child = self.manager._runs[run_id]["process"]
            if child is not None:
                break
            time.sleep(0.01)
        self.assertIsNotNone(child)
        self.request("POST", f"/v1/tests/{run_id}/cancel")
        record = self.wait_for(run_id, {"cancelled"})
        self.assertNotIsInstance(child.poll(), type(None))
        self.assertEqual(record["result"], "cancelled")

    def test_timeout_kills_child_and_records_timed_out(self) -> None:
        self.script.write_text("import time\nprint('started', flush=True)\ntime.sleep(30)\n", encoding="utf-8")
        self.manager.specs["safe_check"] = spec(self.script, timeout=1)
        status, _, body = self.request("POST", "/v1/tests/run", {"test_id": "safe_check"})
        self.assertEqual(status, 202)
        run_id = json.loads(body)["run_id"]
        record = self.wait_for(run_id, {"timed_out"}, timeout=4)
        self.assertEqual(record["result"], "timed_out")
        self.assertIn("超时", record["reason"])

    def test_catalog_marks_yolo_locked_when_external_model_is_not_configured(self) -> None:
        from console_agent.module_tests import DEFAULT_TESTS
        manager = TestManager(repo=self.root, state_root=self.root / "reports", specs=DEFAULT_TESTS)
        yolo = next(item for item in manager.catalog()["tests"] if item["id"] == "yolo_offline")
        self.assertEqual(yolo["mode"], "OFFLINE")
        self.assertEqual(yolo["state"], "locked")
        self.assertIn("模型目录", yolo["reason"])

    def test_catalog_registers_expanded_offline_and_readonly_profiles(self) -> None:
        from console_agent.module_tests import DEFAULT_TESTS

        manager = TestManager(repo=self.root, state_root=self.root / "reports", specs=DEFAULT_TESTS)
        catalog = {item["id"]: item for item in manager.catalog()["tests"]}
        self.assertTrue({"chassis_protocol", "p6_unit", "p6_scenarios", "p7_scenarios"}.issubset(catalog))
        self.assertEqual(catalog["p6_scenarios"]["mode"], "SIMULATION")
        self.assertEqual(catalog["p7_scenarios"]["mode"], "SIMULATION")
        for test_id in ("camera_readonly", "mid360_readonly", "chassis_readonly", "arm_readonly"):
            self.assertEqual(catalog[test_id]["mode"], "LIVE")
            self.assertEqual(catalog[test_id]["category"], "只读设备巡检")

    def test_yolo_discovers_existing_field_lab_model_without_an_env_override(self) -> None:
        from unittest.mock import patch
        from console_agent import module_tests

        with patch.dict("os.environ", {"LUNAR_CONSOLE_YOLO_MODEL_DIR": ""}, clear=False), \
             patch.object(module_tests, "_field_lab_root", return_value=self.root / "lunar-field-lab-p9i"), \
             patch.object(module_tests.Path, "home", return_value=self.root):
            model = self.root / "lunar-field-lab-p9i" / "models" / "blue_block"
            model.mkdir(parents=True)
            self.assertEqual(module_tests._yolo_model_dir(), model)

    def test_live_commands_are_fixed_readonly_profiles(self) -> None:
        from console_agent.module_tests import DEFAULT_TESTS

        specs = {item.id: item for item in DEFAULT_TESTS}
        for test_id, device in (("camera_readonly", "camera"), ("mid360_readonly", "mid360"),
                                ("chassis_readonly", "chassis"), ("arm_readonly", "arm")):
            spec_item = specs[test_id]
            command, parameters, _model = spec_item.command(self.root)
            self.assertEqual(command[-2:], ["--device", device])
            self.assertEqual(spec_item.ros_domain_id, 0)
            self.assertEqual(parameters["ros_localhost_only"], "1")
            self.assertIn("no driver", parameters["observation"])

    def test_runner_rejects_non_loopback_bind_configuration(self) -> None:
        from unittest.mock import patch

        with patch.dict("os.environ", {"LUNAR_CONSOLE_TEST_BIND": "0.0.0.0"}):
            with self.assertRaises(SystemExit):
                serve()

    def test_module_test_service_allows_ros_interface_discovery_without_packet_sockets(self) -> None:
        repo = Path(__file__).resolve().parents[3]
        installer = (repo / "tools/console-agent/install-module-test-service.sh").read_text(encoding="utf-8")
        families = next(line for line in installer.splitlines() if '"RestrictAddressFamilies=' in line)
        self.assertIn("AF_NETLINK", families)
        self.assertNotIn("AF_PACKET", families)

    def test_report_captures_model_digest_when_a_registered_test_uses_one(self) -> None:
        model_dir = self.root / "model"
        model_dir.mkdir()
        for name, payload in (("metadata.yaml", b"task: segment\n"), ("yoloe-26l-seg.xml", b"xml"), ("yoloe-26l-seg.bin", b"weights")):
            (model_dir / name).write_bytes(payload)
        model_script = self.root / "test_model.py"
        model_script.write_text("print('model test passed')\n", encoding="utf-8")
        model_spec = TestSpec(
            id="model_check", title="Model check", description="test", mode="OFFLINE",
            input_source="local fixed model", prerequisites=("model",), timeout_seconds=5,
            command=lambda _repo: ([sys.executable, str(model_script)], {"input": "fixed"}, str(model_dir)),
            check=lambda _repo: [],
        )
        manager = TestManager(repo=self.root, state_root=self.root / "model-reports", specs=(model_spec,))
        record, error, status = manager.start("model_check")
        self.assertIsNone(error)
        self.assertEqual(status, 202)
        assert record is not None
        completed = self.wait_for(str(record["run_id"]), {"passed"}, manager=manager)
        self.assertEqual(len(str(completed["model_sha256"])), 64)

    def test_registered_arm_fk_runs_through_http_and_saves_report(self) -> None:
        repo = Path(__file__).resolve().parents[3]
        from console_agent.module_tests import DEFAULT_TESTS

        manager = TestManager(repo=repo, state_root=self.root / "fk-reports", specs=DEFAULT_TESTS)
        self.server.manager = manager
        status, _, body = self.request("POST", "/v1/tests/run", {"test_id": "arm_fk"})
        self.assertEqual(status, 202)
        run_id = json.loads(body)["run_id"]
        completed = self.wait_for(run_id, {"passed"}, manager=manager)
        self.assertEqual(completed["mode"], "OFFLINE")
        self.assertEqual(completed["allowlisted_script"], "tzb_ws/src/serial/driver/test/test_tzb_urdf_joint_fk.py")
        self.assertEqual(completed["code_sha"], self._git_sha(repo))
        self.assertIn("URDF FK checks passed", completed["log_text"])

    @staticmethod
    def _git_sha(repo: Path) -> str:
        import subprocess

        return subprocess.check_output(["/usr/bin/git", "-C", str(repo), "rev-parse", "HEAD"], text=True).strip()


if __name__ == "__main__":
    unittest.main()
