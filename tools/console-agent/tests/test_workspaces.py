from __future__ import annotations

import ast
import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

from console_agent.workspaces import (
    ModuleSpec,
    WorkspaceSpec,
    _INVENTORY_CACHE,
    _CACHE_LOCK,
    _graph_relationship,
    collect_workspaces,
)


class WorkspaceInventoryTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "workspace"
        self.source = self.root / "src" / "yolo_node"
        (self.source / "launch").mkdir(parents=True)
        (self.source / "config").mkdir(parents=True)
        (self.source / "package.xml").write_text("<package><name>yolo_node</name></package>", encoding="utf-8")
        (self.source / "launch" / "yolo_node.launch.py").write_text("# source only\n", encoding="utf-8")
        (self.source / "config" / "params.yaml").write_text("threshold: 0.5\n", encoding="utf-8")
        self.spec = WorkspaceSpec(
            id="original-vision",
            label="原始机械臂与视觉工程",
            source_kind="original",
            root=self.root,
            source_roots=(self.root,),
            modules=(ModuleSpec("yolo", "YOLO", ("yolo_node",), ("/yolo/detections",)),),
        )
        with _CACHE_LOCK:
            _INVENTORY_CACHE.clear()
        self.expectation_environment = patch.dict("os.environ", {"LUNAR_CONSOLE_EXPECTED_MODULES": ""}, clear=False)
        self.expectation_environment.start()
        self.addCleanup(self.expectation_environment.stop)

    def _fake_process(
        self,
        command: bytes,
        environment: bytes,
        *,
        comm: str = "python3",
        cwd: Path | None = None,
        maps: str | None = None,
    ) -> Path:
        proc = Path(self.temporary.name) / "proc"
        process = proc / "412"
        process.mkdir(parents=True)
        (process / "comm").write_text(f"{comm}\n", encoding="utf-8")
        (process / "cmdline").write_bytes(command)
        (process / "environ").write_bytes(environment)
        (process / "cwd").symlink_to(cwd or self.root, target_is_directory=True)
        executable = Path(self.temporary.name) / "unrelated-executable"
        executable.touch(exist_ok=True)
        (process / "exe").symlink_to(executable)
        if maps is not None:
            (process / "maps").write_text(maps, encoding="utf-8")
        return proc

    def _empty_proc(self) -> Path:
        proc = Path(self.temporary.name) / "empty-proc"
        proc.mkdir(exist_ok=True)
        return proc

    def test_source_presence_is_separate_from_runtime_expectation(self) -> None:
        reading = collect_workspaces(specs=(self.spec,), proc_root=self._empty_proc())
        workspace = reading["workspaces"][0]
        self.assertEqual(workspace["source_status"], "present")
        self.assertEqual(workspace["package_count"], 1)
        self.assertEqual(workspace["launch_count"], 1)
        self.assertEqual(workspace["config_count"], 1)
        self.assertEqual(workspace["runtime_status"], "not_running")
        self.assertEqual(workspace["execution_state"], "not_running")
        self.assertEqual(workspace["run_expectation"], "unconfigured")
        self.assertEqual(workspace["modules"][0]["runtime_status"], "not_running")
        self.assertEqual(workspace["modules"][0]["execution_state"], "not_running")
        self.assertEqual(workspace["modules"][0]["run_expectation"], "unconfigured")
        self.assertNotIn("abnormal", json.dumps(reading, ensure_ascii=False))

    def test_explicit_module_expectation_is_opt_in_and_workspace_scoped(self) -> None:
        second_workspace = WorkspaceSpec(
            id="original-navigation",
            label="原始导航工程",
            source_kind="original",
            root=self.root,
            source_roots=(self.root,),
            modules=(ModuleSpec("navigation", "导航", ("bringup",), ("/scan",)),),
        )
        with patch.dict("os.environ", {"LUNAR_CONSOLE_EXPECTED_MODULES": "original-vision:yolo"}):
            reading = collect_workspaces(specs=(self.spec, second_workspace), proc_root=self._empty_proc())

        self.assertEqual(reading["expectation_status"], "configured")
        self.assertEqual(reading["expectation_source"], "LUNAR_CONSOLE_EXPECTED_MODULES")
        yolo, = reading["workspaces"][0]["modules"]
        navigation, = reading["workspaces"][1]["modules"]
        self.assertEqual(yolo["run_expectation"], "confirmed")
        self.assertEqual(yolo["run_expectation_source"], "LUNAR_CONSOLE_EXPECTED_MODULES")
        self.assertIsNotNone(yolo["run_expectation_observed_at"])
        self.assertIn("显式 Agent 配置", yolo["run_expectation_reason"])
        self.assertEqual(navigation["run_expectation"], "unconfigured")
        self.assertIsNone(navigation["run_expectation_source"])
        self.assertIsNone(reading["expectation_error"])

    def test_invalid_module_expectation_fails_closed(self) -> None:
        with patch.dict(
            "os.environ",
            {"LUNAR_CONSOLE_EXPECTED_MODULES": "original-vision:yolo,original-vision:not-a-module"},
        ):
            reading = collect_workspaces(specs=(self.spec,), proc_root=self._empty_proc())

        self.assertEqual(reading["expectation_status"], "invalid")
        self.assertIn("unknown workspace/module", reading["expectation_error"])
        self.assertEqual(reading["workspaces"][0]["modules"][0]["run_expectation"], "unknown")
        self.assertEqual(reading["workspaces"][0]["modules"][0]["run_expectation_source"], None)
        self.assertNotIn("confirmed", json.dumps(reading, ensure_ascii=False))

    def test_malformed_module_expectation_fails_closed(self) -> None:
        with patch.dict("os.environ", {"LUNAR_CONSOLE_EXPECTED_MODULES": "original-vision:yolo,"}):
            reading = collect_workspaces(specs=(self.spec,), proc_root=self._empty_proc())

        self.assertEqual(reading["expectation_status"], "invalid")
        self.assertIn("comma-separated", reading["expectation_error"])
        self.assertEqual(reading["workspaces"][0]["modules"][0]["run_expectation"], "unknown")

    def test_explicit_simulation_mode_is_reported_as_process_declaration(self) -> None:
        proc = self._fake_process(
            b"/opt/ros/humble/bin/ros2\0run\0yolo_node\0yolo_detector\0",
            f"AMENT_PREFIX_PATH={self.root / 'install'}\0ROS_DOMAIN_ID=187\0LUNAR_CONSOLE_RUN_MODE=simulation\0".encode(),
        )

        reading = collect_workspaces(
            {"status": "ok", "domain_id": 187, "graph_age_ms": 100},
            specs=(self.spec,), proc_root=proc,
        )

        workspace = reading["workspaces"][0]
        self.assertEqual(workspace["execution_state"], "simulation_running")
        self.assertEqual(workspace["modules"][0]["execution_state"], "simulation_running")
        self.assertEqual(workspace["processes"][0]["run_mode_evidence"], "explicit_environment")
        self.assertEqual(workspace["modules"][0]["run_expectation"], "unconfigured")
        self.assertNotIn("abnormal", json.dumps(reading, ensure_ascii=False))

    def test_explicit_hardware_mode_does_not_claim_hardware_health(self) -> None:
        proc = self._fake_process(
            b"/opt/ros/humble/bin/ros2\0run\0yolo_node\0yolo_detector\0",
            f"AMENT_PREFIX_PATH={self.root / 'install'}\0ROS_DOMAIN_ID=187\0LUNAR_CONSOLE_RUN_MODE=hardware\0".encode(),
        )

        reading = collect_workspaces(
            {"status": "ok", "domain_id": 187, "graph_age_ms": 100},
            specs=(self.spec,), proc_root=proc,
        )

        workspace = reading["workspaces"][0]
        self.assertEqual(workspace["execution_state"], "hardware_declared_running")
        self.assertEqual(workspace["run_mode"], "hardware")
        self.assertEqual(workspace["processes"][0]["run_mode_evidence"], "explicit_environment")
        self.assertEqual(workspace["modules"][0]["run_expectation"], "unconfigured")
        self.assertNotIn("hardware_health", json.dumps(reading))

    def test_conflicting_process_mode_declarations_remain_unknown(self) -> None:
        proc = Path(self.temporary.name) / "mixed-mode-proc"
        for pid, mode in (("801", "simulation"), ("802", "hardware")):
            process = proc / pid
            process.mkdir(parents=True)
            (process / "comm").write_text("yolo_detector\n", encoding="utf-8")
            command = f"{self.root}/install/yolo_node/lib/yolo_node/yolo_detector\0".encode()
            (process / "cmdline").write_bytes(command)
            (process / "environ").write_bytes(
                f"AMENT_PREFIX_PATH={self.root / 'install'}\0LUNAR_CONSOLE_RUN_MODE={mode}\0".encode()
            )
            (process / "cwd").symlink_to(self.root, target_is_directory=True)

        reading = collect_workspaces(
            {"status": "ok", "domain_id": 187, "graph_age_ms": 100},
            specs=(self.spec,), proc_root=proc,
        )

        workspace = reading["workspaces"][0]
        self.assertEqual(workspace["runtime_status"], "running")
        self.assertEqual(workspace["run_mode"], "mixed")
        self.assertEqual(workspace["execution_state"], "unknown")
        self.assertEqual(workspace["modules"][0]["runtime_status"], "running")
        self.assertEqual(workspace["modules"][0]["execution_state"], "unknown")

    def test_symlinked_source_is_not_followed(self) -> None:
        outside = Path(self.temporary.name) / "outside"
        outside.mkdir()
        (outside / "package.xml").write_text("<package><name>outside_package</name></package>", encoding="utf-8")
        (self.root / "src" / "linked_package").symlink_to(outside, target_is_directory=True)

        reading = collect_workspaces(specs=(self.spec,), proc_root=self._empty_proc())

        self.assertEqual(reading["workspaces"][0]["package_count"], 1)
        self.assertNotIn("outside_package", {item["name"] for item in reading["workspaces"][0]["packages"]})

    def test_domain_mismatch_keeps_graph_relationship_unknown_without_fault(self) -> None:
        proc = self._fake_process(
            b"/opt/ros/humble/bin/ros2\0run\0yolo_node\0yolo_detector\0",
            f"AMENT_PREFIX_PATH={self.root / 'install'}\0ROS_DOMAIN_ID=218\0ROS_LOCALHOST_ONLY=1\0".encode(),
        )

        reading = collect_workspaces(
            {"status": "ok", "domain_id": 187, "observed_at": "2026-10-16T10:00:00Z", "graph_age_ms": 100,
             "topics": [{"name": "/yolo/detections", "types": ["vision_msgs/msg/Detection2DArray"]}]},
            specs=(self.spec,),
            proc_root=proc,
        )
        workspace = reading["workspaces"][0]

        self.assertEqual(workspace["runtime_status"], "running")
        self.assertEqual(workspace["execution_state"], "unknown")
        self.assertEqual(workspace["processes"][0]["domain_id"], 218)
        self.assertEqual(workspace["graph_relationship"], "different_domain")
        self.assertEqual(workspace["modules"][0]["runtime_status"], "running")
        self.assertEqual(workspace["modules"][0]["run_expectation"], "unconfigured")
        self.assertEqual(workspace["modules"][0]["observed_interface_topics"], [])
        self.assertNotIn("abnormal", json.dumps(reading, ensure_ascii=False))

    def test_unset_domain_and_stale_graph_do_not_claim_communication(self) -> None:
        proc = self._fake_process(
            b"/opt/ros/humble/bin/ros2\0run\0yolo_node\0yolo_detector\0",
            f"AMENT_PREFIX_PATH={self.root / 'install'}\0".encode(),
        )

        reading = collect_workspaces(
            {"status": "ok", "domain_id": 187, "observed_at": "2026-10-16T10:00:00Z", "graph_age_ms": 6000,
             "topics": [{"name": "/yolo/detections", "types": ["vision_msgs/msg/Detection2DArray"]}]},
            specs=(self.spec,),
            proc_root=proc,
        )
        workspace = reading["workspaces"][0]

        self.assertEqual(workspace["processes"][0]["domain_evidence"], "unset")
        self.assertEqual(workspace["processes"][0]["domain_id"], None)
        self.assertEqual(workspace["graph_relationship"], "unknown")

    def test_shared_domain_is_only_graph_visibility_evidence(self) -> None:
        proc = self._fake_process(
            b"/opt/ros/humble/bin/ros2\0run\0yolo_node\0yolo_detector\0",
            f"AMENT_PREFIX_PATH={self.root / 'install'}\0ROS_DOMAIN_ID=187\0".encode(),
        )

        reading = collect_workspaces(
            {"status": "ok", "domain_id": 187, "observed_at": "2026-10-16T10:00:00Z", "graph_age_ms": 100,
             "topics": [{"name": "/yolo/detections", "types": ["vision_msgs/msg/Detection2DArray"]}]},
            specs=(self.spec,),
            proc_root=proc,
        )

        self.assertEqual(reading["workspaces"][0]["graph_relationship"], "shared_domain")
        self.assertIn("仍需以 graph 实际发现为准", reading["workspaces"][0]["graph_reason"])
        self.assertEqual(reading["workspaces"][0]["modules"][0]["observed_interface_topics"], ["/yolo/detections"])

    def test_partial_process_scan_keeps_workspace_graph_relationship_unknown(self) -> None:
        relationship, reason = _graph_relationship(
            [{"domain_id": 187}],
            {"status": "ok", "domain_id": 187, "graph_age_ms": 100},
            "partial",
        )
        self.assertEqual(relationship, "unknown")
        self.assertIn("不完整", reason)

    def test_sourced_terminal_is_not_misclassified_as_a_ros_node(self) -> None:
        proc = self._fake_process(
            b"/bin/bash\0-i\0",
            f"AMENT_PREFIX_PATH={self.root / 'install'}\0ROS_DOMAIN_ID=187\0".encode(),
        )

        reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        self.assertEqual(reading["workspaces"][0]["process_count"], 0)
        self.assertEqual(reading["workspaces"][0]["runtime_status"], "not_running")

    def test_unreadable_unrelated_session_process_does_not_make_scan_partial(self) -> None:
        proc = self._fake_process(
            b"/usr/lib/systemd/systemd --user\0",
            b"",
            comm="sd-pam",
            cwd=Path(self.temporary.name),
        )

        with patch("console_agent.workspaces._read_process_environment", side_effect=PermissionError):
            reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        self.assertEqual(reading["process_scan"]["status"], "ok")
        self.assertIsNone(reading["process_scan"]["reason"])
        self.assertEqual(reading["workspaces"][0]["runtime_status"], "not_running")

    def test_unreadable_observed_sd_pam_identity_and_argv_pair_is_excluded(self) -> None:
        proc = self._fake_process(
            b"(sd-pam)\0",
            b"",
            comm="(sd-pam)",
            cwd=Path(self.temporary.name),
        )

        with patch("console_agent.workspaces._read_process_environment", side_effect=PermissionError):
            reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        self.assertEqual(reading["process_scan"]["status"], "ok")
        self.assertIsNone(reading["process_scan"]["reason"])
        self.assertEqual(reading["workspaces"][0]["runtime_status"], "not_running")

    def test_sd_pam_comm_with_other_argv_still_requires_readable_process_evidence(self) -> None:
        command = f"{self.root}/install/yolo_node/lib/yolo_node/yolo_detector\0".encode()
        proc = self._fake_process(command, b"", comm="(sd-pam)")

        with patch("console_agent.workspaces._read_process_environment", side_effect=PermissionError):
            reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        self.assertEqual(reading["process_scan"]["status"], "partial")
        self.assertEqual(reading["workspaces"][0]["runtime_status"], "running")
        self.assertEqual(reading["workspaces"][0]["processes"][0]["domain_evidence"], "unavailable")

    def test_unreadable_ssh_session_wrapper_does_not_make_scan_partial(self) -> None:
        proc = self._fake_process(
            b"sshd: operator@notty\0",
            b"",
            comm="sshd",
            cwd=Path(self.temporary.name),
        )

        with patch("console_agent.workspaces._read_process_environment", side_effect=PermissionError):
            reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        self.assertEqual(reading["process_scan"]["status"], "ok")
        self.assertIsNone(reading["process_scan"]["reason"])
        self.assertEqual(reading["workspaces"][0]["runtime_status"], "not_running")

    def test_non_session_process_named_sshd_is_still_scanned(self) -> None:
        unrelated_cwd = Path(self.temporary.name) / "custom-runner"
        unrelated_cwd.mkdir()
        command = f"{self.root}/install/yolo_node/lib/yolo_node/yolo_detector\0".encode()
        proc = self._fake_process(
            command,
            f"AMENT_PREFIX_PATH={self.root / 'install'}\0".encode(),
            comm="sshd",
            cwd=unrelated_cwd,
        )

        reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        workspace = reading["workspaces"][0]
        self.assertEqual(workspace["runtime_status"], "running")
        self.assertEqual(workspace["process_count"], 1)
        self.assertEqual(workspace["processes"][0]["process_name"], "sshd")

    def test_unreadable_ros_candidate_keeps_command_evidence_and_marks_scope_partial(self) -> None:
        command = f"{self.root}/install/yolo_node/lib/yolo_node/yolo_detector\0".encode()
        proc = self._fake_process(command, b"", comm="yolo_detector")

        with patch("console_agent.workspaces._read_process_environment", side_effect=PermissionError):
            reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        workspace = reading["workspaces"][0]
        self.assertEqual(reading["process_scan"]["status"], "partial")
        self.assertIn("环境字段不可读", reading["process_scan"]["reason"])
        self.assertEqual(workspace["runtime_status"], "running")
        self.assertEqual(workspace["processes"][0]["package"], "yolo_node")
        self.assertEqual(workspace["processes"][0]["domain_evidence"], "unavailable")
        self.assertEqual(workspace["graph_relationship"], "unknown")

    def test_custom_ros_executable_is_found_without_a_known_process_name(self) -> None:
        unrelated_cwd = Path(self.temporary.name) / "custom-runner"
        unrelated_cwd.mkdir()
        proc = self._fake_process(
            b"/opt/robot/bin/vision_pipeline\0",
            f"AMENT_PREFIX_PATH={self.root / 'install'}\0ROS_DOMAIN_ID=187\0".encode(),
            comm="vision_pipeline",
            cwd=unrelated_cwd,
            maps="7f000000-7f001000 r-xp 00000000 08:01 42 /opt/ros/humble/lib/librclcpp.so\n",
        )

        reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        workspace = reading["workspaces"][0]
        self.assertEqual(reading["process_scan"]["status"], "ok")
        self.assertEqual(workspace["runtime_status"], "running")
        self.assertEqual(workspace["process_count"], 1)
        self.assertEqual(workspace["processes"][0]["process_name"], "vision_pipeline")
        self.assertIn("overlay_environment", workspace["processes"][0]["matched_by"])

    def test_unreadable_unclassified_process_prevents_complete_not_running_claim(self) -> None:
        unrelated_cwd = Path(self.temporary.name) / "custom-runner"
        unrelated_cwd.mkdir()
        proc = self._fake_process(
            b"/opt/robot/bin/custom_runtime\0",
            b"",
            comm="custom_runtime",
            cwd=unrelated_cwd,
        )

        with patch("console_agent.workspaces._read_process_environment", side_effect=PermissionError):
            reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        self.assertEqual(reading["process_scan"]["status"], "partial")
        self.assertEqual(reading["workspaces"][0]["runtime_status"], "unknown")
        self.assertIn("环境字段不可读", reading["process_scan"]["reason"])

    def test_unreadable_process_identity_prevents_complete_not_running_claim(self) -> None:
        proc = self._fake_process(b"/opt/robot/bin/custom_runtime\0", b"", comm="custom_runtime")
        read_text = Path.read_text

        def deny_comm(path: Path, *args, **kwargs):
            if path.name == "comm":
                raise PermissionError("synthetic denied process identity")
            return read_text(path, *args, **kwargs)

        with patch.object(Path, "read_text", deny_comm):
            reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        self.assertEqual(reading["process_scan"]["status"], "partial")
        self.assertEqual(reading["workspaces"][0]["runtime_status"], "unknown")
        self.assertIn("身份信息不可读", reading["process_scan"]["reason"])

    def test_overlaid_workspaces_with_duplicate_package_are_not_guessed(self) -> None:
        arm_root = Path(self.temporary.name) / "arm"
        nav_root = Path(self.temporary.name) / "nav"
        for root in (arm_root, nav_root):
            package_dir = root / "src" / "rm_serial_driver"
            package_dir.mkdir(parents=True)
            (package_dir / "package.xml").write_text("<package><name>rm_serial_driver</name></package>", encoding="utf-8")
        arm = WorkspaceSpec("arm", "机械臂", "original", arm_root, (arm_root,), ())
        nav = WorkspaceSpec("nav", "导航", "original", nav_root, (nav_root,), ())
        proc = Path(self.temporary.name) / "ambiguous-proc"
        process = proc / "812"
        process.mkdir(parents=True)
        (process / "comm").write_text("python3\n", encoding="utf-8")
        (process / "cmdline").write_bytes(b"/opt/ros/humble/bin/ros2\0run\0rm_serial_driver\0serial_driver\0")
        (process / "environ").write_bytes(
            f"AMENT_PREFIX_PATH={arm_root / 'install'}:{nav_root / 'install'}\0ROS_DOMAIN_ID=187\0".encode()
        )
        unrelated_cwd = Path(self.temporary.name) / "runner"
        unrelated_cwd.mkdir()
        (process / "cwd").symlink_to(unrelated_cwd, target_is_directory=True)

        reading = collect_workspaces(
            {"status": "ok", "domain_id": 187, "graph_age_ms": 100},
            specs=(arm, nav),
            proc_root=proc,
        )

        self.assertEqual(reading["process_scan"]["status"], "partial")
        self.assertIn("无法安全归属", reading["process_scan"]["reason"])
        self.assertEqual([workspace["process_count"] for workspace in reading["workspaces"]], [0, 0])
        self.assertEqual([workspace["runtime_status"] for workspace in reading["workspaces"]], ["unknown", "unknown"])
        self.assertEqual([workspace["execution_state"] for workspace in reading["workspaces"]], ["unknown", "unknown"])

    def test_missing_source_remains_unknown(self) -> None:
        missing = WorkspaceSpec("missing", "不存在", "original", self.root / "missing", (self.root / "missing",), ())
        reading = collect_workspaces(specs=(missing,), proc_root=self._empty_proc())
        self.assertEqual(reading["workspaces"][0]["source_status"], "unknown")
        self.assertEqual(reading["workspaces"][0]["runtime_status"], "unknown")

    def test_observer_has_no_ros_or_process_launch_imports(self) -> None:
        source = Path(__file__).resolve().parents[1] / "console_agent" / "workspaces.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        imports = {
            alias.name.split(".", 1)[0]
            for node in ast.walk(tree)
            if isinstance(node, (ast.Import, ast.ImportFrom))
            for alias in node.names
        }
        self.assertNotIn("rclpy", imports)
        self.assertNotIn("subprocess", imports)
        self.assertFalse(any(isinstance(node, ast.Attribute) and node.attr == "system" for node in ast.walk(tree)))


if __name__ == "__main__":
    unittest.main()
