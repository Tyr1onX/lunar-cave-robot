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

    def _fake_process(self, command: bytes, environment: bytes, *, comm: str = "python3") -> Path:
        proc = Path(self.temporary.name) / "proc"
        process = proc / "412"
        process.mkdir(parents=True)
        (process / "comm").write_text(f"{comm}\n", encoding="utf-8")
        (process / "cmdline").write_bytes(command)
        (process / "environ").write_bytes(environment)
        (process / "cwd").symlink_to(self.root, target_is_directory=True)
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
        self.assertEqual(workspace["run_expectation"], "unconfigured")
        self.assertEqual(workspace["modules"][0]["runtime_status"], "unknown")
        self.assertEqual(workspace["modules"][0]["run_expectation"], "unconfigured")

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
        )

        with patch("console_agent.workspaces._read_process_environment", side_effect=PermissionError):
            reading = collect_workspaces(specs=(self.spec,), proc_root=proc)

        self.assertEqual(reading["process_scan"]["status"], "ok")
        self.assertIsNone(reading["process_scan"]["reason"])
        self.assertEqual(reading["workspaces"][0]["runtime_status"], "not_running")

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
