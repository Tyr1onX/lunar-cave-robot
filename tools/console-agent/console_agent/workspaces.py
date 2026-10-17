from __future__ import annotations

import os
import re
import shlex
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


_SKIP_DIRS = {".git", "build", "install", "log", "__pycache__", ".venv", "venv", "SDK", "third_party", "third-party", "vendor"}
_CONFIG_DIRS = {"config", "cfg", "params", "param"}
_INVENTORY_CACHE_SECONDS = 30.0
_GRAPH_MAX_AGE_MS = 5000
_PACKAGE_PATH_PATTERN = re.compile(r"(?:install|lib|share)/([^/]+)/")
_CACHE_LOCK = threading.Lock()
_INVENTORY_CACHE: dict[str, tuple[float, dict[str, Any]]] = {}


@dataclass(frozen=True)
class ModuleSpec:
    id: str
    label: str
    packages: tuple[str, ...]
    interface_topics: tuple[str, ...]


@dataclass(frozen=True)
class WorkspaceSpec:
    id: str
    label: str
    source_kind: str
    root: Path
    source_roots: tuple[Path, ...]
    modules: tuple[ModuleSpec, ...]


def _new_root() -> Path:
    return Path(os.environ.get("LUNAR_CONSOLE_REPO", "/home/tzb/lunar-robot/code")).expanduser()


def _workspace_specs() -> tuple[WorkspaceSpec, ...]:
    repo = _new_root()
    arm = repo / "tzb_ws"
    nav = repo / "Nav2_rea_ws"
    return (
        WorkspaceSpec(
            id="new-engineering",
            label="新版工程",
            source_kind="new",
            root=repo,
            source_roots=(arm, nav),
            modules=(
                ModuleSpec("camera", "相机", ("orbbec_camera",), ("/camera/color/image_raw", "/camera/depth/image_rect_raw", "/camera/depth/color/points", "/camera/color/camera_info")),
                ModuleSpec("yolo", "YOLO", ("yolo_node",), ("/yolo/detections", "/yolo/annotated_image")),
                ModuleSpec("pose", "位姿估计", ("pose_estimator",), ("/pose_estimator/object_pose",)),
                ModuleSpec("arm", "机械臂", ("lite_motion_planner", "rm_serial_driver"), ("/serial/arm_joint_state", "/tzb_catch/arm_joint_command")),
                ModuleSpec("lidar", "雷达与定位", ("livox_ros_driver2", "point_lio", "fast_lio", "dlio"), ("/livox/lidar", "/imu/data", "/terrain_map")),
                ModuleSpec("navigation", "导航", ("bringup", "rm_navigation"), ("/scan", "/map", "/cmd_vel")),
                ModuleSpec("chassis", "底盘", ("chassis_serial_bridge",), ("/cmd_vel", "/chassis/serial_link_status")),
            ),
        ),
        WorkspaceSpec(
            id="original-arm-vision",
            label="原始机械臂与视觉工程",
            source_kind="original",
            root=Path("/home/tzb/tzb_ws"),
            source_roots=(Path("/home/tzb/tzb_ws"),),
            modules=(
                ModuleSpec("camera", "相机", ("orbbec_camera",), ("/camera/color/image_raw", "/camera/depth/image_rect_raw", "/camera/depth/color/points", "/camera/color/camera_info")),
                ModuleSpec("yolo", "YOLO", ("yolo_node",), ("/camera/color/image_raw", "/yolo/detections", "/yolo/annotated_image")),
                ModuleSpec("pose", "位姿估计", ("pose_estimator",), ("/camera/depth/color/points", "/camera/color/camera_info", "/yolo/detections", "/pose_estimator/object_pose")),
                ModuleSpec("arm", "机械臂", ("lite_motion_planner", "rm_serial_driver"), ("/serial/arm_joint_state", "/tzb_catch/arm_joint_command", "/tzb_catch/gripper_command")),
            ),
        ),
        WorkspaceSpec(
            id="original-navigation",
            label="原始导航与雷达工程",
            source_kind="original",
            root=Path("/home/tzb/Nav2_rea_ws"),
            source_roots=(Path("/home/tzb/Nav2_rea_ws"),),
            modules=(
                ModuleSpec("lidar", "雷达与定位", ("livox_ros_driver2", "point_lio", "fast_lio", "dlio"), ("/livox/lidar", "/imu/data", "/terrain_map")),
                ModuleSpec("navigation", "Nav2 导航", ("bringup", "rm_navigation", "behavior_ext_plugins"), ("/scan", "/map", "/cmd_vel", "/cmd_vel_chassis")),
                ModuleSpec("chassis", "底盘", ("rm_serial_driver",), ("/cmd_vel_chassis",)),
            ),
        ),
    )


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _walk_source(source_roots: tuple[Path, ...]):
    def fail_walk(error: OSError) -> None:
        raise error

    for source_root in source_roots:
        if not source_root.is_dir():
            continue
        scan_root = source_root / "src" if (source_root / "src").is_dir() else source_root
        for current, directories, files in os.walk(scan_root, followlinks=False, onerror=fail_walk):
            directories[:] = sorted(
                name for name in directories
                if name not in _SKIP_DIRS and not (Path(current) / name).is_symlink()
            )
            for name in files:
                path = Path(current) / name
                if not path.is_symlink():
                    yield source_root, path


def _relative(path: Path, workspace_root: Path) -> str:
    try:
        return path.relative_to(workspace_root).as_posix()
    except ValueError:
        return str(path)


def _collect_source_inventory(spec: WorkspaceSpec) -> dict[str, Any]:
    cache_key = str(spec.root)
    now = time.monotonic()
    with _CACHE_LOCK:
        cached = _INVENTORY_CACHE.get(cache_key)
        if cached and now - cached[0] < _INVENTORY_CACHE_SECONDS:
            return dict(cached[1])

    observed_at = _utc_now()
    try:
        present_roots = [root for root in spec.source_roots if root.is_dir()]
        if not present_roots:
            result = {
                "status": "unknown",
                "observed_at": observed_at,
                "package_count": 0,
                "packages": [],
                "launch_count": 0,
                "launch_files": [],
                "config_count": 0,
                "config_files": [],
                "reason": "configured source workspace path is not present",
            }
        else:
            packages: list[dict[str, str]] = []
            launch_files: list[str] = []
            config_files: list[str] = []
            package_count = launch_count = config_count = 0
            for _source_root, path in _walk_source(spec.source_roots):
                relative = _relative(path, spec.root)
                if path.name == "package.xml":
                    package_count += 1
                    try:
                        name = ET.parse(path).getroot().findtext("name")
                    except (ET.ParseError, OSError):
                        name = None
                    if name:
                        packages.append({"name": name.strip(), "path": relative})
                if "launch" in {part.lower() for part in path.parts} and path.suffix.lower() in {".py", ".xml", ".launch"}:
                    launch_count += 1
                    launch_files.append(relative)
                if path.suffix.lower() in {".yaml", ".yml"} and path.parent.name.lower() in _CONFIG_DIRS:
                    config_count += 1
                    config_files.append(relative)
            result = {
                "status": "present",
                "observed_at": observed_at,
                "package_count": package_count,
                "packages": sorted(packages, key=lambda item: (item["name"], item["path"]))[:256],
                "launch_count": launch_count,
                "launch_files": sorted(launch_files)[:256],
                "config_count": config_count,
                "config_files": sorted(config_files)[:256],
                "reason": None,
            }
    except OSError as exc:
        result = {
            "status": "unknown",
            "observed_at": observed_at,
            "package_count": 0,
            "packages": [],
            "launch_count": 0,
            "launch_files": [],
            "config_count": 0,
            "config_files": [],
            "reason": f"source inventory could not be read: {type(exc).__name__}",
        }

    with _CACHE_LOCK:
        _INVENTORY_CACHE[cache_key] = (time.monotonic(), dict(result))
    return result


def _read_process_environment(path: Path) -> dict[str, str]:
    allowlisted = {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "ROS_PACKAGE_PATH", "ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "LUNAR_CONSOLE_RUN_MODE"}
    values: dict[str, str] = {}
    try:
        entries = path.read_bytes().split(b"\0")
    except FileNotFoundError:
        return values
    for entry in entries:
        key, separator, value = entry.partition(b"=")
        if separator and key.decode("ascii", errors="ignore") in allowlisted:
            values[key.decode("ascii", errors="ignore")] = value.decode("utf-8", errors="replace")
    return values


def _under(path: str, root: Path) -> bool:
    try:
        Path(path).resolve().relative_to(root.resolve())
        return True
    except (OSError, ValueError, RuntimeError):
        return False


def _package_from_text(text: str, known_packages: set[str]) -> str | None:
    for match in _PACKAGE_PATH_PATTERN.finditer(text):
        if match.group(1) in known_packages:
            return match.group(1)
    for token in re.split(r"[\s/=:,]+", text):
        if token in known_packages:
            return token
    return None


def _is_known_session_helper(comm: str, cmdline: str) -> bool:
    """Exclude only audited systemd/SSH session wrappers from ROS probing."""
    if Path(comm).name == "sshd":
        return cmdline.startswith("sshd:") and (len(cmdline) == 5 or cmdline[5].isspace())
    try:
        tokens = shlex.split(cmdline)
    except ValueError:
        tokens = cmdline.split()
    return (
        Path(comm).name == "sd-pam"
        and bool(tokens)
        and Path(tokens[0]).name == "systemd"
        and "--user" in tokens
    )


def _process_scan(specs: tuple[WorkspaceSpec, ...], proc_root: Path) -> tuple[dict[str, list[dict[str, Any]]], str, str | None]:
    found: dict[str, list[dict[str, Any]]] = {spec.id: [] for spec in specs}
    observed_at = _utc_now()
    scan_status = "ok"
    scan_reason: str | None = None
    try:
        entries = list(proc_root.iterdir())
    except OSError as exc:
        return found, "unknown", f"process table could not be read: {type(exc).__name__}"

    current_uid = os.getuid()
    package_sets = {
        spec.id: {item["name"] for item in _collect_source_inventory(spec).get("packages", [])}
        for spec in specs
    }
    known_packages = set().union(*package_sets.values()) if package_sets else set()
    ambiguous_process = False
    for process_dir in entries:
        if not process_dir.name.isdigit():
            continue
        try:
            metadata = process_dir.stat()
            if metadata.st_uid != current_uid:
                continue
            comm = (process_dir / "comm").read_text(encoding="utf-8", errors="replace").strip()[:64]
            raw_cmdline = (process_dir / "cmdline").read_bytes()
            cmdline = raw_cmdline.replace(b"\0", b" ").decode("utf-8", errors="replace").strip()
        except FileNotFoundError:
            continue
        except PermissionError:
            scan_status = "partial"
            scan_reason = "部分当前 UID 进程身份信息不可读"
            continue
        except OSError:
            continue

        # Do not use an executable-name allowlist here: custom ROS executables
        # may have arbitrary names and may expose their workspace only through
        # cwd or the allowlisted overlay environment. The one excluded helper
        # is the audited systemd session process whose metadata is commonly
        # non-dumpable on the target NUC.
        if _is_known_session_helper(comm, cmdline):
            continue

        environment_readable = True
        try:
            environment = _read_process_environment(process_dir / "environ")
        except FileNotFoundError:
            continue
        except PermissionError:
            environment = {}
            environment_readable = False
            scan_status = "partial"
            scan_reason = "部分当前 UID 进程的 allowlisted 环境字段不可读"
        try:
            cwd = str((process_dir / "cwd").resolve())
        except FileNotFoundError:
            continue
        except PermissionError:
            cwd = ""
            scan_status = "partial"
            scan_reason = "部分当前 UID 进程的工作目录信息不可读"
        except OSError:
            cwd = ""
            scan_status = "partial"
            scan_reason = "部分当前 UID 进程的工作目录信息不可读"
        try:
            executable = str((process_dir / "exe").resolve())
        except FileNotFoundError:
            continue
        except PermissionError:
            executable = ""
            scan_status = "partial"
            scan_reason = "部分当前 UID 进程的可执行文件信息不可读"
        except OSError:
            executable = ""
            scan_status = "partial"
            scan_reason = "部分当前 UID 进程的可执行文件信息不可读"

        candidates: list[tuple[WorkspaceSpec, list[str]]] = []
        for spec in specs:
            roots = tuple(root.resolve() for root in spec.source_roots)
            evidence: list[str] = []
            if any(cwd and _under(cwd, root) for root in roots):
                evidence.append("working_directory")
            if any(executable and _under(executable, root) for root in roots):
                evidence.append("executable_path")
            if any(str(root) in cmdline for root in roots):
                evidence.append("command_path")
            for variable in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "ROS_PACKAGE_PATH"):
                value = environment.get(variable, "")
                if any(_under(entry, root) for entry in value.split(os.pathsep) if entry for root in roots):
                    evidence.append("overlay_environment")
                    break
            if not evidence:
                continue
            candidates.append((spec, evidence))

        if not candidates:
            continue

        package = _package_from_text(cmdline, known_packages)
        try:
            command_tokens = shlex.split(cmdline)
        except ValueError:
            command_tokens = cmdline.split()
        command_basenames = {Path(token).name for token in command_tokens}
        ros_command = (
            ("ros2" in command_basenames and bool({"launch", "run"} & set(command_tokens)))
            or bool({"component_container", "component_container_mt", "ros2_control_node", "robot_state_publisher", "rviz2"} & command_basenames)
        )
        path_package = _package_from_text(" ".join((cmdline, executable)), known_packages)
        if package is None:
            package = path_package
        maps = ""
        if package is None and not ros_command:
            # A sourced terminal is not a ROS node merely because its
            # environment contains a workspace overlay.
            try:
                maps = (process_dir / "maps").read_text(encoding="utf-8", errors="replace")
            except PermissionError:
                maps = ""
                scan_status = "partial"
                scan_reason = "部分工作区候选进程的只读映射信息不可读"
            except OSError:
                maps = ""
            package = _package_from_text(maps, known_packages)
            if package is None and not any(token in maps for token in ("librclcpp", "librcl", "rclpy")):
                continue

        explicit = [
            candidate for candidate in candidates
            if set(candidate[1]) & {"working_directory", "executable_path", "command_path"}
        ]
        if len(explicit) == 1:
            selected = explicit
        elif len(explicit) > 1:
            ambiguous_process = True
            continue
        elif package is not None:
            selected = [candidate for candidate in candidates if package in package_sets[candidate[0].id]]
            if len(selected) != 1:
                ambiguous_process = True
                continue
        elif len(candidates) == 1:
            selected = candidates
        else:
            ambiguous_process = True
            continue

        domain_raw = environment.get("ROS_DOMAIN_ID")
        try:
            domain_id = int(domain_raw) if domain_raw is not None and 0 <= int(domain_raw) <= 232 else None
            domain_evidence = "observed" if domain_id is not None else ("invalid" if domain_raw is not None else "unset")
        except ValueError:
            domain_id = None
            domain_evidence = "invalid"
        if not environment_readable:
            domain_evidence = "unavailable"
        local_only = environment.get("ROS_LOCALHOST_ONLY")
        raw_mode = environment.get("LUNAR_CONSOLE_RUN_MODE", "").strip().lower()
        run_mode = raw_mode if raw_mode in {"hardware", "simulation", "offline", "read-only"} else "unknown"
        run_mode_evidence = "explicit_environment" if run_mode != "unknown" else "unknown"
        for spec, evidence in selected:
            found[spec.id].append({
                "pid": int(process_dir.name),
                "process_name": comm or "未知",
                "package": package,
                "domain_id": domain_id,
                "domain_evidence": domain_evidence,
                "localhost_only": local_only if local_only in {"0", "1"} else None,
                "run_mode": run_mode,
                "run_mode_evidence": run_mode_evidence,
                "matched_by": sorted(set(evidence)),
                "observed_at": observed_at,
            })

    for processes in found.values():
        processes.sort(key=lambda item: (item["package"] or "", item["pid"]))
    reason = scan_reason
    if ambiguous_process:
        reason = "部分 ROS 进程引用多个工作区，无法安全归属" if reason is None else f"{reason}；另有部分 ROS 进程无法安全归属"
    if ambiguous_process and scan_status == "ok":
        scan_status = "partial"
    return found, scan_status, reason


def _graph_relationship(
    processes: list[dict[str, Any]],
    graph: dict[str, Any],
    process_scan_status: str,
) -> tuple[str, str]:
    if not processes:
        return "unknown", "未发现关联 ROS 进程；没有可用于归属工作区的运行时 graph。"
    if process_scan_status != "ok":
        return "unknown", "Agent UID 进程扫描不完整；工作区进程与 graph 的关系保持未知。"
    domain_ids = [process["domain_id"] for process in processes]
    if any(domain is None for domain in domain_ids):
        return "unknown", "至少一个进程未提供可解析的 ROS_DOMAIN_ID。"
    age = graph.get("graph_age_ms")
    if graph.get("status") != "ok" or not isinstance(age, int) or age < 0 or age > _GRAPH_MAX_AGE_MS:
        return "unknown", "Agent ROS graph 未处于可验证的新鲜状态；下游状态保持未知。"
    graph_domain = graph.get("domain_id")
    if not isinstance(graph_domain, int):
        return "unknown", "Agent ROS graph 未提供实际 Domain。"
    domains = set(domain_ids)
    if domains == {graph_domain}:
        return "shared_domain", f"进程 Domain 与 Agent 当前观测 Domain {graph_domain} 相同；节点/Topic 仍需以 graph 实际发现为准。"
    if graph_domain in domains:
        return "partially_shared", f"部分进程位于 Agent Domain {graph_domain}，其它 Domain 未由该 graph 观测。"
    return "different_domain", f"工作区进程 Domain {sorted(domains)} 与 Agent Domain {graph_domain} 不同；该工作区运行状态未知，不据此判故障。"


def _execution_state(processes: list[dict[str, Any]], scan_status: str, runtime_status: str) -> str:
    """Describe observed process/mode evidence without asserting device health."""
    if runtime_status == "not_running":
        return "not_running"
    if runtime_status != "running" or scan_status != "ok" or not processes:
        return "unknown"
    if any(process.get("run_mode_evidence") != "explicit_environment" for process in processes):
        return "unknown"
    modes = {process.get("run_mode", "unknown") for process in processes}
    if modes == {"simulation"}:
        return "simulation_running"
    if modes == {"hardware"}:
        return "hardware_declared_running"
    return "unknown"


def collect_workspaces(
    graph: dict[str, Any] | None = None,
    *,
    specs: tuple[WorkspaceSpec, ...] | None = None,
    proc_root: Path = Path("/proc"),
) -> dict[str, Any]:
    """Read workspace source metadata and current-UID process evidence only."""
    selected_specs = specs if specs is not None else _workspace_specs()
    observed_at = _utc_now()
    active_graph = graph or {"status": "unknown"}
    processes_by_workspace, scan_status, scan_reason = _process_scan(selected_specs, proc_root)
    readings: list[dict[str, Any]] = []

    for spec in selected_specs:
        source = _collect_source_inventory(spec)
        processes = processes_by_workspace.get(spec.id, [])
        if source["status"] != "present":
            runtime_status = "unknown"
        elif processes:
            runtime_status = "running"
        elif scan_status == "ok":
            runtime_status = "not_running"
        else:
            runtime_status = "unknown"

        relationship, relationship_reason = _graph_relationship(processes, active_graph, scan_status)
        packages = {item["name"] for item in source.get("packages", [])}
        modules = []
        for module in spec.modules:
            module_source = "present" if any(name in packages for name in module.packages) else "unknown"
            module_processes = [process for process in processes if process.get("package") in module.packages]
            if module_processes:
                module_runtime = "running"
            elif module_source == "present" and scan_status == "ok" and not any(process.get("package") is None for process in processes):
                module_runtime = "not_running"
            else:
                module_runtime = "unknown"
            module_execution_state = _execution_state(module_processes, scan_status, module_runtime)
            graph_topics = []
            if relationship == "shared_domain":
                observed_names = {
                    topic.get("name") for topic in (active_graph.get("topics") or [])
                    if isinstance(topic, dict) and isinstance(topic.get("name"), str)
                }
                graph_topics = [topic for topic in module.interface_topics if topic in observed_names]
            modules.append({
                "id": module.id,
                "label": module.label,
                "packages": list(module.packages),
                "source_status": module_source,
                "runtime_status": module_runtime,
                "execution_state": module_execution_state,
                "run_expectation": "unconfigured",
                "interface_topics": list(module.interface_topics),
                "observed_interface_topics": graph_topics,
                "reason": "源码接口候选不代表运行时存在；未配置运行预期，不据此产生故障事件。",
            })

        domain_ids = sorted({process["domain_id"] for process in processes if process["domain_id"] is not None})
        run_modes = sorted({process["run_mode"] for process in processes})
        run_mode = run_modes[0] if len(run_modes) == 1 else ("mixed" if run_modes else "unknown")
        readings.append({
            "id": spec.id,
            "label": spec.label,
            "source_kind": spec.source_kind,
            "root": str(spec.root),
            "source_status": source["status"],
            "source_observed_at": source["observed_at"],
            "package_count": source["package_count"],
            "packages": source["packages"],
            "launch_count": source["launch_count"],
            "launch_files": source["launch_files"],
            "config_count": source["config_count"],
            "config_files": source["config_files"],
            "runtime_status": runtime_status,
            "runtime_scope": "Agent UID 可见进程；/proc 与工作区路径只读检查",
            "runtime_observed_at": observed_at,
            "process_count": len(processes),
            "processes": processes,
            "domain_ids": domain_ids,
            "run_mode": run_mode,
            "execution_state": _execution_state(processes, scan_status, runtime_status),
            "run_expectation": "unconfigured",
            "graph_domain_id": active_graph.get("domain_id"),
            "graph_observed_at": active_graph.get("observed_at"),
            "graph_relationship": relationship,
            "graph_reason": relationship_reason,
            "modules": modules,
            "reason": source.get("reason") or scan_reason,
        })

    return {
        "status": "ok" if scan_status != "unknown" else "unknown",
        "source": "nuc-read-only-workspace-and-process-observer",
        "collected_at": observed_at,
        "process_scan": {
            "status": scan_status,
            "scope": "Agent UID; process identity and workspace references; audited systemd session helper excluded",
            "source": str(proc_root),
            "observed_at": observed_at,
            "reason": scan_reason,
        },
        "graph_source": {
            "domain_id": active_graph.get("domain_id"),
            "observed_at": active_graph.get("observed_at"),
            "graph_age_ms": active_graph.get("graph_age_ms"),
            "source": "existing Agent ROS graph observer; no additional ROS participant is started",
        },
        "workspaces": readings,
    }
