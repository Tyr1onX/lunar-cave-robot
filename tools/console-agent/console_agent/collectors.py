from __future__ import annotations

import glob
import json
import os
import re
import shutil
import subprocess
import threading
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .link_telemetry import TELEMETRY_STREAMS, LinkTelemetryObserver
from .vision import STREAMS, VisionObserver
from .workspaces import collect_workspaces

SCHEMA_VERSION = 1
_AGENT_NODE_NAME = "lunar_console_graph_probe"
_CPU_LOCK = threading.Lock()
_ROS_OBSERVER: "ROSGraphObserver | None" = None
_SHA_RE = re.compile(r"^[0-9a-fA-F]{7,40}$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _unknown(reason: str) -> dict[str, Any]:
    return {"status": "unknown", "reason": reason}


def _expected_ros_domain_id() -> tuple[int | None, str | None]:
    raw = os.environ.get("LUNAR_CONSOLE_EXPECTED_ROS_DOMAIN_ID", "").strip()
    if not raw:
        return None, None
    try:
        domain_id = int(raw)
    except ValueError:
        return None, "LUNAR_CONSOLE_EXPECTED_ROS_DOMAIN_ID must be an integer from 0 to 232"
    if not 0 <= domain_id <= 232:
        return None, "LUNAR_CONSOLE_EXPECTED_ROS_DOMAIN_ID must be an integer from 0 to 232"
    return domain_id, None


def _expected_ros_nodes() -> tuple[list[str], str | None]:
    raw = os.environ.get("LUNAR_CONSOLE_EXPECTED_ROS_NODES", "").strip()
    if not raw:
        return [], None
    expected: set[str] = set()
    for item in raw.split(","):
        name = item.strip()
        if not name:
            continue
        full_name = "/" + name.strip("/")
        if not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*", full_name):
            return [], "LUNAR_CONSOLE_EXPECTED_ROS_NODES must contain comma-separated ROS node names"
        expected.add(full_name)
    return sorted(expected), None


def _read_cpu_counters() -> tuple[int, int] | None:
    try:
        fields = Path("/proc/stat").read_text(encoding="ascii").splitlines()[0].split()
        values = [int(value) for value in fields[1:]]
        idle = values[3] + (values[4] if len(values) > 4 else 0)
        return sum(values), idle
    except (OSError, ValueError, IndexError):
        return None


def collect_cpu() -> dict[str, Any]:
    with _CPU_LOCK:
        before = _read_cpu_counters()
        if before is None:
            return _unknown("/proc/stat unavailable")
        time.sleep(0.12)
        after = _read_cpu_counters()
    if after is None:
        return _unknown("/proc/stat unavailable")
    total = after[0] - before[0]
    idle = after[1] - before[1]
    if total <= 0 or idle < 0:
        return _unknown("CPU counters did not advance")
    return {"status": "ok", "percent": round(max(0.0, min(100.0, 100.0 * (total - idle) / total)), 1), "source": "/proc/stat"}


def collect_memory() -> dict[str, Any]:
    try:
        values: dict[str, int] = {}
        for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
            key, raw = line.split(":", 1)
            parts = raw.split()
            if parts and key in {"MemTotal", "MemAvailable"}:
                values[key] = int(parts[0]) * 1024
        total = values["MemTotal"]
        available = values["MemAvailable"]
        if total <= 0 or available < 0:
            raise ValueError("invalid memory counters")
        used = total - min(available, total)
        return {"status": "ok", "total_bytes": total, "available_bytes": available, "used_percent": round(100.0 * used / total, 1), "source": "/proc/meminfo"}
    except (OSError, ValueError, KeyError):
        return _unknown("memory counters unavailable")


def collect_disk() -> dict[str, Any]:
    try:
        usage = shutil.disk_usage("/")
        used = usage.total - usage.free
        return {"status": "ok", "path": "/", "total_bytes": usage.total, "used_bytes": used, "free_bytes": usage.free, "used_percent": round(100.0 * used / usage.total, 1)}
    except OSError:
        return _unknown("root filesystem usage unavailable")


def collect_temperature() -> dict[str, Any]:
    readings: list[dict[str, Any]] = []
    thermal_paths = sorted(glob.glob("/sys/class/thermal/thermal_zone*/temp"))
    for path in thermal_paths:
        try:
            raw = int(Path(path).read_text(encoding="ascii").strip())
            celsius = raw / 1000.0 if abs(raw) > 1000 else float(raw)
            if not -40.0 <= celsius <= 200.0:
                continue
            label_path = Path(path).parent / "type"
            label = label_path.read_text(encoding="utf-8").strip() if label_path.exists() else Path(path).parent.name
            readings.append({"label": label or Path(path).parent.name, "celsius": round(celsius, 1), "source": path})
        except (OSError, ValueError):
            continue
    if not readings:
        for path in sorted(glob.glob("/sys/class/hwmon/hwmon*/temp*_input")):
            try:
                raw = int(Path(path).read_text(encoding="ascii").strip())
                celsius = raw / 1000.0 if abs(raw) > 1000 else float(raw)
                if not -40.0 <= celsius <= 200.0:
                    continue
                sensor = re.search(r"(temp[0-9]+)_input$", path)
                directory = Path(path).parent
                label_path = directory / f"{sensor.group(1)}_label" if sensor else directory / "name"
                chip_name = directory / "name"
                label = label_path.read_text(encoding="utf-8").strip() if label_path.exists() else ""
                if not label and chip_name.exists():
                    label = chip_name.read_text(encoding="utf-8").strip()
                readings.append({"label": label or directory.name, "celsius": round(celsius, 1), "source": path})
            except (OSError, ValueError):
                continue
    if readings:
        return {"status": "ok", "readings": readings}
    return _unknown("no readable thermal sensor exported by the host")


def _git_read(repo: Path, *args: str) -> str | None:
    try:
        result = subprocess.run(
            ["/usr/bin/git", "-c", "core.fsmonitor=false", "-c", "core.untrackedCache=false", "-c", "core.hooksPath=/dev/null", "-C", str(repo), *args],
            check=True,
            capture_output=True,
            text=True,
            timeout=2.0,
            env={
                "PATH": "/usr/bin:/bin",
                "HOME": str(Path.home()),
                "GIT_OPTIONAL_LOCKS": "0",
                "GIT_CONFIG_NOSYSTEM": "1",
                "GIT_CONFIG_GLOBAL": "/dev/null",
            },
        )
        return result.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def collect_git() -> dict[str, Any]:
    repo = Path(os.environ.get("LUNAR_CONSOLE_REPO", "~/lunar-robot/code")).expanduser()
    head = _git_read(repo, "rev-parse", "HEAD")
    if not head:
        return _unknown("repository HEAD is unavailable at the configured repository path")
    branch = _git_read(repo, "branch", "--show-current") or "detached"
    porcelain = _git_read(repo, "status", "--porcelain=v1")
    if porcelain is None:
        return _unknown("repository status could not be read")
    dirty_files = sum(1 for line in porcelain.splitlines() if line.strip())
    return {"status": "ok", "head_sha": head, "branch": branch, "dirty": dirty_files > 0, "dirty_files": dirty_files, "repository_path": str(repo)}


def _state_root() -> Path:
    configured = os.environ.get("XDG_STATE_HOME")
    if configured:
        base = Path(configured).expanduser()
    else:
        repo = Path(os.environ.get("LUNAR_CONSOLE_REPO", "~/lunar-robot/code")).expanduser()
        project_root = repo.parent if repo.name == "code" else None
        base = project_root / "tests" / "state" if project_root is not None else Path.home() / ".local" / "state"
    return base / "lunar-cave-robot"


def collect_deployment() -> dict[str, Any]:
    path = _state_root() / "deployed-sha"
    try:
        raw = path.read_text(encoding="utf-8").splitlines()[0].strip()
    except (OSError, IndexError):
        return _unknown("deployed-sha has not been recorded by a deployment owner")
    if not _SHA_RE.fullmatch(raw):
        return _unknown("deployed-sha file does not contain a valid Git SHA")
    return {"status": "ok", "sha": raw.lower(), "source": str(path)}


def _yaml_scalar(raw: str) -> str:
    value = raw.strip()
    if len(value) >= 2 and value[0] == value[-1] and value[0] in {"'", '"'}:
        value = value[1:-1]
    return value


def _parse_simple_yaml(path: Path) -> dict[str, str]:
    result: dict[str, str] = {}
    for line in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if not line or line.lstrip().startswith("#") or line[0].isspace() or ":" not in line:
            continue
        key, value = line.split(":", 1)
        key = key.strip()
        if re.fullmatch(r"[a-zA-Z0-9_-]+", key):
            result[key] = _yaml_scalar(value)
    return result


def collect_tests() -> dict[str, Any]:
    tests_dir = _state_root() / "tests"
    files: list[tuple[float, Path]] = []
    try:
        for path in tests_dir.glob("*.yaml"):
            try:
                if path.is_symlink():
                    continue
                metadata = path.stat()
                if path.is_file() and metadata.st_size <= 65536:
                    files.append((metadata.st_mtime, path))
            except OSError:
                continue
    except OSError:
        pass
    records: list[dict[str, Any]] = []
    for _, path in sorted(files, key=lambda item: item[0], reverse=True)[:100]:
        try:
            fields = _parse_simple_yaml(path)
            records.append({
                "id": path.stem,
                "timestamp": fields.get("timestamp"),
                "result": fields.get("result", "unknown"),
                "test": fields.get("test", path.stem),
                "scope": fields.get("scope", "范围未记录"),
                "sha": fields.get("sha"),
                "branch": fields.get("branch"),
                "artifact": fields.get("artifact"),
                "record_path": str(path),
            })
        except OSError:
            continue
    return {"status": "ok", "records": records, "source": str(tests_dir)}


class ROSGraphObserver:
    """One ROS graph reader with fixed, read-only image and link subscriptions."""

    def __init__(self) -> None:
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._snapshot: dict[str, Any] = {
            "status": "unknown",
            "domain_id": self._domain_id(),
            "nodes": [],
            "topics": [],
            "reason": "ROS graph observer is starting",
        }
        self._last_observed_at: str | None = None
        self._last_observed_mono: float | None = None
        self._vision = VisionObserver()
        self._link_telemetry = LinkTelemetryObserver()
        self._thread = threading.Thread(target=self._run, name="ros-graph-observer", daemon=True)
        self._thread.start()

    @staticmethod
    def _domain_id() -> int | None:
        try:
            return int(os.environ.get("ROS_DOMAIN_ID", "0"))
        except ValueError:
            return None

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            result = dict(self._snapshot)
            observed_at = self._last_observed_at
            observed_mono = self._last_observed_mono
        expected_domain_id, expected_domain_error = _expected_ros_domain_id()
        expected_nodes, expected_nodes_error = _expected_ros_nodes()
        result.update({
            "observed_at": observed_at,
            "graph_age_ms": None if observed_mono is None else round(max(0.0, time.monotonic() - observed_mono) * 1000),
            "expected_domain_id": expected_domain_id,
            "expected_domain_error": expected_domain_error,
            "expected_nodes": expected_nodes,
            "expected_nodes_error": expected_nodes_error,
        })
        return result

    def vision_snapshot(self) -> dict[str, Any]:
        return self._vision.snapshot()

    def link_telemetry_snapshot(self) -> dict[str, Any]:
        return self._link_telemetry.snapshot()

    def vision_frame_png(self, stream_id: str) -> tuple[bytes | None, str | None, int | None]:
        return self._vision.frame_png(stream_id)

    def stop(self) -> None:
        self._stop.set()
        self._thread.join(timeout=2.0)

    def _store_snapshot(self, value: dict[str, Any]) -> None:
        observed_at = utc_now() if value.get("status") == "ok" else None
        observed_mono = time.monotonic() if observed_at is not None else None
        with self._lock:
            self._snapshot = value
            if observed_at is not None:
                self._last_observed_at = observed_at
                self._last_observed_mono = observed_mono

    def _run(self) -> None:
        context = None
        node = None
        executor = None
        try:
            import rclpy
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import (
                QoSDurabilityPolicy,
                QoSHistoryPolicy,
                QoSProfile,
                QoSReliabilityPolicy,
                qos_profile_sensor_data,
            )
            from sensor_msgs.msg import Image

            context = rclpy.context.Context()
            rclpy.init(context=context)
            node = rclpy.create_node(
                _AGENT_NODE_NAME,
                context=context,
                start_parameter_services=False,
                enable_rosout=False,
            )
            _disable_default_parameter_event_publisher(node)
            for stream_id, definition in STREAMS.items():
                node.create_subscription(
                    Image,
                    definition["topic"],
                    lambda message, selected=stream_id: self._vision.receive(selected, message),
                    qos_profile_sensor_data,
                )
            message_types = _load_link_telemetry_message_types()
            self._link_telemetry.set_available_message_types(set(message_types))
            telemetry_qos = QoSProfile(
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.VOLATILE,
            )
            telemetry_subscriptions: dict[str, tuple[str, Any]] = {}
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            last_graph_read = 0.0
            while not self._stop.is_set():
                executor.spin_once(timeout_sec=0.02)
                now = time.monotonic()
                if now - last_graph_read < 1.0:
                    continue
                last_graph_read = now
                nodes = [
                    {"name": name, "namespace": namespace}
                    for name, namespace in node.get_node_names_and_namespaces()
                    if name != _AGENT_NODE_NAME
                ]
                topics = [
                    {"name": name, "types": sorted(types)}
                    for name, types in node.get_topic_names_and_types()
                    if name not in {entry["topic"] for entry in STREAMS.values()}
                    or bool(node.get_publishers_info_by_topic(name))
                ]
                for stream_id, definition in STREAMS.items():
                    publishers = node.get_publishers_info_by_topic(definition["topic"])
                    self._vision.update_publishers(
                        stream_id,
                        [publisher.topic_type for publisher in publishers],
                        len(publishers),
                    )
                for stream_id, definition in TELEMETRY_STREAMS.items():
                    try:
                        publishers = node.get_publishers_info_by_topic(definition["topic"])
                    except Exception:
                        self._link_telemetry.update_publishers(stream_id, None)
                        continue
                    endpoints = [
                        {
                            "name": str(getattr(publisher, "node_name", "")),
                            "namespace": str(getattr(publisher, "node_namespace", "")),
                            "topic_type": str(getattr(publisher, "topic_type", "")),
                        }
                        for publisher in publishers
                    ]
                    self._link_telemetry.update_publishers(stream_id, endpoints)
                    self._sync_link_telemetry_subscription(
                        node,
                        stream_id,
                        endpoints,
                        message_types,
                        telemetry_qos,
                        telemetry_subscriptions,
                        self._link_telemetry,
                    )
                self._vision.graph_available()
                expected_domain_id, expected_domain_error = _expected_ros_domain_id()
                self._link_telemetry.graph_available(
                    domain_id=self._domain_id(),
                    observed_at=utc_now(),
                    expected_domain_id=expected_domain_id,
                    expected_domain_error=expected_domain_error,
                )
                self._store_snapshot({
                    "status": "ok",
                    "domain_id": self._domain_id(),
                    "nodes": sorted(nodes, key=lambda item: (item["namespace"], item["name"])),
                    "topics": sorted(topics, key=lambda item: item["name"]),
                    "discovery": "rclpy runtime graph · read-only observer",
                })
        except Exception as exc:
            self._vision.graph_unavailable(f"ROS image observer unavailable: {type(exc).__name__}: {exc}")
            self._link_telemetry.graph_unavailable(f"ROS graph observer unavailable: {type(exc).__name__}: {exc}")
            self._store_snapshot({
                "status": "unknown",
                "domain_id": self._domain_id(),
                "nodes": [],
                "topics": [],
                "reason": f"ROS graph unavailable: {type(exc).__name__}: {exc}",
            })
        finally:
            if executor is not None and node is not None:
                try:
                    executor.remove_node(node)
                except Exception:
                    pass
            if node is not None:
                for _message_type, subscription in locals().get("telemetry_subscriptions", {}).values():
                    try:
                        node.destroy_subscription(subscription)
                    except Exception:
                        pass
            if node is not None:
                try:
                    node.destroy_node()
                except Exception:
                    pass
            if context is not None:
                try:
                    import rclpy
                    if context.ok():
                        rclpy.shutdown(context=context)
                except Exception:
                    pass


def _disable_default_parameter_event_publisher(node: Any) -> None:
    """Remove rclpy Humble's implicit writer; this agent never sets parameters."""
    publisher = getattr(node, "_parameter_event_publisher", None)
    if publisher is not None:
        node.destroy_publisher(publisher)
        node._parameter_event_publisher = None


def _load_link_telemetry_message_types() -> dict[str, Any]:
    """Load only fixed ROS message types; missing workspace type support stays observable as unknown."""
    result: dict[str, Any] = {}
    try:
        from sensor_msgs.msg import Imu, LaserScan, PointCloud2

        result.update({
            "sensor_msgs/msg/Imu": Imu,
            "sensor_msgs/msg/LaserScan": LaserScan,
            "sensor_msgs/msg/PointCloud2": PointCloud2,
        })
    except (ImportError, AttributeError):
        pass
    try:
        from nav_msgs.msg import OccupancyGrid, Odometry

        result.update({
            "nav_msgs/msg/OccupancyGrid": OccupancyGrid,
            "nav_msgs/msg/Odometry": Odometry,
        })
    except (ImportError, AttributeError):
        pass
    try:
        from livox_ros_driver2.msg import CustomMsg

        result["livox_ros_driver2/msg/CustomMsg"] = CustomMsg
    except (ImportError, AttributeError):
        pass
    return result


def _sync_link_telemetry_subscription(
    node: Any,
    stream_id: str,
    endpoints: list[dict[str, str]],
    message_types: dict[str, Any],
    qos_profile: Any,
    subscriptions: dict[str, tuple[str, Any]],
    telemetry: LinkTelemetryObserver,
) -> None:
    """Create subscriptions only for one allowlisted type on a fixed topic."""
    definition = TELEMETRY_STREAMS[stream_id]
    topic_types = sorted({endpoint["topic_type"] for endpoint in endpoints if endpoint.get("topic_type")})
    existing = subscriptions.get(stream_id)
    if len(topic_types) != 1 or topic_types[0] not in definition["message_types"]:
        if existing is not None:
            try:
                node.destroy_subscription(existing[1])
            except Exception:
                pass
            subscriptions.pop(stream_id, None)
        reason = "publisher type is missing or ambiguous" if len(topic_types) != 1 else "publisher type is outside the fixed telemetry allowlist"
        # An empty publisher list is a valid observation and does not need an active subscription.
        if not endpoints:
            reason = "no publisher is currently visible in this ROS Domain"
        telemetry.subscription_unavailable(stream_id, None, reason)
        return

    message_type = topic_types[0]
    message_class = message_types.get(message_type)
    if message_class is None:
        if existing is not None:
            try:
                node.destroy_subscription(existing[1])
            except Exception:
                pass
            subscriptions.pop(stream_id, None)
        telemetry.subscription_unavailable(stream_id, message_type, "ROS message type support is not loaded in the Agent environment")
        return
    if existing is not None and existing[0] == message_type:
        telemetry.subscription_ready(stream_id, message_type)
        return
    if existing is not None:
        try:
            node.destroy_subscription(existing[1])
        except Exception:
            pass
        subscriptions.pop(stream_id, None)
    try:
        subscription = node.create_subscription(
            message_class,
            definition["topic"],
            lambda message, selected=stream_id, selected_type=message_type: telemetry.receive(selected, message, selected_type),
            qos_profile,
        )
    except Exception as exc:
        telemetry.subscription_unavailable(stream_id, message_type, f"fixed read-only subscription unavailable: {type(exc).__name__}")
        return
    subscriptions[stream_id] = (message_type, subscription)
    telemetry.subscription_ready(stream_id, message_type)


def start_ros_graph_observer() -> ROSGraphObserver:
    global _ROS_OBSERVER
    observer = ROSGraphObserver()
    _ROS_OBSERVER = observer
    return observer


def collect_ros() -> dict[str, Any]:
    observer = _ROS_OBSERVER
    if observer is None:
        return {
            "status": "unknown",
            "domain_id": ROSGraphObserver._domain_id(),
            "nodes": [],
            "topics": [],
            "reason": "ROS graph observer has not started",
            "observed_at": None,
            "graph_age_ms": None,
            "expected_domain_id": _expected_ros_domain_id()[0],
            "expected_domain_error": _expected_ros_domain_id()[1],
            "expected_nodes": _expected_ros_nodes()[0],
            "expected_nodes_error": _expected_ros_nodes()[1],
        }
    return observer.snapshot()


def collect_vision() -> dict[str, Any]:
    observer = _ROS_OBSERVER
    if observer is not None:
        return observer.vision_snapshot()
    return {
        "status": "unknown",
        "source": "nuc-ros2-read-only-subscriber",
        "collected_at": utc_now(),
        "reason": "ROS graph observer has not started",
        "streams": [
            {
                "id": stream_id,
                "label": definition["label"],
                "topic": definition["topic"],
                "topic_type": None,
                "status": "unknown",
                "reason": "ROS graph observer has not started",
                "publisher_count": 0,
                "width": None,
                "height": None,
                "encoding": None,
                "frame_rate_hz": None,
                "frame_age_ms": None,
                "source_stamp": None,
                "frame_id": None,
                "header_to_agent_ms": None,
                "received_at": None,
                "gap_events": 0,
            }
            for stream_id, definition in STREAMS.items()
        ],
    }


def collect_link_telemetry() -> dict[str, Any]:
    observer = _ROS_OBSERVER
    if observer is not None:
        return observer.link_telemetry_snapshot()
    return LinkTelemetryObserver().snapshot()


def collect_vision_frame(stream_id: str) -> tuple[bytes | None, str | None, int | None]:
    observer = _ROS_OBSERVER
    if observer is None:
        return None, "ROS graph observer has not started", 503
    return observer.vision_frame_png(stream_id)


def collect_snapshot() -> dict[str, Any]:
    ros = collect_ros()
    return {
        "schema_version": SCHEMA_VERSION,
        "collected_at": utc_now(),
        "source": "nuc-read-only-agent",
        "host": {"hostname": os.uname().nodename, "platform": os.uname().machine, "os_release": Path("/etc/os-release").read_text(encoding="utf-8", errors="replace").splitlines() if Path("/etc/os-release").exists() else []},
        "system": {"cpu": collect_cpu(), "memory": collect_memory(), "disk": collect_disk(), "temperature": collect_temperature()},
        "git": collect_git(),
        "deployment": collect_deployment(),
        "ros": ros,
        "workspaces": collect_workspaces(ros),
        "tests": collect_tests(),
        "safety": {"physical_estop": _unknown("no verified physical E-stop input is connected to this read-only agent"), "hardware_health": _unknown("ROS graph presence is not hardware-health evidence"), "control": "disabled"},
    }


def collect_logs(limit: int = 100) -> dict[str, Any]:
    root = Path.home() / ".ros" / "log" / "latest"
    try:
        if not root.is_dir():
            return {"status": "unknown", "reason": "no ROS log directory is available", "entries": [], "source": str(root)}
        files = [path for path in root.rglob("*") if not path.is_symlink() and path.is_file() and path.suffix.lower() in {".log", ".txt"}]
    except OSError:
        return {"status": "unknown", "reason": "ROS log directory cannot be read", "entries": [], "source": str(root)}
    files.sort(key=lambda path: path.stat().st_mtime, reverse=True)
    entries: list[dict[str, Any]] = []
    for path in files[:40]:
        try:
            with path.open("rb") as stream:
                stream.seek(0, os.SEEK_END)
                size = stream.tell()
                stream.seek(max(0, size - 262144))
                lines = stream.read().decode("utf-8", errors="replace").splitlines()[-limit:]
            file_modified_at = datetime.fromtimestamp(path.stat().st_mtime, timezone.utc).isoformat(timespec="seconds").replace("+00:00", "Z")
            for line in lines:
                if not line.strip():
                    continue
                upper = line.upper()
                severity = "error" if "[ERROR]" in upper or " FATAL" in upper else "warning" if "[WARN]" in upper else "info"
                entries.append({"file_modified_at": file_modified_at, "severity": severity, "message": line[-2000:], "source": path.name})
        except OSError:
            continue
    entries.sort(key=lambda item: item["file_modified_at"], reverse=True)
    return {"status": "ok", "entries": entries[:limit], "source": str(root)}
