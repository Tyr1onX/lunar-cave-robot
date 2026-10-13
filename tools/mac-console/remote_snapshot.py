#!/usr/bin/env python3
"""Read-only NUC snapshot sent over SSH by Lunar Robot Console."""

from __future__ import annotations

import json
import importlib.util
import os
import platform
import re
import shutil
import socket
import subprocess
import time
from pathlib import Path


REPO = Path.home() / "lunar-cave-robot"
ROS_SETUP = Path("/opt/ros/humble/setup.bash")
ROS_COMMANDS = {
    "nodes": "ros2 node list --no-daemon",
    "topics": "ros2 topic list --no-daemon",
}
PROJECT_PROCESSES = re.compile(
    r"ros|nav2|livox|orbbec|camera|yolo|openvino|robot|serial|foxglove",
    re.IGNORECASE,
)
SECRET_PATTERN = re.compile(
    r"(?i)\b(token|password|passwd|api[_-]?key|secret)(\s*[:=]\s*)([^\s,;]+)"
)


def read_text(path: Path, default: str = "") -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace").strip()
    except OSError:
        return default


def command(args: list[str], timeout: float = 4.0) -> tuple[int, str, str]:
    try:
        result = subprocess.run(
            args,
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
            env={**os.environ, "GIT_OPTIONAL_LOCKS": "0"},
        )
        return result.returncode, result.stdout.strip(), result.stderr.strip()
    except (OSError, subprocess.TimeoutExpired) as error:
        return 127, "", type(error).__name__


def ros_query(statement: str) -> dict[str, object]:
    script = (
        f"if [ -r {str(ROS_SETUP)!r} ]; then "
        f"source {str(ROS_SETUP)!r} >/dev/null 2>&1 && {statement}; "
        "else exit 127; fi"
    )
    code, stdout, stderr = command(
        ["bash", "--noprofile", "--norc", "-c", script], timeout=9.0
    )
    return {
        "ok": code == 0,
        "items": [line.strip() for line in stdout.splitlines() if line.strip()],
        "error": stderr[-300:] if code else "",
    }


def memory_info() -> dict[str, float | None]:
    values: dict[str, int] = {}
    for line in read_text(Path("/proc/meminfo")).splitlines():
        key, _, rest = line.partition(":")
        match = re.search(r"(\d+)\s+kB", rest)
        if match:
            values[key] = int(match.group(1)) * 1024
    total = values.get("MemTotal")
    available = values.get("MemAvailable")
    return {
        "totalGiB": round(total / (1024**3), 1) if total else None,
        "availableGiB": round(available / (1024**3), 1) if available else None,
        "usedPercent": round((1 - available / total) * 100, 1)
        if total and available is not None
        else None,
    }


def cpu_model() -> str:
    for line in read_text(Path("/proc/cpuinfo")).splitlines():
        if line.lower().startswith(("model name", "hardware")):
            return line.partition(":")[2].strip()
    return platform.processor() or "unknown"


def thermal_zones() -> list[dict[str, object]]:
    zones: list[dict[str, object]] = []
    for folder in sorted(Path("/sys/class/thermal").glob("thermal_zone*")):
        try:
            raw = int((folder / "temp").read_text().strip())
            name = (folder / "type").read_text().strip()
        except (OSError, ValueError):
            continue
        zones.append({"name": name or folder.name, "celsius": round(raw / 1000, 1)})
    return zones


def network_links() -> list[dict[str, str]]:
    links: list[dict[str, str]] = []
    for folder in sorted(Path("/sys/class/net").iterdir()):
        if folder.name == "lo":
            continue
        links.append({"name": folder.name, "state": read_text(folder / "operstate", "unknown")})
    return links


def repository_state() -> dict[str, object]:
    if not (REPO / ".git").exists():
        return {"available": False}
    _, sha, _ = command(["git", "-C", str(REPO), "rev-parse", "HEAD"])
    _, branch, _ = command(["git", "-C", str(REPO), "branch", "--show-current"])
    _, dirty, _ = command(
        ["git", "-C", str(REPO), "status", "--porcelain=v1"]
    )
    return {
        "available": bool(sha),
        "sha": sha,
        "branch": branch or "detached",
        "dirtyCount": len(dirty.splitlines()) if dirty else 0,
    }


def installed_workspace_packages() -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for workspace in ("tzb_ws", "Nav2_rea_ws"):
        for overlay in ("p7", "p9b"):
            prefix = REPO / workspace / "install" / overlay
            if not (prefix / "setup.bash").is_file():
                continue
            names = sorted(
                marker.name
                for marker in prefix.glob("*/share/colcon-core/packages/*")
                if marker.is_file()
            )
            result.append(
                {"workspace": workspace, "overlay": overlay, "packages": names}
            )
    return result


def recent_build_logs() -> list[dict[str, str]]:
    logs: list[dict[str, str]] = []
    candidates = (
        ("Nav2 工作区最近构建日志", REPO / "Nav2_rea_ws/log/latest_build/logger_all.log"),
        ("机械臂/视觉工作区最近构建日志", REPO / "tzb_ws/log/latest_build/logger_all.log"),
    )
    for label, path in candidates:
        try:
            if not path.is_file():
                continue
            text = path.read_bytes()[-16_000:].decode("utf-8", errors="replace")
            lines = []
            for line in text.splitlines():
                if "DEBUG:" in line or "Level 1:" in line or "CMAKE_PREFIX_PATH=" in line:
                    continue
                line = SECRET_PATTERN.sub(r"\1\2<redacted>", line)
                line = re.sub(r"/home/[^/\s]+", "~", line)
                lines.append(line[:600] + ("…" if len(line) > 600 else ""))
            excerpt = "\n".join(lines[-60:])
            logs.append(
                {
                    "title": label,
                    "updatedAt": time.strftime(
                        "%Y-%m-%d %H:%M:%S %Z", time.localtime(path.stat().st_mtime)
                    ),
                    "text": excerpt,
                }
            )
        except OSError:
            continue
    return logs


def project_processes() -> list[str]:
    running: list[str] = []
    for folder in Path("/proc").glob("[0-9]*"):
        name = read_text(folder / "comm")
        if name and PROJECT_PROCESSES.search(name):
            running.append(name)
    return sorted(set(running))


def snapshot() -> dict[str, object]:
    try:
        load = [round(value, 2) for value in os.getloadavg()]
    except (AttributeError, OSError):
        load = []
    disk = shutil.disk_usage("/")
    nodes = ros_query(ROS_COMMANDS["nodes"])
    topics = ros_query(ROS_COMMANDS["topics"])
    ros_available = ROS_SETUP.is_file()
    root = REPO
    model_dir = root / "tzb_ws/src/best_openvino_model"
    p8_livox_setup = Path.home() / "lunar-cave-p8/install/mid360/setup.bash"
    orbbec_source = root / "tzb_ws/src/OrbbecSDK_ROS2/orbbec_camera/package.xml"
    ros_install_names = {
        package
        for overlay in installed_workspace_packages()
        for package in overlay["packages"]
    }
    return {
        "capturedAt": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "readOnly": True,
        "host": {
            "hostname": socket.gethostname(),
            "os": platform.freedesktop_os_release().get("PRETTY_NAME", platform.platform()),
            "kernel": platform.release(),
            "uptimeSeconds": round(float(read_text(Path("/proc/uptime"), "0").split()[0])),
            "cpuModel": cpu_model(),
            "logicalCores": os.cpu_count(),
            "loadAverage": load,
        },
        "memory": memory_info(),
        "disk": {
            "totalGiB": round(disk.total / (1024**3), 1),
            "freeGiB": round(disk.free / (1024**3), 1),
            "usedPercent": round((disk.used / disk.total) * 100, 1)
            if disk.total
            else None,
        },
        "thermalZones": thermal_zones(),
        "networkLinks": network_links(),
        "repository": repository_state(),
        "ros": {
            "distro": "humble" if ros_available else None,
            "nodeQueryOk": bool(nodes["ok"]),
            "nodes": nodes["items"],
            "topicQueryOk": bool(topics["ok"]),
            "topics": topics["items"],
            "messageFlowSampled": False,
            "processes": project_processes(),
        },
        "workspaces": installed_workspace_packages(),
        "components": {
            "livoxSdkLibrary": Path("/usr/local/lib/liblivox_lidar_sdk_shared.so").is_file(),
            "legacyP8LivoxOverlay": p8_livox_setup.is_file(),
            "orbbecSourcePresent": orbbec_source.is_file(),
            "orbbecBuiltInMainOverlay": "orbbec_camera" in ros_install_names,
            "yoloBuiltInMainOverlay": "yolo_node" in ros_install_names,
            "poseEstimatorBuiltInMainOverlay": "pose_estimator" in ros_install_names,
            "openvinoXmlPresent": (model_dir / "best.xml").is_file(),
            "openvinoWeightsBinPresent": (model_dir / "best.bin").is_file(),
            "openvinoPythonPresent": importlib.util.find_spec("openvino") is not None,
            "ultralyticsPythonPresent": importlib.util.find_spec("ultralytics") is not None,
            "orbbecPythonSdkPresent": importlib.util.find_spec("pyorbbecsdk") is not None,
            "rosbag2DebPresent": Path("/opt/ros/humble/share/rosbag2/package.xml").is_file(),
        },
        "recentBuildLogs": recent_build_logs(),
    }


print(json.dumps(snapshot(), ensure_ascii=False))
