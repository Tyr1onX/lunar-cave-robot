from __future__ import annotations

import importlib.util
import hashlib
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time
import uuid
from dataclasses import dataclass
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Callable
from urllib.parse import parse_qs, urlsplit

HOST = "127.0.0.1"
PORT = 8769
MAX_REQUEST_BYTES = 4096
MAX_LOG_BYTES = 1_048_576
MAX_RUNS = 40
PROCESS_CLEANUP_GRACE_SECONDS = 8.0
RUN_ID_RE = re.compile(r"^[0-9a-f]{32}$")
RESOURCE_LIMITS = {
    "cpu_quota_percent": 300,
    "memory_high_bytes": 8 * 1024**3,
    "memory_max_bytes": 10 * 1024**3,
    "build_jobs": 2,
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _repo_root() -> Path:
    configured = os.environ.get("LUNAR_CONSOLE_REPO")
    repo = Path(configured).expanduser().resolve() if configured else Path(__file__).resolve().parents[3]
    protected = (
        Path("/home/tzb/tzb_ws"),
        Path("/home/tzb/Nav2_rea_ws"),
        Path.home() / "robot_p0" / "tzb_ws",
        Path.home() / "robot_p0" / "Nav2_rea_ws",
    )
    for workspace in protected:
        workspace = workspace.resolve()
        if repo == workspace or workspace in repo.parents:
            raise RuntimeError(f"refusing to run module tests from protected original ROS workspace: {workspace}")
    return repo


def _state_root() -> Path:
    configured = os.environ.get("XDG_STATE_HOME")
    if configured:
        base = Path(configured).expanduser()
    else:
        repo = _repo_root()
        project_root = repo.parent if repo.name == "code" else None
        base = project_root / "tests" / "state" if project_root is not None else Path.home() / ".local" / "state"
    return base / "lunar-cave-robot" / "module-tests"


def _code_revision(repo: Path) -> tuple[str | None, bool | None, int | None]:
    env = {
        "PATH": "/usr/bin:/bin",
        "HOME": str(Path.home()),
        "GIT_OPTIONAL_LOCKS": "0",
        "GIT_CONFIG_NOSYSTEM": "1",
        "GIT_CONFIG_GLOBAL": "/dev/null",
    }
    try:
        head = subprocess.run(
            ["/usr/bin/git", "-c", "core.fsmonitor=false", "-c", "core.untrackedCache=false", "-c", "core.hooksPath=/dev/null", "-C", str(repo), "rev-parse", "HEAD"],
            check=True, capture_output=True, text=True, timeout=2.0, env=env,
        ).stdout.strip()
        status = subprocess.run(
            ["/usr/bin/git", "-c", "core.fsmonitor=false", "-c", "core.untrackedCache=false", "-c", "core.hooksPath=/dev/null", "-C", str(repo), "status", "--porcelain=v1"],
            check=True, capture_output=True, text=True, timeout=2.0, env=env,
        ).stdout.splitlines()
        return head, bool(status), len(status)
    except (OSError, subprocess.SubprocessError):
        return None, None, None


def _model_digest(model_dir: str | None, should_stop: Callable[[], bool], deadline: float) -> str | None:
    if model_dir is None:
        return None
    digest = hashlib.sha256()
    root = Path(model_dir)
    for name in ("metadata.yaml", "yoloe-26l-seg.xml", "yoloe-26l-seg.bin"):
        path = root / name
        digest.update(name.encode("utf-8") + b"\0")
        with path.open("rb") as source:
            while True:
                if should_stop():
                    raise InterruptedError("model hashing cancelled")
                if time.monotonic() >= deadline:
                    raise TimeoutError("model hashing timed out")
                chunk = source.read(1024 * 1024)
                if not chunk:
                    break
                digest.update(chunk)
    return digest.hexdigest()


def _host_resources() -> dict[str, Any]:
    snapshot: dict[str, Any] = {"load_average_1m": None, "memory_total_bytes": None, "memory_available_bytes": None, "thermal_zones_c": {}}
    try:
        snapshot["load_average_1m"] = round(os.getloadavg()[0], 2)
    except (AttributeError, OSError):
        pass
    try:
        for line in Path("/proc/meminfo").read_text(encoding="ascii").splitlines():
            key, _, value = line.partition(":")
            if key in {"MemTotal", "MemAvailable"}:
                snapshot[f"memory_{'total' if key == 'MemTotal' else 'available'}_bytes"] = int(value.split()[0]) * 1024
    except (OSError, ValueError, IndexError):
        pass
    for zone in Path("/sys/class/thermal").glob("thermal_zone*"):
        try:
            kind = (zone / "type").read_text(encoding="ascii").strip() or zone.name
            value = int((zone / "temp").read_text(encoding="ascii").strip())
            snapshot["thermal_zones_c"][kind] = round(value / 1000, 1)
        except (OSError, ValueError):
            continue
    return snapshot


def _process_tree_usage(root_pid: int, seen_cpu_ticks: dict[tuple[int, int], int]) -> tuple[int, int]:
    """Return current descendant RSS and cumulative observed CPU time for one test process tree."""
    proc_root = Path("/proc")
    if not proc_root.is_dir():
        return 0, 0
    processes: dict[int, tuple[int, int, int, int]] = {}
    for entry in proc_root.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            stat = (entry / "stat").read_text(encoding="ascii")
            tail = stat[stat.rfind(")") + 2:].split()
            # tail starts at stat field 3 (state): ppid=field 4, utime/stime=14/15, starttime=22.
            ppid = int(tail[1])
            cpu_ticks = int(tail[11]) + int(tail[12])
            start_ticks = int(tail[19])
            rss_kib = 0
            for line in (entry / "status").read_text(encoding="ascii").splitlines():
                if line.startswith("VmRSS:"):
                    rss_kib = int(line.split()[1])
                    break
            processes[int(entry.name)] = (ppid, cpu_ticks, start_ticks, rss_kib)
        except (OSError, ValueError, IndexError):
            continue
    descendants = {root_pid}
    changed = True
    while changed:
        changed = False
        for pid, (ppid, _cpu, _start, _rss) in processes.items():
            if pid not in descendants and ppid in descendants:
                descendants.add(pid)
                changed = True
    rss_bytes = 0
    ticks_per_second = os.sysconf("SC_CLK_TCK")
    for pid in descendants:
        row = processes.get(pid)
        if row is None:
            continue
        _ppid, cpu_ticks, start_ticks, rss_kib = row
        identity = (pid, start_ticks)
        seen_cpu_ticks[identity] = max(seen_cpu_ticks.get(identity, 0), cpu_ticks)
        rss_bytes += rss_kib * 1024
    cpu_ms = round(sum(seen_cpu_ticks.values()) * 1000 / ticks_per_second)
    return rss_bytes, cpu_ms


def _elapsed_ms(started_at: str) -> int | None:
    try:
        parsed = datetime.fromisoformat(started_at.replace("Z", "+00:00"))
        return max(0, round((datetime.now(timezone.utc) - parsed).total_seconds() * 1000))
    except (TypeError, ValueError):
        return None


@dataclass(frozen=True)
class TestSpec:
    id: str
    title: str
    description: str
    mode: str
    input_source: str
    prerequisites: tuple[str, ...]
    timeout_seconds: int
    command: Callable[[Path], tuple[list[str], dict[str, Any], str | None]]
    check: Callable[[Path], list[str]]
    category: str = "基础检查"
    ros_domain_id: int = 187


def _script(repo: Path, relative: str) -> Path:
    return repo / relative


def _check_ros_environment(repo: Path) -> list[str]:
    script = _script(repo, "tools/module-tests/scripts/check_ros_environment.py")
    return [] if script.is_file() else [f"缺少检查脚本：{script.relative_to(repo)}"]


def _check_arm_fk(repo: Path) -> list[str]:
    script = _script(repo, "tzb_ws/src/serial/driver/test/test_tzb_urdf_joint_fk.py")
    urdf = _script(repo, "tzb_ws/src/tzb_description/urdf/tzb_description.urdf")
    missing = [str(path.relative_to(repo)) for path in (script, urdf) if not path.is_file()]
    return [f"缺少仓库文件：{', '.join(missing)}"] if missing else []


def _field_lab_root() -> Path:
    configured = os.environ.get("LUNAR_CONSOLE_FIELD_LAB_ROOT", "").strip()
    if configured:
        return Path(configured).expanduser()
    repo = _repo_root()
    if repo.name == "code":
        return repo.parent / "lab" / "vision"
    return Path.home() / "lunar-field-lab-p9i"


def _yolo_model_dir() -> Path | None:
    configured = os.environ.get("LUNAR_CONSOLE_YOLO_MODEL_DIR", "").strip()
    if not configured:
        config = Path.home() / ".config" / "lunar-console" / "yolo-model-dir"
        try:
            configured = config.read_text(encoding="utf-8").splitlines()[0].strip()
        except (OSError, IndexError):
            configured = ""
    if configured:
        return Path(configured).expanduser()
    discovered = _field_lab_root() / "models" / "blue_block"
    return discovered if discovered.is_dir() else None


def _check_script_files(repo: Path, relative_paths: tuple[str, ...]) -> list[str]:
    missing = [path for path in relative_paths if not _script(repo, path).is_file()]
    return [f"缺少仓库文件：{', '.join(missing)}"] if missing else []


def _check_ros_suite(repo: Path, script: str) -> list[str]:
    issues = _check_script_files(repo, (script,))
    if not Path("/opt/ros/humble/setup.bash").is_file():
        issues.append("缺少 ROS 2 Humble：/opt/ros/humble/setup.bash")
    if shutil.which("colcon") is None:
        issues.append("缺少 colcon；不会自动安装依赖")
    return issues


def _check_chassis_protocol(repo: Path) -> list[str]:
    issues = _check_script_files(repo, (
        "tools/module-tests/scripts/run_chassis_protocol_checks.py",
        "tools/p5a_offline/test_chassis_control_path.py",
        "tools/p5b_validate_configuration.py",
    ))
    if importlib.util.find_spec("yaml") is None:
        issues.append("当前 Python 环境缺少 PyYAML；不会自动安装")
    return issues


def _check_p6_scenarios(repo: Path) -> list[str]:
    issues = _check_ros_suite(repo, "tools/p6_offline/validate_scenarios.sh")
    for relative in ("Nav2_rea_ws/src/p6_autonomous_mission", "tzb_ws/src/lite_motion_planner"):
        if not _script(repo, relative).is_dir():
            issues.append(f"缺少 P6 离线包目录：{relative}")
    return issues


def _check_p7_scenarios(repo: Path) -> list[str]:
    issues = _check_ros_suite(repo, "tools/p7_offline/validate_scenarios.sh")
    for relative in ("Nav2_rea_ws/src/p6_autonomous_mission", "Nav2_rea_ws/src/drivers/serial/driver", "tzb_ws/src/lite_motion_planner", "tzb_ws/src/serial/driver"):
        if not _script(repo, relative).is_dir():
            issues.append(f"缺少 P7 离线包目录：{relative}")
    return issues


def _check_device_observer(repo: Path) -> list[str]:
    issues = _check_script_files(repo, (
        "tools/module-tests/scripts/observe_ros_device.py",
        "tools/module-tests/scripts/run_ros_readonly_check.sh",
    ))
    if not Path("/opt/ros/humble/setup.bash").is_file():
        issues.append("缺少 ROS 2 Humble：/opt/ros/humble/setup.bash")
    try:
        rclpy_available = importlib.util.find_spec("rclpy") is not None
    except (ImportError, ValueError):
        rclpy_available = False
    if not rclpy_available:
        issues.append("当前服务没有可用的 ROS 2 Python 运行时")
    return issues


def _check_python_modules(python: Path, modules: tuple[str, ...]) -> list[str]:
    probe = "import importlib.util,sys; missing=[name for name in sys.argv[1:] if importlib.util.find_spec(name) is None]; print(','.join(missing)); raise SystemExit(bool(missing))"
    try:
        result = subprocess.run([str(python), "-c", probe, *modules], check=False, capture_output=True, text=True, timeout=3.0)
    except (OSError, subprocess.SubprocessError) as exc:
        return [f"无法检查现有 Python 环境：{type(exc).__name__}"]
    missing = result.stdout.strip()
    if result.returncode:
        return [f"现有 Python 环境缺少模块：{missing or '运行时检查失败'}"]
    return []


def _check_yolo(repo: Path) -> list[str]:
    # These probes inspect the existing environment only; they never install packages.
    issues: list[str] = []
    issues.extend(_check_script_files(repo, ("tools/perception/offline_yolo_ros_smoke.py", "tools/module-tests/scripts/run_yolo_offline.sh")))
    field_lab = _field_lab_root()
    model = _yolo_model_dir()
    if model is None:
        issues.append("未找到模型目录；配置 LUNAR_CONSOLE_YOLO_MODEL_DIR 或在 lab/vision/models/blue_block 准备模型")
    else:
        required = ("metadata.yaml", "yoloe-26l-seg.xml", "yoloe-26l-seg.bin")
        missing = [name for name in required if not (model / name).is_file()]
        if missing:
            issues.append(f"模型目录 {model} 缺少：{', '.join(missing)}")
    python = field_lab / "venv" / "bin" / "python"
    setup = field_lab / "build" / "tzb" / "install" / "setup.bash"
    yolo_prefix = field_lab / "build" / "tzb" / "install" / "yolo_node"
    if not python.is_file() or not os.access(python, os.X_OK):
        issues.append(f"缺少可用的现成 Python 环境：{python}")
    else:
        issues.extend(_check_python_modules(python, ("rclpy", "ament_index_python", "cv2", "ultralytics", "numpy", "openvino")))
    if not Path("/opt/ros/humble/setup.bash").is_file():
        issues.append("缺少 ROS 2 Humble：/opt/ros/humble/setup.bash")
    if not setup.is_file():
        issues.append(f"缺少已构建 ROS overlay：{setup}")
    for relative in ("lib/yolo_node/yolo_detector.py", "share/yolo_node/package.xml", "share/ament_index/resource_index/packages/yolo_node"):
        if not (yolo_prefix / relative).is_file():
            issues.append(f"现有 overlay 缺少 yolo_node 文件：{relative}")
    if not any(yolo_prefix.glob("lib/python*/site-packages/yolo_node/msg/_detection_array.py")):
        issues.append("现有 overlay 缺少 yolo_node/msg/DetectionArray 生成消息")
    for package in ("sensor_msgs", "std_msgs"):
        if not Path("/opt/ros/humble/share", package, "package.xml").is_file():
            issues.append(f"ROS Humble overlay 缺少 {package} 包")
    return issues


def _ros_command(repo: Path) -> tuple[list[str], dict[str, Any], str | None]:
    return [sys.executable, str(_script(repo, "tools/module-tests/scripts/check_ros_environment.py"))], {
        "expected_ros_distro": "humble",
        "ros_domain_id": 187,
        "ros_localhost_only": "1",
        "network_source": "local ROS environment only",
    }, None


def _fk_command(repo: Path) -> tuple[list[str], dict[str, Any], str | None]:
    return [sys.executable, str(_script(repo, "tzb_ws/src/serial/driver/test/test_tzb_urdf_joint_fk.py"))], {
        "urdf": "tzb_ws/src/tzb_description/urdf/tzb_description.urdf",
        "poses": "fixed zero and nonzero six-joint poses in the existing test script",
    }, None


def _yolo_command(repo: Path) -> tuple[list[str], dict[str, Any], str | None]:
    model = _yolo_model_dir()
    if model is None:
        raise RuntimeError("YOLO model directory disappeared after preflight")
    field_lab = _field_lab_root()
    wrapper = _script(repo, "tools/module-tests/scripts/run_yolo_offline.sh")
    return ["/bin/bash", str(wrapper)], {
        "input": "synthetic blue-block image generated by tools/perception/offline_test_data.py",
        "model_dir": str(model.resolve()),
        "ros_overlay_setup": str((field_lab / "build/tzb/install/setup.bash").resolve()),
        "python_environment": str((field_lab / "venv/bin/python").resolve()),
        "inference_device": "CPU",
        "timeout_seconds": 90,
    }, str(model)


def _chassis_protocol_command(repo: Path) -> tuple[list[str], dict[str, Any], str | None]:
    return [sys.executable, str(_script(repo, "tools/module-tests/scripts/run_chassis_protocol_checks.py"))], {
        "checks": ["p5a offline 14-byte protocol model", "p5b static ROS/STM32 source and configuration contract"],
        "device_access": "none; source and software model only",
    }, None


def _fixed_script_command(relative: str, parameters: dict[str, Any]) -> Callable[[Path], tuple[list[str], dict[str, Any], str | None]]:
    def command(repo: Path) -> tuple[list[str], dict[str, Any], str | None]:
        return ["/bin/bash", str(_script(repo, relative))], dict(parameters), None
    return command


def _device_command(device: str) -> Callable[[Path], tuple[list[str], dict[str, Any], str | None]]:
    def command(repo: Path) -> tuple[list[str], dict[str, Any], str | None]:
        return ["/bin/bash", str(_script(repo, "tools/module-tests/scripts/run_ros_readonly_check.sh")), "--device", device], {
            "device_profile": device,
            "ros_domain_id": 0,
            "ros_localhost_only": "1",
            "observation": "existing ROS graph and fixed low-bandwidth status topics; no driver, publisher, service client, or control topic",
        }, None
    return command


def _device_spec(test_id: str, title: str, device: str, description: str, topics: tuple[str, ...]) -> TestSpec:
    return TestSpec(
        id=test_id, title=title, description=description, mode="LIVE",
        input_source="NUC 当前本机 ROS Domain 0 的只读图巡检；localhost-only",
        prerequisites=("ROS 2 Humble 已运行", "只发现既有 Topic/发布者；不启动硬件驱动", *topics),
        timeout_seconds=12, command=_device_command(device), check=_check_device_observer,
        category="只读设备巡检", ros_domain_id=0,
    )


DEFAULT_TESTS: tuple[TestSpec, ...] = (
    TestSpec(
        id="ros_environment", title="ROS 环境检查", description="检查 ROS 2 Humble、Python rclpy 与本地隔离测试环境。",
        mode="OFFLINE", input_source="NUC 软件环境与本地 ROS 配置", prerequisites=("检查目标：ROS 2 Humble / rclpy", "隔离环境：ROS Domain 187 / localhost-only"),
        timeout_seconds=20, command=_ros_command, check=_check_ros_environment, category="基础环境",
    ),
    TestSpec(
        id="arm_fk", title="机械臂 FK / 运动学", description="复用仓库现有六轴 URDF 正向运动学与轴方向检查。",
        mode="OFFLINE", input_source="仓库 tzb_description URDF 与固定六轴测试姿态", prerequisites=("现有 tzb_description URDF", "Python 3 标准库"),
        timeout_seconds=20, command=_fk_command, check=_check_arm_fk, category="机械臂离线模型",
    ),
    TestSpec(
        id="yolo_offline", title="YOLO 离线推理", description="复用现有 ROS 离线烟雾测试，在合成蓝色方块帧上做 CPU 推理。",
        mode="OFFLINE", input_source="合成测试帧；不读取相机，不代表真实场景精度", prerequisites=("NUC OpenVINO 模型目录", "已构建 yolo_node overlay", "OpenCV / Ultralytics / ROS Python"),
        timeout_seconds=180, command=_yolo_command, check=_check_yolo, category="视觉离线测试",
    ),
    TestSpec(
        id="chassis_protocol", title="底盘串口协议 / 静态合同", description="运行现有 14 字节协议离线模型并核对 ROS 桥接源码、参数和帧格式；不打开串口。",
        mode="OFFLINE", input_source="仓库 P5a/P5b 协议模型、桥接源码与 YAML 配置", prerequisites=("Python 3 标准库", "PyYAML", "仅 PTY/源码离线检查；无真实 UART"),
        timeout_seconds=20, command=_chassis_protocol_command,
        check=_check_chassis_protocol,
        category="底盘离线协议",
    ),
    TestSpec(
        id="p6_unit", title="P6 任务协调单元测试", description="复用现有任务状态机和故障处理单元测试。",
        mode="OFFLINE", input_source="P6 自带 unittest 场景与故障用例", prerequisites=("仓库 P6 包与测试文件", "Python 3 标准库", "不启动 ROS 节点或硬件"),
        timeout_seconds=30, command=_fixed_script_command("tools/p6_offline/run_unit_tests.sh", {"suite": "p6_autonomous_mission unittest"}),
        check=lambda repo: _check_script_files(repo, ("tools/p6_offline/run_unit_tests.sh", "Nav2_rea_ws/src/p6_autonomous_mission/test/test_mission_logic.py")),
        category="Nav2 / 任务离线测试",
    ),
    TestSpec(
        id="p6_scenarios", title="P6 Nav2 + 机械臂路径规划场景", description="复用六场景隔离仿真，覆盖 Nav2 导航、机械臂 IK/RRTConnect 路径、取消和故障终止；复用 ROS 依赖，并针对当前仓库源码增量构建测试工作区。",
        mode="SIMULATION", input_source="P6 静态地图、合成 LaserScan/里程计/目标与机械臂反馈", prerequisites=("ROS 2 Humble 与 colcon", "localhost-only 隔离 Domain", "脚本不会启动串口桥、传感器或实体控制"),
        timeout_seconds=3600, command=_fixed_script_command("tools/p6_offline/validate_scenarios.sh", {"suite": "six P6 simulation scenarios", "workspace_build": "incremental colcon build from this checkout", "dependency_install": False, "hardware_access": "none"}),
        check=_check_p6_scenarios,
        category="Nav2 / 任务离线测试",
    ),
    TestSpec(
        id="p7_scenarios", title="P7 底盘 / 机械臂 PTY 离线验收", description="复用 12 个离线故障场景和 PTY STM32/机械臂模拟器；无真实串口、电机或传感器。",
        mode="SIMULATION", input_source="P7 固定场景输入、临时 PTY 和合成反馈", prerequisites=("ROS 2 Humble 与 colcon", "localhost-only ROS Domain", "只构建仓库软件包，不触碰物理设备"),
        timeout_seconds=5400, command=_fixed_script_command("tools/p7_offline/validate_scenarios.sh", {"suite": "12 P7 PTY simulation scenarios", "workspace_build": "incremental colcon build from this checkout", "dependency_install": False, "hardware_access": "none"}),
        check=_check_p7_scenarios,
        category="Nav2 / 任务离线测试",
    ),
    _device_spec("camera_readonly", "相机 ROS 只读连接", "camera", "检查现有相机图像/CameraInfo Topic 发布者、低带宽 CameraInfo 频率及诊断状态；不订阅图像流。", ("/camera/color/image_raw", "/camera/depth/image_rect_raw", "/camera/color/camera_info", "/diagnostics")),
    _device_spec("mid360_readonly", "MID-360 ROS 只读连接", "mid360", "检查现有雷达点云/IMU Topic 发布者、低带宽 IMU 频率及诊断状态；不订阅点云。", ("/livox/lidar", "/livox/lidar/pointcloud", "/livox/imu", "/diagnostics")),
    _device_spec("chassis_readonly", "底盘控制器只读状态", "chassis", "只读检查串口桥状态 Topic、消息频率和错误文本；不发布速度或打开串口。", ("/chassis/serial_link_status", "/diagnostics")),
    _device_spec("arm_readonly", "机械臂控制器只读状态", "arm", "只读检查机械臂安全状态与关节反馈 Topic、消息频率和故障文本；不发运动命令。", ("/serial/arm_safety/status", "/joint_states", "/diagnostics")),
)


class TestManager:
    def __init__(self, repo: Path | None = None, state_root: Path | None = None, specs: tuple[TestSpec, ...] = DEFAULT_TESTS):
        self.repo = (repo or _repo_root()).resolve()
        self.state_root = state_root or _state_root()
        self.specs = {item.id: item for item in specs}
        self._lock = threading.RLock()
        self._runs: dict[str, dict[str, Any]] = {}
        self._restore_reports()

    def _restore_reports(self) -> None:
        if not self.state_root.is_dir():
            return
        reports: list[tuple[float, dict[str, Any]]] = []
        for path in self.state_root.glob("*.json"):
            try:
                record = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            if not isinstance(record, dict) or not RUN_ID_RE.fullmatch(str(record.get("run_id", ""))):
                continue
            run_id = str(record["run_id"])
            log_path = self.state_root / f"{run_id}.log"
            record["report_file"] = str(path)
            record["log_file"] = str(log_path)
            if record.get("status") in {"running", "cancelling"}:
                record.update({
                    "status": "interrupted",
                    "result": "interrupted",
                    "finished_at": utc_now(),
                    "duration_ms": _elapsed_ms(str(record.get("started_at", ""))),
                    "reason": "受限测试服务重启时测试中断；systemd 已清理该服务的子进程。",
                })
                try:
                    with log_path.open("ab") as handle:
                        handle.write(b"\n[INTERRUPTED] test runner restarted; child process cleanup is managed by systemd.\n")
                except OSError:
                    record["reason"] += " 测试日志文件无法追加恢复标记。"
            try:
                log_bytes = b""
                log_bytes = log_path.read_bytes()[-MAX_LOG_BYTES:]
                log_text = log_bytes.decode("utf-8", errors="replace")
            except OSError:
                log_text = str(record.get("log_text", ""))
            lines = log_text.splitlines()
            record["log_text"] = log_text
            record["log_truncated"] = bool(record.get("log_truncated", False)) or len(log_bytes) >= MAX_LOG_BYTES
            record["log_lines_truncated"] = bool(record.get("log_lines_truncated", False)) or len(lines) > 5000
            reports.append((path.stat().st_mtime, record))
            if record.get("status") == "interrupted":
                try:
                    self._save_report(record)
                except OSError:
                    pass
        for _modified, record in sorted(reports, key=lambda item: item[0], reverse=True)[:MAX_RUNS]:
            run_id = record["run_id"]
            lines = str(record.get("log_text", "")).splitlines()
            discarded = max(0, len(lines) - 5000)
            self._runs[run_id] = {
                "record": record,
                "spec": self.specs.get(str(record.get("test_id", ""))),
                "command": [],
                "model_dir": None,
                "logs": lines[-5000:],
                "log_bytes": min(MAX_LOG_BYTES, len(str(record.get("log_text", "")).encode("utf-8"))),
                "discarded_lines": discarded,
                "cancel": threading.Event(),
                "process": None,
                "thread": None,
                "seen_cpu_ticks": {},
            }

    def catalog(self) -> dict[str, Any]:
        tests: list[dict[str, Any]] = []
        for spec in self.specs.values():
            reasons = spec.check(self.repo)
            tests.append({
                "id": spec.id,
                "title": spec.title,
                "category": spec.category,
                "description": spec.description,
                "mode": spec.mode,
                "state": "ready" if not reasons else "locked",
                "reason": "; ".join(reasons) if reasons else None,
                "input_source": spec.input_source,
                "prerequisites": list(spec.prerequisites),
                "timeout_seconds": spec.timeout_seconds,
            })
        with self._lock:
            active = any(run["record"]["status"] in {"running", "cancelling"} for run in self._runs.values())
        return {"service": "lunar-console-module-test-runner", "source": "nuc-restricted-module-test-runner", "tests": tests, "busy": active, "collected_at": utc_now()}

    def history(self, limit: int = 20) -> dict[str, Any]:
        with self._lock:
            records = [run["record"] for run in self._runs.values()]
            ordered = sorted(records, key=lambda item: item.get("started_at", ""), reverse=True)[:max(1, min(limit, MAX_RUNS))]
            summary_keys = ("run_id", "test_id", "title", "mode", "status", "result", "code_sha", "started_at", "finished_at", "duration_ms", "reason")
            return {"reports": [{key: record.get(key) for key in summary_keys} for record in ordered]}

    def start(self, test_id: str) -> tuple[dict[str, Any] | None, str | None, int]:
        spec = self.specs.get(test_id)
        if spec is None:
            return None, "unknown test id", 404
        with self._lock:
            if any(run["record"]["status"] in {"running", "cancelling"} for run in self._runs.values()):
                return None, "another test is already running", 409
            reasons = spec.check(self.repo)
            if reasons:
                return {"test_id": test_id, "state": "locked", "reason": "; ".join(reasons)}, "test prerequisites are not met", 409
            run_id = uuid.uuid4().hex
            started_at = utc_now()
            sha, dirty, dirty_files = _code_revision(self.repo)
            command, parameters, model_dir = spec.command(self.repo)
            record: dict[str, Any] = {
                "schema_version": 1,
                "run_id": run_id,
                "test_id": spec.id,
                "title": spec.title,
                "mode": spec.mode,
                "status": "running",
                "result": "running",
                "input_source": spec.input_source,
                "parameters": parameters,
                "allowlisted_script": str(Path(command[1]).resolve().relative_to(self.repo)) if len(command) > 1 else None,
                "code_sha": sha,
                "model_sha256": None,
                "code_dirty": dirty,
                "code_dirty_files": dirty_files,
                "started_at": started_at,
                "finished_at": None,
                "duration_ms": None,
                "return_code": None,
                "log_file": str(self.state_root / f"{run_id}.log"),
                "report_file": str(self.state_root / f"{run_id}.json"),
                "log_text": "",
                "log_truncated": False,
                "log_lines_truncated": False,
                "reason": None,
            }
            run = {"record": record, "spec": spec, "command": command, "model_dir": model_dir, "logs": [], "log_bytes": 0, "discarded_lines": 0, "cancel": threading.Event(), "process": None, "thread": None}
            self._runs[run_id] = run
            try:
                self.state_root.mkdir(mode=0o700, parents=True, exist_ok=True)
                os.chmod(self.state_root, 0o700)
            except OSError as exc:
                self._runs.pop(run_id, None)
                return None, f"无法创建本地报告目录：{type(exc).__name__}", 507
            mode_note = {
                "SIMULATION": "使用合成数据和隔离仿真；不连接实体设备。",
                "OFFLINE": "软件离线检查；不连接实体设备。",
                "LIVE": "只读检查既有本机 ROS Topic；不启动驱动或发送控制。",
            }.get(spec.mode, "")
            self._write_log(run_id, f"[{spec.mode}] {mode_note}\n")
            try:
                record["log_text"] = "\n".join(run["logs"])
                self._save_report(record)
            except OSError as exc:
                self._runs.pop(run_id, None)
                return None, f"无法保存初始测试报告：{type(exc).__name__}", 507
            worker = threading.Thread(target=self._execute, args=(run_id,), name=f"module-test-{spec.id}", daemon=True)
            run["thread"] = worker
            worker.start()
            self._prune_runs_locked()
        return self.get(run_id), None, 202

    def _prune_runs_locked(self) -> None:
        if len(self._runs) <= MAX_RUNS:
            return
        for run_id, run in list(self._runs.items()):
            if run["record"]["status"] not in {"running", "cancelling"}:
                self._runs.pop(run_id, None)
            if len(self._runs) <= MAX_RUNS:
                break

    def _write_log(self, run_id: str, line: str) -> None:
        run = self._runs[run_id]
        payload = line.encode("utf-8", errors="replace")
        with self._lock:
            if run["log_bytes"] >= MAX_LOG_BYTES:
                run["record"]["log_truncated"] = True
                return
            allowed = MAX_LOG_BYTES - run["log_bytes"]
            if len(payload) > allowed:
                payload = payload[:allowed]
                line = payload.decode("utf-8", errors="replace")
                run["record"]["log_truncated"] = True
            run["log_bytes"] += len(payload)
            run["logs"].append(line.rstrip("\r\n"))
            if len(run["logs"]) > 5000:
                discarded = len(run["logs"]) - 5000
                del run["logs"][:discarded]
                run["discarded_lines"] += discarded
                run["record"]["log_lines_truncated"] = True
            log_path = Path(run["record"]["log_file"])
        try:
            with log_path.open("ab") as handle:
                handle.write(payload)
            os.chmod(log_path, 0o600)
        except OSError as exc:
            with self._lock:
                run["record"]["log_storage_error"] = f"{type(exc).__name__}: 无法持久化日志"

    def _execute(self, run_id: str) -> None:
        with self._lock:
            run = self._runs[run_id]
            spec: TestSpec = run["spec"]
            command: list[str] = run["command"]
        started_mono = time.monotonic()
        status = "failed"
        result = "failed"
        reason = None
        return_code: int | None = None
        deadline = started_mono + spec.timeout_seconds
        child_env = os.environ.copy()
        child_env["ROS_DOMAIN_ID"] = str(spec.ros_domain_id)
        child_env["ROS_LOCALHOST_ONLY"] = "1"
        child_env["PYTHONDONTWRITEBYTECODE"] = "1"
        child_env["YOLO_AUTOINSTALL"] = "false"
        if spec.id in {"p6_scenarios", "p7_scenarios"}:
            child_env.update({
                "P6_BUILD_JOBS": str(RESOURCE_LIMITS["build_jobs"]),
                "P7_BUILD_JOBS": str(RESOURCE_LIMITS["build_jobs"]),
                "CMAKE_BUILD_PARALLEL_LEVEL": str(RESOURCE_LIMITS["build_jobs"]),
                "MAKEFLAGS": f"-j{RESOURCE_LIMITS['build_jobs']}",
                "OMP_NUM_THREADS": "1",
                "OPENBLAS_NUM_THREADS": "1",
                "MKL_NUM_THREADS": "1",
                "NUMEXPR_MAX_THREADS": "1",
            })
            scenario_results = self.state_root / "scenario-runs" / spec.id / run_id
            if spec.id == "p6_scenarios":
                child_env["P6_RESULTS_DIR"] = str(scenario_results)
            else:
                child_env["P7_RESULTS_DIR"] = str(scenario_results)
            with self._lock:
                run["record"]["parameters"]["resource_limits"] = dict(RESOURCE_LIMITS)
                run["record"]["parameters"]["persistent_scenario_results"] = str(scenario_results)
        resources: dict[str, Any] | None = None
        if spec.id in {"p6_scenarios", "p7_scenarios"}:
            resources = {"host_before": _host_resources(), "peak_process_tree_rss_bytes": 0, "process_tree_cpu_ms": 0, "samples": 0}
            run["seen_cpu_ticks"] = {}
        cache_root: Path | None = None
        child: subprocess.Popen[str] | None = None
        try:
            cache_root = Path(tempfile.mkdtemp(prefix="lunar-console-module-test-"))
            child_env["YOLO_CONFIG_DIR"] = str(cache_root / "yolo-config")
            child_env["OV_CACHE_DIR"] = str(cache_root / "openvino-cache")
            if run["model_dir"]:
                model_sha = _model_digest(run["model_dir"], run["cancel"].is_set, deadline)
                with self._lock:
                    run["record"]["model_sha256"] = model_sha
            if run["cancel"].is_set():
                raise InterruptedError("test cancelled before process start")
            if time.monotonic() >= deadline:
                raise TimeoutError("test timed out before process start")
            # One fixed script and its fixed, registry-generated arguments per test ID.
            child = subprocess.Popen(
                command, cwd=self.repo, env=child_env, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8", errors="replace",
                bufsize=1, start_new_session=True,
            )
            with self._lock:
                run["process"] = child

            def read_output() -> None:
                assert child.stdout is not None
                for line in child.stdout:
                    self._write_log(run_id, line)

            reader = threading.Thread(target=read_output, name=f"module-test-log-{run_id[:8]}", daemon=True)
            reader.start()
            next_resource_sample = time.monotonic()
            while child.poll() is None:
                if run["cancel"].is_set():
                    self._terminate_process_group(child)
                    if run.get("service_shutdown"):
                        status, result = "interrupted", "interrupted"
                        reason = "受限测试服务关闭时测试中断；测试子进程已清理。"
                    else:
                        status, result = "cancelled", "cancelled"
                    break
                if time.monotonic() >= deadline:
                    self._terminate_process_group(child)
                    status, result = "timed_out", "timed_out"
                    reason = f"超过 {spec.timeout_seconds} 秒超时限制"
                    break
                if resources is not None and time.monotonic() >= next_resource_sample:
                    rss_bytes, cpu_ms = _process_tree_usage(child.pid, run["seen_cpu_ticks"])
                    resources["peak_process_tree_rss_bytes"] = max(resources["peak_process_tree_rss_bytes"], rss_bytes)
                    resources["process_tree_cpu_ms"] = max(resources["process_tree_cpu_ms"], cpu_ms)
                    resources["samples"] += 1
                    next_resource_sample = time.monotonic() + 1.0
                time.sleep(0.05)
            return_code = child.poll()
            if status == "failed" and return_code == 0:
                status, result = "passed", "passed"
            elif status == "failed":
                result = "failed"
                reason = f"测试脚本退出码 {return_code}"
            reader.join(timeout=3.0)
            if child.stdout is not None:
                child.stdout.close()
        except InterruptedError as exc:
            if child is not None and child.poll() is None:
                self._terminate_process_group(child)
            if run.get("service_shutdown"):
                status, result = "interrupted", "interrupted"
                reason = "受限测试服务关闭时测试中断；测试子进程已清理。"
            else:
                status, result, reason = "cancelled", "cancelled", str(exc)
        except TimeoutError as exc:
            if child is not None and child.poll() is None:
                self._terminate_process_group(child)
            status, result, reason = "timed_out", "timed_out", str(exc)
            self._write_log(run_id, f"{reason}\n")
        except Exception as exc:
            if child is not None and child.poll() is None:
                self._terminate_process_group(child)
            status, result, reason = "failed", "failed", f"执行异常：{type(exc).__name__}: {exc}"
            self._write_log(run_id, f"{reason}\n")
        finally:
            if resources is not None:
                if child is not None:
                    rss_bytes, cpu_ms = _process_tree_usage(child.pid, run["seen_cpu_ticks"])
                    resources["peak_process_tree_rss_bytes"] = max(resources["peak_process_tree_rss_bytes"], rss_bytes)
                    resources["process_tree_cpu_ms"] = max(resources["process_tree_cpu_ms"], cpu_ms)
                resources["host_after"] = _host_resources()
                with self._lock:
                    run["record"]["resource_usage"] = resources
            if cache_root is not None:
                shutil.rmtree(cache_root, ignore_errors=True)
            finished = utc_now()
            duration = round((time.monotonic() - started_mono) * 1000)
            with self._lock:
                run = self._runs[run_id]
                record = run["record"]
                record.update({
                    "status": status,
                    "result": result,
                    "finished_at": finished,
                    "duration_ms": duration,
                    "return_code": return_code,
                    "reason": reason,
                    "log_text": "\n".join(run["logs"]) + ("\n[日志达到 1 MiB 上限，后续输出已截断。]" if record["log_truncated"] else "") + ("\n[内存中的前段日志已裁剪；完整保留日志见独立 .log 文件。]" if record["log_lines_truncated"] else ""),
                })
                if record.get("log_storage_error"):
                    record["reason"] = "; ".join(part for part in (record["reason"], record["log_storage_error"]) if part)
                try:
                    self._save_report(record)
                except OSError as exc:
                    record["status"] = "failed"
                    record["result"] = "failed"
                    record["reason"] = "; ".join(part for part in (record["reason"], f"无法持久化测试报告：{type(exc).__name__}") if part)

    @staticmethod
    def _terminate_process_group(child: subprocess.Popen[str]) -> None:
        if child.poll() is not None:
            return
        try:
            os.killpg(child.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        try:
            child.wait(timeout=PROCESS_CLEANUP_GRACE_SECONDS)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait(timeout=2.0)

    def _save_report(self, record: dict[str, Any]) -> None:
        path = Path(record["report_file"])
        temp = path.with_suffix(".json.tmp")
        temp.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        os.chmod(temp, 0o600)
        os.replace(temp, path)

    def cancel(self, run_id: str) -> tuple[dict[str, Any] | None, str | None, int]:
        with self._lock:
            run = self._runs.get(run_id)
            if run is None:
                return None, "run not found", 404
            if run["record"]["status"] not in {"running", "cancelling"}:
                return self.get(run_id), "run has already finished", 409
            run["record"]["status"] = "cancelling"
            run["cancel"].set()
        return self.get(run_id), None, 202

    def get(self, run_id: str) -> dict[str, Any] | None:
        with self._lock:
            run = self._runs.get(run_id)
            return dict(run["record"]) if run else None

    def logs(self, run_id: str, after: int) -> dict[str, Any] | None:
        with self._lock:
            run = self._runs.get(run_id)
            if not run:
                return None
            values = run["logs"]
            base = run["discarded_lines"]
            next_cursor = base + len(values)
            start = max(0, min(after, next_cursor) - base)
            return {"run_id": run_id, "lines": values[start:], "cursor": next_cursor, "status": run["record"]["status"]}

    def report(self, run_id: str) -> dict[str, Any] | None:
        with self._lock:
            run = self._runs.get(run_id)
            if not run or run["record"]["status"] in {"running", "cancelling"}:
                return None
            return dict(run["record"])

    def stop_all(self) -> None:
        with self._lock:
            active = [run for run in self._runs.values() if run["record"]["status"] in {"running", "cancelling"}]
            for run in active:
                run["service_shutdown"] = True
                run["cancel"].set()
        for run in active:
            worker = run["thread"]
            if worker is not None:
                worker.join(timeout=PROCESS_CLEANUP_GRACE_SECONDS + 4.0)


class ModuleTestHandler(BaseHTTPRequestHandler):
    server_version = "LunarConsoleModuleTests/0.3"
    sys_version = ""

    @property
    def manager(self) -> TestManager:
        return self.server.manager  # type: ignore[attr-defined]

    def log_message(self, _format: str, *_args: object) -> None:
        return

    def _respond(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        request = urlsplit(self.path)
        if request.path == "/health":
            self._respond(200, {"ok": True, "schema_version": 1, "service": "lunar-console-module-test-runner", "collected_at": utc_now()})
            return
        if request.path == "/v1/tests/catalog":
            self._respond(200, self.manager.catalog())
            return
        if request.path == "/v1/tests/reports":
            query = parse_qs(request.query, keep_blank_values=False)
            try:
                limit = int(query.get("limit", ["20"])[0])
            except ValueError:
                self._respond(400, {"error": "limit must be an integer"})
                return
            self._respond(200, self.manager.history(limit))
            return
        parts = request.path.split("/")
        if len(parts) == 4 and parts[:3] == ["", "v1", "tests"] and RUN_ID_RE.fullmatch(parts[3]):
            record = self.manager.get(parts[3])
            self._respond(200 if record else 404, record or {"error": "run not found"})
            return
        if len(parts) == 5 and parts[:3] == ["", "v1", "tests"] and RUN_ID_RE.fullmatch(parts[3]) and parts[4] == "logs":
            query = parse_qs(request.query, keep_blank_values=False)
            try:
                after = int(query.get("after", ["0"])[0])
            except ValueError:
                self._respond(400, {"error": "after must be an integer"})
                return
            payload = self.manager.logs(parts[3], after)
            self._respond(200 if payload else 404, payload or {"error": "run not found"})
            return
        if len(parts) == 5 and parts[:3] == ["", "v1", "tests"] and RUN_ID_RE.fullmatch(parts[3]) and parts[4] == "report":
            payload = self.manager.report(parts[3])
            self._respond(200 if payload else 409, payload or {"error": "report is not ready"})
            return
        self._respond(404, {"error": "not found"})

    def do_POST(self) -> None:
        request = urlsplit(self.path)
        if request.path == "/v1/tests/run":
            try:
                length = int(self.headers.get("Content-Length", "0"))
            except ValueError:
                self._respond(400, {"error": "invalid content length"})
                return
            if length < 1 or length > MAX_REQUEST_BYTES:
                self._respond(413 if length > MAX_REQUEST_BYTES else 400, {"error": "request body must be 1..4096 bytes"})
                return
            try:
                payload = json.loads(self.rfile.read(length))
            except (json.JSONDecodeError, UnicodeDecodeError):
                self._respond(400, {"error": "invalid JSON"})
                return
            if not isinstance(payload, dict) or set(payload) != {"test_id"} or not isinstance(payload["test_id"], str):
                self._respond(400, {"error": "body must contain only the allowlisted test_id field"})
                return
            run, error, status = self.manager.start(payload["test_id"])
            self._respond(status, run or {"error": error or "test could not start"})
            return
        parts = request.path.split("/")
        if len(parts) == 5 and parts[:3] == ["", "v1", "tests"] and RUN_ID_RE.fullmatch(parts[3]) and parts[4] == "cancel":
            payload, error, status = self.manager.cancel(parts[3])
            self._respond(status, payload or {"error": error or "run could not be cancelled"})
            return
        self._respond(404, {"error": "not found"})

    def _method_not_allowed(self) -> None:
        self.send_response(405)
        self.send_header("Allow", "GET, POST")
        self.send_header("Content-Length", "0")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

    do_PUT = _method_not_allowed
    do_PATCH = _method_not_allowed
    do_DELETE = _method_not_allowed
    do_OPTIONS = _method_not_allowed


class ModuleTestHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, manager: TestManager, address: tuple[str, int] = (HOST, PORT)):
        self.manager = manager
        super().__init__(address, ModuleTestHandler)


def serve() -> None:
    host = os.environ.get("LUNAR_CONSOLE_TEST_BIND", HOST)
    if host != HOST:
        raise SystemExit("LUNAR_CONSOLE_TEST_BIND must remain 127.0.0.1")
    server = ModuleTestHTTPServer(TestManager())
    def shutdown(_signum: int, _frame: Any) -> None:
        threading.Thread(target=server.shutdown, name="module-test-shutdown", daemon=True).start()
    signal.signal(signal.SIGTERM, shutdown)
    signal.signal(signal.SIGINT, shutdown)
    print(f"Lunar Console restricted module test runner listening on {HOST}:{PORT}", flush=True)
    try:
        server.serve_forever(poll_interval=0.5)
    finally:
        server.manager.stop_all()
        server.server_close()


if __name__ == "__main__":
    serve()
