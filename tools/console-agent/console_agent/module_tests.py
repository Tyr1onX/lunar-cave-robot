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
RUN_ID_RE = re.compile(r"^[0-9a-f]{32}$")


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _repo_root() -> Path:
    configured = os.environ.get("LUNAR_CONSOLE_REPO")
    return Path(configured).expanduser().resolve() if configured else Path(__file__).resolve().parents[3]


def _state_root() -> Path:
    configured = os.environ.get("XDG_STATE_HOME")
    base = Path(configured).expanduser() if configured else Path.home() / ".local" / "state"
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


def _check_yolo(repo: Path) -> list[str]:
    issues: list[str] = []
    script = _script(repo, "tools/perception/offline_yolo_ros_smoke.py")
    if not script.is_file():
        issues.append(f"缺少离线推理脚本：{script.relative_to(repo)}")
    model_value = os.environ.get("LUNAR_CONSOLE_YOLO_MODEL_DIR", "").strip()
    if not model_value:
        issues.append("未配置 NUC 环境变量 LUNAR_CONSOLE_YOLO_MODEL_DIR（模型目录不随 Git 分发）")
    else:
        model = Path(model_value).expanduser()
        required = ("metadata.yaml", "yoloe-26l-seg.xml", "yoloe-26l-seg.bin")
        missing = [name for name in required if not (model / name).is_file()]
        if missing:
            issues.append(f"模型目录缺少：{', '.join(missing)}")
    for package in ("rclpy", "ament_index_python", "cv2", "ultralytics", "numpy"):
        try:
            available = importlib.util.find_spec(package) is not None
        except (ImportError, ValueError):
            available = False
        if not available:
            issues.append(f"NUC Python 环境缺少模块：{package}")
    try:
        from ament_index_python.packages import get_package_prefix
        for package in ("yolo_node", "sensor_msgs", "std_msgs"):
            get_package_prefix(package)
    except Exception:
        issues.append("ROS overlay 中缺少 yolo_node、sensor_msgs 或 std_msgs 包")
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
    model = Path(os.environ["LUNAR_CONSOLE_YOLO_MODEL_DIR"]).expanduser().resolve()
    return [sys.executable, str(_script(repo, "tools/perception/offline_yolo_ros_smoke.py")), "--model", str(model), "--timeout", "90"], {
        "input": "synthetic blue-block image generated by tools/perception/offline_test_data.py",
        "model_dir": str(model),
        "inference_device": "CPU",
        "timeout_seconds": 90,
    }, str(model)


DEFAULT_TESTS: tuple[TestSpec, ...] = (
    TestSpec(
        id="ros_environment", title="ROS 环境检查", description="检查 ROS 2 Humble、Python rclpy 与本地隔离测试环境。",
        mode="OFFLINE", input_source="NUC 软件环境与本地 ROS 配置", prerequisites=("检查目标：ROS 2 Humble / rclpy", "隔离环境：ROS Domain 187 / localhost-only"),
        timeout_seconds=20, command=_ros_command, check=_check_ros_environment,
    ),
    TestSpec(
        id="arm_fk", title="机械臂 FK / 运动学", description="复用仓库现有六轴 URDF 正向运动学与轴方向检查。",
        mode="OFFLINE", input_source="仓库 tzb_description URDF 与固定六轴测试姿态", prerequisites=("现有 tzb_description URDF", "Python 3 标准库"),
        timeout_seconds=20, command=_fk_command, check=_check_arm_fk,
    ),
    TestSpec(
        id="yolo_offline", title="YOLO 离线推理", description="复用现有 ROS 离线烟雾测试，在合成蓝色方块帧上做 CPU 推理。",
        mode="OFFLINE", input_source="合成测试帧；不读取相机，不代表真实场景精度", prerequisites=("NUC OpenVINO 模型目录", "已构建 yolo_node overlay", "OpenCV / Ultralytics / ROS Python"),
        timeout_seconds=120, command=_yolo_command, check=_check_yolo,
    ),
)


class TestManager:
    def __init__(self, repo: Path | None = None, state_root: Path | None = None, specs: tuple[TestSpec, ...] = DEFAULT_TESTS):
        self.repo = (repo or _repo_root()).resolve()
        self.state_root = state_root or _state_root()
        self.specs = {item.id: item for item in specs}
        self._lock = threading.RLock()
        self._runs: dict[str, dict[str, Any]] = {}

    def catalog(self) -> dict[str, Any]:
        tests: list[dict[str, Any]] = []
        for spec in self.specs.values():
            reasons = spec.check(self.repo)
            tests.append({
                "id": spec.id,
                "title": spec.title,
                "description": spec.description,
                "mode": spec.mode,
                "state": "ready" if not reasons else "locked",
                "reason": "; ".join(reasons) if reasons else None,
                "input_source": spec.input_source,
                "prerequisites": list(spec.prerequisites),
                "timeout_seconds": spec.timeout_seconds,
            })
        with self._lock:
            active = any(run["status"] in {"running", "cancelling"} for run in self._runs.values())
        return {"service": "lunar-console-module-test-runner", "source": "nuc-restricted-module-test-runner", "tests": tests, "busy": active, "collected_at": utc_now()}

    def start(self, test_id: str) -> tuple[dict[str, Any] | None, str | None, int]:
        spec = self.specs.get(test_id)
        if spec is None:
            return None, "unknown test id", 404
        with self._lock:
            if any(run["status"] in {"running", "cancelling"} for run in self._runs.values()):
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
            self._write_log(run_id, "[SIMULATION/OFFLINE] 此任务为软件离线测试；不连接实体设备。\n")
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
        child_env["ROS_DOMAIN_ID"] = "187"
        child_env["ROS_LOCALHOST_ONLY"] = "1"
        child_env["PYTHONDONTWRITEBYTECODE"] = "1"
        child_env["YOLO_AUTOINSTALL"] = "false"
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
            while child.poll() is None:
                if run["cancel"].is_set():
                    self._terminate_process_group(child)
                    status, result = "cancelled", "cancelled"
                    break
                if time.monotonic() >= deadline:
                    self._terminate_process_group(child)
                    status, result = "timed_out", "timed_out"
                    reason = f"超过 {spec.timeout_seconds} 秒超时限制"
                    break
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
        except InterruptedError:
            if child is not None and child.poll() is None:
                self._terminate_process_group(child)
            status, result = "cancelled", "cancelled"
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
            child.wait(timeout=2.0)
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
            active = [(run["cancel"], run["thread"]) for run in self._runs.values() if run["record"]["status"] in {"running", "cancelling"}]
            for cancel, _thread in active:
                cancel.set()
        for _cancel, worker in active:
            if worker is not None:
                worker.join(timeout=3.0)


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
