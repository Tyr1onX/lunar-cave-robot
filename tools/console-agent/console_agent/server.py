from __future__ import annotations

import json
import os
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlsplit

from .collectors import collect_logs, collect_snapshot, start_ros_graph_observer, utc_now

HOST = "127.0.0.1"
PORT = 8765


class ReadOnlyHandler(BaseHTTPRequestHandler):
    server_version = "LunarConsoleAgent/0.1"
    sys_version = ""

    def log_message(self, _format: str, *_args: object) -> None:
        return

    def _respond(self, status: int, payload: dict[str, object]) -> None:
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
            self._respond(200, {"ok": True, "schema_version": 1, "collected_at": utc_now()})
            return
        if request.path == "/v1/snapshot":
            try:
                self._respond(200, collect_snapshot())
            except Exception as exc:
                self._respond(500, {"error": "snapshot collection failed", "reason": type(exc).__name__})
            return
        if request.path == "/v1/logs":
            query = parse_qs(request.query, keep_blank_values=False)
            try:
                limit = int(query.get("limit", ["100"])[0])
            except ValueError:
                self._respond(400, {"error": "limit must be an integer"})
                return
            self._respond(200, collect_logs(max(1, min(limit, 200))))
            return
        self._respond(404, {"error": "not found"})

    def _method_not_allowed(self) -> None:
        self.send_response(405)
        self.send_header("Allow", "GET")
        self.send_header("Content-Length", "0")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()

    do_POST = _method_not_allowed
    do_PUT = _method_not_allowed
    do_PATCH = _method_not_allowed
    do_DELETE = _method_not_allowed
    do_OPTIONS = _method_not_allowed


def main() -> None:
    host = os.environ.get("LUNAR_CONSOLE_BIND", HOST)
    if host != HOST:
        raise SystemExit("LUNAR_CONSOLE_BIND must remain 127.0.0.1")
    observer = start_ros_graph_observer()
    server = ThreadingHTTPServer((HOST, PORT), ReadOnlyHandler)
    server.daemon_threads = True
    print(f"Lunar Console read-only agent listening on {HOST}:{PORT}", flush=True)
    try:
        server.serve_forever(poll_interval=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        observer.stop()


if __name__ == "__main__":
    main()
