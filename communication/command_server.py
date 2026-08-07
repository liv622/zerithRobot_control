"""JSON/HTTP command adapter for a robot simulation application."""

from __future__ import annotations

import json
import threading
import time
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any, Protocol

JsonObject = dict[str, Any]

# SSE stream throttle: push at most one frame every STREAM_INTERVAL_S seconds.
_STREAM_INTERVAL_S = 0.020  # 50 Hz


class _OscilloscopeSource(Protocol):
    """Minimal interface that both JointMonitor and OscilloscopeService satisfy."""

    def latest(self) -> Any: ...
    @property
    def joint_count(self) -> int: ...

# SSE stream throttle: push at most one frame every STREAM_INTERVAL_S seconds.
_STREAM_INTERVAL_S = 0.020  # 50 Hz


class SimulationCommandServer:
    """Expose application state and commands over a small local HTTP API."""

    def __init__(
        self,
        host: str,
        port: int,
        state_reader: Callable[[], JsonObject],
        command_handler: Callable[[JsonObject], JsonObject],
        oscilloscope: _OscilloscopeSource | None = None,
    ) -> None:
        self.state_reader = state_reader
        self.command_handler = command_handler
        self.oscilloscope = oscilloscope
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def _send_cors(self) -> None:
                self.send_header("Access-Control-Allow-Origin", "*")
                self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
                self.send_header("Access-Control-Allow-Headers", "Content-Type")

            def _reply(self, status: int, payload: JsonObject) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self._send_cors()
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_OPTIONS(self) -> None:  # noqa: N802
                self.send_response(HTTPStatus.NO_CONTENT)
                self._send_cors()
                self.send_header("Content-Length", "0")
                self.end_headers()

            def do_GET(self) -> None:  # noqa: N802
                if self.path.rstrip("/") == "/api/oscilloscope/stream":
                    self._serve_sse()
                    return
                if self.path.rstrip("/") != "/api/state":
                    self._reply(
                        HTTPStatus.NOT_FOUND,
                        {"ok": False, "error": "not found"},
                    )
                    return
                try:
                    self._reply(
                        HTTPStatus.OK,
                        {"ok": True, "state": bridge.state_reader()},
                    )
                except Exception as exc:  # pragma: no cover - API boundary
                    self._reply(
                        HTTPStatus.INTERNAL_SERVER_ERROR,
                        {"ok": False, "error": str(exc)},
                    )

            def do_POST(self) -> None:  # noqa: N802
                if self.path.rstrip("/") != "/api/command":
                    self._reply(
                        HTTPStatus.NOT_FOUND,
                        {"ok": False, "error": "not found"},
                    )
                    return
                try:
                    size = int(self.headers.get("Content-Length", "0"))
                    if size <= 0 or size > 64 * 1024:
                        raise ValueError("请求正文大小无效")
                    payload = json.loads(self.rfile.read(size))
                    if not isinstance(payload, dict):
                        raise ValueError("命令必须是 JSON 对象")
                    result = bridge.command_handler(payload)
                    self._reply(HTTPStatus.OK, {"ok": True, **result})
                except (ValueError, KeyError, TypeError, json.JSONDecodeError) as exc:
                    self._reply(
                        HTTPStatus.BAD_REQUEST,
                        {"ok": False, "error": str(exc)},
                    )
                except Exception as exc:  # pragma: no cover - API boundary
                    self._reply(
                        HTTPStatus.INTERNAL_SERVER_ERROR,
                        {"ok": False, "error": str(exc)},
                    )

            # ----------------------------------------------------------
            # Server-Sent Events for the oscilloscope
            # ----------------------------------------------------------

            def _serve_sse(self) -> None:
                source = bridge.oscilloscope
                if source is None:
                    self._reply(
                        HTTPStatus.SERVICE_UNAVAILABLE,
                        {"ok": False, "error": "oscilloscope not available"},
                    )
                    return
                self.send_response(HTTPStatus.OK)
                self._send_cors()
                self.send_header("Content-Type", "text/event-stream")
                self.send_header("Cache-Control", "no-store")
                self.send_header("Connection", "keep-alive")
                self.send_header("X-Accel-Buffering", "no")
                self.end_headers()
                wfile = self.wfile
                try:
                    # Send initial metadata so the client knows joint count
                    joint_count = source.joint_count
                    init = json.dumps(
                        {
                            "type": "init",
                            "joint_count": joint_count,
                            "stream_hz": 1.0 / _STREAM_INTERVAL_S,
                        },
                        ensure_ascii=False,
                    )
                    wfile.write(f"data: {init}\n\n".encode("utf-8"))
                    wfile.flush()

                    last_t: float | None = None
                    while not bridge._closed:
                        frame = source.latest()
                        if frame is not None and frame.t != last_t:
                            last_t = frame.t
                            payload = json.dumps(
                                frame.as_json(), ensure_ascii=False
                            )
                            wfile.write(
                                f"data: {payload}\n\n".encode("utf-8")
                            )
                            wfile.flush()
                        time.sleep(_STREAM_INTERVAL_S)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    # Client disconnected — normal, no action needed
                    pass

            def log_message(self, format: str, *args: object) -> None:
                return

        self._server = ThreadingHTTPServer((host, port), Handler)
        self._closed = False
        self._thread = threading.Thread(
            target=self._server.serve_forever,
            name="e1pro-command-server",
            daemon=True,
        )

    @property
    def address(self) -> tuple[str, int]:
        host, port = self._server.server_address[:2]
        return str(host), int(port)

    def start(self) -> None:
        self._thread.start()

    def close(self) -> None:
        """Shut down the server and all active SSE connections."""
        self._closed = True
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)
