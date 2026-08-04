"""JSON/HTTP command adapter for a robot simulation application."""

from __future__ import annotations

import json
import threading
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any


JsonObject = dict[str, Any]


class SimulationCommandServer:
    """Expose application state and commands over a small local HTTP API."""

    def __init__(
        self,
        host: str,
        port: int,
        state_reader: Callable[[], JsonObject],
        command_handler: Callable[[JsonObject], JsonObject],
    ) -> None:
        self.state_reader = state_reader
        self.command_handler = command_handler
        bridge = self

        class Handler(BaseHTTPRequestHandler):
            def _reply(self, status: int, payload: JsonObject) -> None:
                body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
                self.send_response(status)
                self.send_header("Content-Type", "application/json; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_OPTIONS(self) -> None:  # noqa: N802
                self._reply(HTTPStatus.NO_CONTENT, {})

            def do_GET(self) -> None:  # noqa: N802
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

            def log_message(self, format: str, *args: object) -> None:
                return

        self._server = ThreadingHTTPServer((host, port), Handler)
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
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)
