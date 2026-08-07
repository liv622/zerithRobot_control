"""HTTP gateway serving a pendant UI and proxying simulator API calls."""

from __future__ import annotations

import json
import urllib.error
import urllib.request
from collections.abc import Callable
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


class PendantGatewayServer:
    def __init__(
        self,
        host: str,
        port: int,
        simulator_url: str,
        asset_reader: Callable[[str], tuple[bytes, str] | None],
        local_api: Callable[[str, str, dict], dict] | None = None,
    ) -> None:
        upstream = simulator_url.rstrip("/")

        class Handler(BaseHTTPRequestHandler):
            def _send(self, status: int, body: bytes, content_type: str) -> None:
                self.send_response(status)
                self.send_header("Content-Type", content_type)
                self.send_header("Content-Length", str(len(body)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(body)

            def do_GET(self) -> None:  # noqa: N802
                if self.path == "/api/state":
                    self._proxy("GET", b"")
                    return
                asset = asset_reader(self.path)
                if asset is None:
                    self._send(HTTPStatus.NOT_FOUND, b"not found", "text/plain")
                    return
                body, content_type = asset
                self._send(HTTPStatus.OK, body, content_type)

            def do_POST(self) -> None:  # noqa: N802
                if self.path != "/api/command":
                    self._send(HTTPStatus.NOT_FOUND, b"not found", "text/plain")
                    return
                size = int(self.headers.get("Content-Length", "0"))
                body = self.rfile.read(size)
                try:
                    command = json.loads(body)
                except json.JSONDecodeError:
                    command = {}
                if (
                    command.get("action") == "select_urdf"
                    and local_api is not None
                ):
                    try:
                        local_api("POST", "/api/can-reload-urdf", {})
                    except (OSError, RuntimeError, ValueError) as exc:
                        self._send(
                            HTTPStatus.BAD_REQUEST,
                            json.dumps(
                                {"ok": False, "error": str(exc)},
                                ensure_ascii=False,
                            ).encode("utf-8"),
                            "application/json; charset=utf-8",
                        )
                        return
                self._proxy("POST", body, command)

            def _proxy(
                self,
                method: str,
                body: bytes,
                command: dict | None = None,
            ) -> None:
                try:
                    request = urllib.request.Request(
                        upstream + self.path,
                        data=body if method == "POST" else None,
                        method=method,
                        headers={"Content-Type": "application/json"},
                    )
                    with urllib.request.urlopen(request, timeout=3.0) as response:
                        data = response.read()
                        if (
                            method == "POST"
                            and command is not None
                            and command.get("action") == "select_urdf"
                            and local_api is not None
                        ):
                            result = json.loads(data)
                            if result.get("ok") and result.get("path"):
                                reloaded = local_api(
                                    "POST",
                                    "/api/reload-urdf",
                                    {"path": result["path"]},
                                )
                                result.update(reloaded)
                                data = json.dumps(
                                    result, ensure_ascii=False
                                ).encode("utf-8")
                        self._send(
                            response.status,
                            data,
                            "application/json; charset=utf-8",
                        )
                except urllib.error.HTTPError as exc:
                    self._send(
                        exc.code,
                        exc.read(),
                        "application/json; charset=utf-8",
                    )
                except (urllib.error.URLError, TimeoutError) as exc:
                    data = json.dumps(
                        {
                            "ok": False,
                            "error": (
                                "无法连接仿真："
                                f"{getattr(exc, 'reason', exc)}"
                            ),
                        }
                    ).encode("utf-8")
                    self._send(
                        HTTPStatus.BAD_GATEWAY,
                        data,
                        "application/json; charset=utf-8",
                    )
                except (OSError, RuntimeError, ValueError) as exc:
                    self._send(
                        HTTPStatus.BAD_REQUEST,
                        json.dumps(
                            {"ok": False, "error": str(exc)},
                            ensure_ascii=False,
                        ).encode("utf-8"),
                        "application/json; charset=utf-8",
                    )

            def log_message(self, format: str, *args: object) -> None:
                return

        self.upstream = upstream
        self._server = ThreadingHTTPServer((host, port), Handler)

    @property
    def address(self) -> tuple[str, int]:
        host, port = self._server.server_address[:2]
        return str(host), int(port)

    def serve_forever(self) -> None:
        self._server.serve_forever()

    def close(self) -> None:
        self._server.server_close()
