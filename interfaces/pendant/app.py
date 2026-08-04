"""Robot teach-pendant web application."""

from __future__ import annotations

import webbrowser
from pathlib import Path

from communication import PendantGatewayServer
from .template import PENDANT_HTML


def _read_asset(
    path: str,
    viser_url: str = "http://127.0.0.1:8080",
) -> tuple[bytes, str] | None:
    if path in {"/", "/index.html"}:
        html = PENDANT_HTML.replace("__VISER_URL__", viser_url.rstrip("/"))
        return html.encode("utf-8"), "text/html; charset=utf-8"
    assets = {
        "/assets/pendant.css": ("pendant.css", "text/css; charset=utf-8"),
        "/assets/pendant.js": (
            "pendant.js",
            "text/javascript; charset=utf-8",
        ),
    }
    if path in assets:
        name, content_type = assets[path]
        body = (Path(__file__).parent / "assets" / name).read_bytes()
        return body, content_type
    return None


def run_pendant(
    host: str,
    port: int,
    simulator_url: str,
    viser_url: str,
    open_browser: bool,
) -> None:
    server = PendantGatewayServer(
        host,
        port,
        simulator_url,
        lambda path: _read_asset(path, viser_url),
    )
    actual_host, actual_port = server.address
    browser_host = "127.0.0.1" if actual_host in {"0.0.0.0", "::"} else actual_host
    url = f"http://{browser_host}:{actual_port}"
    print(f"示教器：{url}")
    print(f"仿真通信：{server.upstream}")
    if open_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("正在退出示教器。")
    finally:
        server.close()
