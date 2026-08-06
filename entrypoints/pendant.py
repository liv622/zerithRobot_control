"""Robot teach-pendant command-line application."""

from __future__ import annotations

import argparse
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from urllib.parse import urlsplit

from interfaces.pendant.app import run_pendant


def _host_port(url: str, default_port: int) -> tuple[str, int]:
    parsed = urlsplit(url)
    if not parsed.hostname:
        raise ValueError(f"地址缺少主机名：{url}")
    return parsed.hostname, parsed.port or default_port


def _start_simulator(
    *,
    robot: str,
    urdf: str | None,
    simulator_url: str,
    viser_url: str,
) -> tuple[subprocess.Popen, str]:
    control_host, control_port = _host_port(simulator_url, 8765)
    viser_parts = urlsplit(viser_url)
    viser_host, requested_port = _host_port(viser_url, 8080)
    try:
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", requested_port))
            viser_port = requested_port
    except OSError:
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 0))
            viser_port = int(probe.getsockname()[1])
        print(
            f"Viser 端口 {requested_port} 已占用，自动改用 {viser_port}。"
        )
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "entrypoints.simulator",
            "--robot",
            robot,
            "--host",
            "127.0.0.1",
            "--port",
            str(viser_port),
            "--control-host",
            control_host,
            "--control-port",
            str(control_port),
        ]
        + (["--urdf", urdf] if urdf else [])
    )
    state_url = simulator_url.rstrip("/") + "/api/state"
    deadline = time.monotonic() + 15.0
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(
                f"仿真进程启动失败，退出码：{process.returncode}"
            )
        try:
            with urllib.request.urlopen(state_url, timeout=0.5):
                display_host = (
                    "127.0.0.1"
                    if viser_host in {"0.0.0.0", "::"}
                    else viser_host
                )
                resolved_url = (
                    f"{viser_parts.scheme or 'http'}://"
                    f"{display_host}:{viser_port}"
                )
                return process, resolved_url
        except (urllib.error.URLError, TimeoutError, OSError):
            time.sleep(0.15)
    process.terminate()
    raise RuntimeError(f"等待仿真通信接口超时：{state_url}")


def _stop_simulator(process: subprocess.Popen | None) -> None:
    if process is None or process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=3.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="0.0.0.0", help="示教器页面监听地址")
    parser.add_argument("--port", type=int, default=8090, help="示教器页面端口")
    parser.add_argument(
        "--sim-url",
        default="http://127.0.0.1:8765",
        help="仿真 JSON 通信地址",
    )
    parser.add_argument(
        "--viser-url",
        default="http://127.0.0.1:8080",
        help="Viser 三维场景地址",
    )
    parser.add_argument(
        "--robot",
        default="e1pro",
        help="机器人型号插件",
    )
    parser.add_argument(
        "--urdf",
        help="要加载的 URDF（传给仿真器）；指定后按 URDF 自动识别机械臂链和末端",
    )
    parser.add_argument(
        "--no-simulator",
        action="store_true",
        help="不自动启动仿真，连接已经运行的外部仿真进程",
    )
    parser.add_argument("--open", action="store_true", help="启动后打开浏览器")
    args = parser.parse_args()
    simulator = None
    resolved_viser_url = args.viser_url
    try:
        if not args.no_simulator:
            simulator, resolved_viser_url = _start_simulator(
                robot=args.robot,
                urdf=args.urdf,
                simulator_url=args.sim_url,
                viser_url=args.viser_url,
            )
        run_pendant(
            args.host,
            args.port,
            args.sim_url,
            resolved_viser_url,
            args.open,
        )
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"示教器启动失败：{exc}", file=sys.stderr)
        return 2
    finally:
        _stop_simulator(simulator)


if __name__ == "__main__":
    raise SystemExit(main())
