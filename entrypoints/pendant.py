"""Robot teach-pendant command-line application."""

from __future__ import annotations

import argparse
import atexit
import os
import signal
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path
from urllib.parse import urlsplit

from interfaces.pendant.app import run_pendant
from robots.registry import get_robot_plugin


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
        + (["--urdf", urdf] if urdf else []),
        # The pendant owns this process group.  This makes a URDF reload able
        # to stop every Viser child cleanly without touching the pendant
        # process itself.
        start_new_session=True,
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


def _stop_simulator_tree(process: subprocess.Popen | None) -> None:
    """Kill the process and its entire process group.

    This ensures the simulator (which may itself spawn threads and
    subprocesses for viser) is fully cleaned up on every exit path.
    """
    if process is None:
        return
    if process.poll() is not None:
        return
    pid = process.pid
    if pid is None:
        _stop_simulator(process)
        return
    try:
        # Let the simulator handle Ctrl+C first so its command server and
        # Viser listener release their ports before the replacement starts.
        # SIGTERM remains the fallback for a blocked process.
        os.killpg(pid, signal.SIGINT)
        try:
            process.wait(timeout=2.0)
            return
        except subprocess.TimeoutExpired:
            pass
        os.killpg(pid, signal.SIGTERM)
        try:
            process.wait(timeout=2.0)
            return
        except subprocess.TimeoutExpired:
            pass
        # Escalate to SIGKILL only for stragglers.
        os.killpg(pid, signal.SIGKILL)
        process.wait(timeout=2.0)
    except (ProcessLookupError, OSError):
        # Already dead or not in a group — fall back to single-process kill.
        _stop_simulator(process)


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
    project_root = Path(__file__).resolve().parents[1]
    simulator_ref: dict[str, subprocess.Popen | None] = {"process": None}
    resolved_viser_url = args.viser_url
    # Keep the path owned by the currently running child.  In particular, a
    # request to load the already running URDF must not tear down the control
    # endpoint and its Viser WebSocket just to recreate identical state.
    active_urdf = (
        (
            Path(args.urdf)
            if Path(args.urdf).is_absolute()
            else project_root / Path(args.urdf)
        ).resolve()
        if args.urdf
        else get_robot_plugin(args.robot).resolve_urdf_path(project_root).resolve()
    )

    def _cleanup() -> None:
        """Guaranteed cleanup — runs on normal exit, exception, and SIGTERM."""
        sim = simulator_ref.get("process")
        if sim is None:
            return
        # Detach from atexit so we don't call twice.
        simulator_ref["process"] = None
        _stop_simulator_tree(sim)

    # Register cleanup for every exit path: normal return, unhandled
    # exception, SIGTERM (kill / IDE stop), and SIGINT (Ctrl+C).
    atexit.register(_cleanup)

    def _signal_handler(signum: int, _frame: object) -> None:
        print(f"\n收到信号 {signum}，正在退出……", file=sys.stderr)
        sys.exit(128 + signum)

    signal.signal(signal.SIGTERM, _signal_handler)
    # SIGINT is already handled by the KeyboardInterrupt path, but
    # registering a handler ensures atexit still fires if something
    # swallows the exception.

    def local_api(method: str, path: str, payload: dict) -> dict:
        nonlocal active_urdf, resolved_viser_url
        if method == "POST" and path == "/api/can-reload-urdf":
            if args.no_simulator:
                raise ValueError("外部仿真模式不能从示教器切换 URDF")
            return {}
        if method == "POST" and path == "/api/reload-urdf":
            if args.no_simulator:
                raise ValueError("外部仿真模式不能从示教器切换 URDF")
            selected = Path(str(payload.get("path", ""))).resolve()
            if not selected.is_file() or selected.suffix.lower() != ".urdf":
                raise ValueError("仿真器返回的 URDF 路径无效")
            current = simulator_ref["process"]
            if selected == active_urdf and current is not None and current.poll() is None:
                return {
                    "message": f"{selected.stem} 已在仿真中运行，无需重启",
                    "viser_url": resolved_viser_url,
                    "active_path": str(selected),
                    "reloaded": False,
                }
            previous = active_urdf
            old = simulator_ref["process"]
            simulator_ref["process"] = None
            _stop_simulator_tree(old)
            try:
                simulator_ref["process"], resolved_viser_url = _start_simulator(
                    robot=args.robot,
                    urdf=str(selected),
                    simulator_url=args.sim_url,
                    viser_url=args.viser_url,
                )
            except (OSError, RuntimeError, ValueError) as exc:
                # A bad URDF must not leave the pendant with no simulator at
                # all.  Recreate the last known-good model before reporting
                # the failed selection to the operator.
                try:
                    simulator_ref["process"], resolved_viser_url = _start_simulator(
                        robot=args.robot,
                        urdf=str(previous),
                        simulator_url=args.sim_url,
                        viser_url=args.viser_url,
                    )
                except (OSError, RuntimeError, ValueError) as restore_exc:
                    raise RuntimeError(
                        f"加载 {selected.name} 失败，且无法恢复原模型 {previous.name}："
                        f"{restore_exc}"
                    ) from exc
                raise RuntimeError(
                    f"加载 {selected.name} 失败，已恢复原模型 {previous.name}：{exc}"
                ) from exc
            active_urdf = selected
            return {
                "message": f"已加载 {selected.stem}",
                "viser_url": resolved_viser_url,
                "active_path": str(selected),
                "reloaded": True,
            }
        raise ValueError("未知本地示教器请求")

    try:
        if not args.no_simulator:
            simulator_ref["process"], resolved_viser_url = _start_simulator(
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
            local_api,
        )
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"示教器启动失败：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
