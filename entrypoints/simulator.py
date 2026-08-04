"""Robot simulator command-line application."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from robots.registry import available_robot_keys, get_robot_plugin


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--smoke-test",
        action="store_true",
        help="run headless model, IK, trajectory, and joint-lock checks",
    )
    parser.add_argument(
        "--robot",
        choices=available_robot_keys(),
        default="e1pro",
        help="robot model plugin",
    )
    parser.add_argument("--host", default="0.0.0.0", help="Viser bind address")
    parser.add_argument("--port", type=int, default=8080, help="preferred Viser port")
    parser.add_argument(
        "--control-host",
        default="127.0.0.1",
        help="teach-pendant JSON API bind address",
    )
    parser.add_argument(
        "--control-port",
        type=int,
        default=8765,
        help="teach-pendant JSON API port",
    )
    return parser.parse_args()


def main(project_root: Path | None = None) -> int:
    root = (
        Path(__file__).resolve().parents[1]
        if project_root is None
        else project_root
    )
    args = parse_args()
    plugin = get_robot_plugin(args.robot)
    if args.smoke_test:
        return plugin.run_smoke_test(root)

    try:
        from interfaces.simulator import run_ui
    except ModuleNotFoundError as exc:
        if exc.name == "viser":
            print(
                "缺少 viser。请先执行 "
                "`python -m pip install -r requirements.txt`。",
                file=sys.stderr,
            )
            return 2
        raise

    run_ui(
        root,
        host=args.host,
        port=args.port,
        control_host=args.control_host,
        control_port=args.control_port,
        plugin=plugin,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
