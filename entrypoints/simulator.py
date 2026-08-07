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
    parser.add_argument(
        "--urdf",
        type=Path,
        help="要加载的 URDF（绝对路径，或相对于项目根目录）；指定后自动按 URDF 解析机械臂末端",
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
    urdf_path = None
    if args.urdf is not None:
        urdf_path = args.urdf if args.urdf.is_absolute() else root / args.urdf
        urdf_path = urdf_path.resolve()
        if not urdf_path.is_file():
            print(f"URDF 不存在：{urdf_path}", file=sys.stderr)
            return 2
        from dataclasses import replace

        from robots.e1pro_dual.model import DualArmUrdfModel
        from robots.generic import GenericUrdfRobotModel
        from urdf import is_dual_arm_urdf

        load_model = (
            DualArmUrdfModel.from_urdf
            if is_dual_arm_urdf(urdf_path)
            else GenericUrdfRobotModel.from_urdf
        )
        plugin = replace(
            plugin,
            key="urdf",
            display_name=urdf_path.stem,
            load_model=load_model,
            urdf_relative_path=urdf_path,
        )
    if args.smoke_test:
        if urdf_path is not None:
            print("--smoke-test 目前仅适用于内置 E1-PRO 模型", file=sys.stderr)
            return 2
        return plugin.run_smoke_test(root)

    try:
        from interfaces.simulator import run_ui

        run_ui(
            root,
            host=args.host,
            port=args.port,
            control_host=args.control_host,
            control_port=args.control_port,
            plugin=plugin,
            urdf_path=urdf_path,
        )
    except ModuleNotFoundError as exc:
        if exc.name == "viser":
            print(
                "缺少 viser。请先执行 "
                "`python -m pip install -r requirements.txt`。",
                file=sys.stderr,
            )
            return 2
        raise
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
