"""Dual-arm robot headless end-to-end checks used after installation."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from robot_framework.controller import Controller
from .model import DualArmUrdfModel

URDF_RELATIVE_PATH = Path(
    "e1_pro_full/eu_robot_describtion2/urdf/eu_robot_describtion2.urdf"
)


def resolve_urdf_path(project_root: Path) -> Path:
    """Return the delivered dual-arm URDF, newest first if several exist."""
    direct = project_root / URDF_RELATIVE_PATH
    if direct.is_file():
        return direct
    directory = project_root / "e1_pro_full"
    candidates = sorted(
        directory.rglob("eu_robot_describtion2*.urdf"),
        key=lambda path: (path.stat().st_mtime_ns, path.name),
        reverse=True,
    )
    if not candidates:
        raise FileNotFoundError(f"未在 {directory} 找到双臂 URDF 文件")
    return candidates[0]


def run_smoke_test(project_root: Path) -> int:
    checks: list[tuple[str, bool, str]] = []
    urdf_path = resolve_urdf_path(project_root)
    try:
        model = DualArmUrdfModel.from_urdf(urdf_path)
        checks.append(("双臂 URDF 加载", True, urdf_path.name))
        checks.append(
            (
                "中柱辅助轴",
                model.aux_joint_names == ("j1", "j2", "j3"),
                ", ".join(model.aux_joint_names),
            )
        )

        aux = model.aux_configuration(model.initial_configuration)
        model.set_active_arm("left")
        left_tcp = model.tcp_pose(model.stored_arm("left"), aux)
        model.set_active_arm("right")
        right_tcp = model.tcp_pose(model.stored_arm("right"), aux)
        span = abs(left_tcp[0, 3]) + abs(right_tcp[0, 3])
        checks.append(
            (
                "双臂对称展开可见",
                span >= 0.9 and left_tcp[2, 3] > 0.2 and right_tcp[2, 3] > 0.2,
                f"span={span:.3f} m, z_L={left_tcp[2, 3]:.3f} m, "
                f"z_R={right_tcp[2, 3]:.3f} m",
            )
        )

        controller = Controller(model, project_root / "last_solution.json")
        checks.append(
            ("双臂使用 SciPy IK", controller.solver.backend_name == "SciPy fallback",
             controller.solver.backend_name)
        )

        left_arm = controller.arm.copy()
        left_aux = controller.aux.copy()
        right_before = model.stored_arm("right").copy()
        target = controller.target.copy()
        target[:3, 3] += np.array([0.02, -0.01, 0.0])
        target[:3, :3] = (
            Rotation.from_rotvec([0.0, np.deg2rad(3.0), 0.0])
            * Rotation.from_matrix(target[:3, :3])
        ).as_matrix()
        controller.target = target
        solution = controller.solve(recovery_seeds=6)
        strict = (
            solution.position_error_m <= 0.002
            and solution.orientation_error_rad <= np.deg2rad(1.0)
        )
        checks.append(
            (
                "左臂独立 IK",
                strict,
                f"{solution.position_error_m * 1000:.3f} mm / "
                f"{np.rad2deg(solution.orientation_error_rad):.3f}°",
            )
        )
        checks.append(
            (
                "右臂值保持不变",
                np.allclose(model.stored_arm("right"), right_before, atol=1e-12),
                "right stored unchanged",
            )
        )
        checks.append(
            (
                "中柱辅助轴硬锁定",
                all(
                    left_aux[name] == controller.aux[name]
                    for name in model.aux_joint_names
                ),
                ", ".join(model.aux_joint_names),
            )
        )

        model.commit_active_arm(controller.arm)
        model.set_active_arm("left")
        checks.append(
            ("切换后左臂保留求解值", True, f"left active")
        )
    except Exception as exc:
        checks.append(("执行异常", False, f"{type(exc).__name__}: {exc}"))

    print("E1-PRO 双臂 smoke test")
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}: {detail}")
    failed = [name for name, passed, _ in checks if not passed]
    if failed:
        print(f"Smoke test failed: {', '.join(failed)}")
        return 1
    print("All smoke checks passed.")
    return 0
