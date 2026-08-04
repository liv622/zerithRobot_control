"""E1-PRO headless end-to-end checks used after installation."""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from .model import (
    ARM_JOINTS,
    AUX_JOINTS,
    INITIAL_CONFIGURATION,
    TCP_EXTENSION_M,
    RobotModel,
)
from robot_framework.controller import Controller
from robot_framework.solver import IKSolver
from trajectory import plan_trajectory

URDF_RELATIVE_PATH = Path(
    "e1_pro_full/urdf/E1-PRO_EVT2.0_V9_260714.urdf"
)


def run_smoke_test(project_root: Path) -> int:
    checks: list[tuple[str, bool, str]] = []
    urdf_path = project_root / URDF_RELATIVE_PATH
    try:
        model = RobotModel.from_urdf(urdf_path)
        checks.append(("URDF 与关节限位加载", True, urdf_path.name))

        arm = model.arm_vector(INITIAL_CONFIGURATION)
        aux = model.aux_configuration(INITIAL_CONFIGURATION)
        flange = model.flange_pose(arm, aux)
        tcp = model.tcp_pose(arm, aux)
        offset = flange[:3, :3].T @ (tcp[:3, 3] - flange[:3, 3])
        checks.append(
            (
                "真实 TCP 延伸",
                bool(np.allclose(offset, [0, 0, TCP_EXTENSION_M], atol=1e-9)),
                f"{offset[2] * 1000:.2f} mm",
            )
        )

        output_path = project_root / "last_solution.json"
        controller = Controller(model, output_path)
        checks.append(
            (
                "IK 后端",
                True,
                controller.solver.backend_name,
            )
        )
        target = controller.target.copy()
        target[:3, 3] += np.array([0.020, 0.010, -0.010])
        target[:3, :3] = (
            Rotation.from_rotvec([0.0, np.deg2rad(2.0), 0.0])
            * Rotation.from_matrix(target[:3, :3])
        ).as_matrix()
        controller.target = target
        aux_before = controller.aux.copy()
        solution = controller.solve(recovery_seeds=6)
        strict = (
            solution.position_error_m <= 0.002
            and solution.orientation_error_rad <= np.deg2rad(1.0)
        )
        checks.append(
            (
                "7 轴 TCP IK",
                strict,
                f"{solution.position_error_m * 1000:.3f} mm / "
                f"{np.rad2deg(solution.orientation_error_rad):.3f}°",
            )
        )
        checks.append(
            (
                "非机械臂关节硬锁定",
                all(aux_before[name] == controller.aux[name] for name in AUX_JOINTS),
                ", ".join(AUX_JOINTS),
            )
        )

        guides = (
            np.array([-1.0, -1.1, 1.2, -2.0, -0.1, -0.65, -0.9]),
            np.array([-0.25, -0.5, 0.3, -2.0, -0.02, -0.61, -0.15]),
        )
        redundant_solutions = [
            IKSolver(model).solve(
                model.tcp_pose(arm, aux),
                guide,
                aux,
                guide=guide,
                guide_strength=0.02,
                multi_start=False,
            )
            for guide in guides
        ]
        posture_difference = float(
            np.linalg.norm(
                redundant_solutions[0].arm - redundant_solutions[1].arm
            )
        )
        redundant_ok = all(
            item.position_error_m <= 0.002
            and item.orientation_error_rad <= np.deg2rad(1.0)
            for item in redundant_solutions
        ) and posture_difference >= 0.25
        checks.append(
            (
                "同一 TCP 的冗余臂形",
                redundant_ok,
                f"joint-space difference {posture_difference:.3f} rad",
            )
        )

        start_pose = model.tcp_pose(controller.arm, controller.aux)
        middle_pose = start_pose.copy()
        middle_pose[:3, 3] += np.array([0.008, -0.004, 0.004])
        middle_pose[:3, :3] = (
            Rotation.from_rotvec([np.deg2rad(1.0), 0.0, 0.0])
            * Rotation.from_matrix(middle_pose[:3, :3])
        ).as_matrix()
        end_pose = middle_pose.copy()
        end_pose[:3, 3] += np.array([0.006, 0.006, -0.003])
        trajectory = plan_trajectory(
            controller.solver,
            [start_pose, middle_pose, end_pose],
            controller.arm,
            controller.aux,
            duration_s=1.0,
            frequency_hz=12.0,
        )
        largest_step = max(
            (
                float(np.max(np.abs(second.arm - first.arm)))
                for first, second in zip(
                    trajectory.solutions[:-1], trajectory.solutions[1:]
                )
            ),
            default=0.0,
        )
        checks.append(
            (
                "MOVL 姿态轨迹",
                len(trajectory.solutions) >= 3
                and largest_step <= np.deg2rad(15.0),
                f"{len(trajectory.solutions)} samples, "
                f"max step {np.rad2deg(largest_step):.2f}°",
            )
        )

        saved = json.loads(output_path.read_text(encoding="utf-8"))
        checks.append(
            (
                "last_solution.json",
                set(saved["arm_joints_rad"]) == set(ARM_JOINTS),
                str(output_path),
            )
        )
    except Exception as exc:
        checks.append(("执行异常", False, f"{type(exc).__name__}: {exc}"))

    print("E1-PRO smoke test")
    for name, passed, detail in checks:
        print(f"  [{'PASS' if passed else 'FAIL'}] {name}: {detail}")
    failed = [name for name, passed, _ in checks if not passed]
    if failed:
        print(f"Smoke test failed: {', '.join(failed)}")
        return 1
    print("All smoke checks passed.")
    return 0
