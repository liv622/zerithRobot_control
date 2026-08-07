"""Generic robot solve state and result persistence."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from robot_logging import get_logger
from .model_protocol import RobotModelProtocol
from .solver import IKSolution, IKSolver

_logger = get_logger("robot_framework.controller")

# Minimum wall-clock gap between two solution-file writes.  The file is a
# diagnostic snapshot, not a trajectory log, so writing it at the full
# interpolation rate would add disk I/O to every control period for no benefit.
_SAVE_INTERVAL_S = 0.10


class Controller:
    def __init__(self, model: RobotModelProtocol, output_path: Path) -> None:
        self.model = model
        try:
            from .pyroki_backend import create_solver

            self.solver = create_solver(model)
        except ModuleNotFoundError:
            self.solver = IKSolver(model)
        self.output_path = output_path
        self._save_lock = threading.Lock()
        self._last_save_s = 0.0
        # Protects concurrent reads/writes to ``arm`` — the motion loop
        # mutates elements or replaces the whole array while the oscilloscope
        # sampler reads a snapshot; both operations release the GIL at the C
        # level, so without this lock ``copy()`` can return a torn view.
        self._arm_lock = threading.Lock()
        self.arm = model.arm_vector(model.initial_configuration)
        self.aux = model.aux_configuration(model.initial_configuration)
        self.guide = self.arm.copy()
        self.target = model.tcp_pose(self.arm, self.aux)
        self.solution: IKSolution | None = None

    def arm_snapshot(self) -> np.ndarray:
        """Thread-safe read of the joint-position vector.

        Call this from any thread (sampler, diagnostics) to get a consistent
        copy even while the motion loop is updating individual elements.
        """
        with self._arm_lock:
            return self.arm.copy()

    def reset(self) -> None:
        self.arm = self.model.arm_vector(self.model.initial_configuration)
        self.aux = self.model.aux_configuration(
            self.model.initial_configuration
        )
        self.guide = self.arm.copy()
        self.target = self.model.tcp_pose(self.arm, self.aux)
        self.solve()

    def set_target_xyz_rpy(
        self, xyz: np.ndarray, rpy_degrees: np.ndarray
    ) -> None:
        self.target = self.model.pose(
            np.asarray(xyz, dtype=float),
            Rotation.from_euler("xyz", rpy_degrees, degrees=True),
        )

    def target_xyz_rpy(self) -> tuple[np.ndarray, np.ndarray]:
        xyz = self.target[:3, 3].copy()
        rpy = Rotation.from_matrix(self.target[:3, :3]).as_euler("xyz", degrees=True)
        return xyz, rpy

    def solve(
        self,
        *,
        lock_orientation: bool = True,
        guide_enabled: bool = False,
        guide_strength: float = 0.05,
        recovery_seeds: int = 10,
        force_recovery: bool = False,
        multi_start: bool = True,
        smooth_strength: float = 0.3,
        velocity_limit_dt: float | None = None,
        manipulability_weight: float = 0.0,
    ) -> IKSolution:
        aux_before = self.aux.copy()
        solution = self.solver.solve(
            self.target,
            self.arm,
            self.aux,
            lock_orientation=lock_orientation,
            guide=self.guide if guide_enabled else None,
            guide_strength=guide_strength,
            multi_start=multi_start,
            recovery_seeds=recovery_seeds,
            force_recovery=force_recovery,
            smooth_strength=smooth_strength,
            velocity_limit_dt=velocity_limit_dt,
            manipulability_weight=manipulability_weight,
        )
        self.solver.assert_aux_unchanged(aux_before, self.aux)
        self.arm = solution.arm
        self.solution = solution
        self.save()
        return solution

    def save_if_due(self, *, force: bool = False) -> bool:
        """Persist the solution snapshot at most every ``_SAVE_INTERVAL_S``.

        Motion loops call this instead of :meth:`save` so a 200 Hz trajectory
        does not turn into 200 file writes per second.  Returns whether a write
        actually happened.
        """
        now = time.monotonic()
        with self._save_lock:
            if not force and now - self._last_save_s < _SAVE_INTERVAL_S:
                return False
            self._last_save_s = now
        self.save()
        return True

    def save(self) -> None:
        if self.solution is None:
            return
        xyz, rpy = self.target_xyz_rpy()
        actual = self.model.tcp_pose(self.arm, self.aux)
        actual_rpy = Rotation.from_matrix(actual[:3, :3]).as_euler("xyz", degrees=True)
        payload = {
            "target": {
                "position_m": xyz.tolist(),
                "rpy_degrees": rpy.tolist(),
            },
            "actual": {
                "position_m": actual[:3, 3].tolist(),
                "rpy_degrees": actual_rpy.tolist(),
            },
            "reachable": self.solution.reachable,
            "position_error_mm": self.solution.position_error_m * 1000.0,
            "orientation_error_degrees": np.rad2deg(
                self.solution.orientation_error_rad
            ),
            "attempts": self.solution.attempts,
            "recovered": self.solution.recovered,
            "arm_joints_rad": self.solution.joint_dict(),
            "arm_joints_degrees": {
                name: float(np.rad2deg(value))
                for name, value in zip(self.model.arm_joint_names, self.arm)
            },
            "auxiliary_joints": {
                name: float(self.aux[name])
                for name in self.model.aux_joint_names
            },
        }
        # Written via a temporary file and renamed so a reader never observes a
        # half-written snapshot.  A failing write is logged rather than raised:
        # this file is diagnostic output, and losing it must not abort a motion.
        temporary = self.output_path.with_suffix(".tmp")
        try:
            temporary.write_text(
                json.dumps(payload, indent=2, ensure_ascii=False) + "\n",
                encoding="utf-8",
            )
            temporary.replace(self.output_path)
        except OSError as exc:
            _logger.warning("求解结果写入失败：%s", exc)
            temporary.unlink(missing_ok=True)
