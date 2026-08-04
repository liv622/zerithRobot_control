"""Redundant-arm null-space motion while locking the current TCP pose."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation

from robot_framework.controller import Controller
from robot_framework.model_protocol import RobotModelProtocol
from robot_framework.solver import IKSolution

from .contracts import ApplicationEvents, ApplicationSettings


class NullSpaceMotionService:
    """Drive joint preferences while PyRoki keeps the TCP pose constrained."""

    def __init__(
        self,
        model: RobotModelProtocol,
        controller: Controller,
        settings: ApplicationSettings,
        events: ApplicationEvents,
    ) -> None:
        self.model = model
        self.controller = controller
        self.settings = settings
        self.events = events
        self._locked_target: np.ndarray | None = None
        self._null_direction: np.ndarray | None = None

    @property
    def active(self) -> bool:
        return self._locked_target is not None

    def set_events(self, events: ApplicationEvents) -> None:
        self.events = events

    def begin(self) -> None:
        self._locked_target = self.model.tcp_pose(
            self.controller.arm,
            self.controller.aux,
        ).copy()
        self.controller.target = self._locked_target.copy()
        self.controller.guide = self.controller.arm.copy()
        self._null_direction = self._calculate_null_direction(
            self.controller.arm,
        )
        self.events.target_changed()
        self.events.guide_changed()

    def _calculate_null_direction(self, arm: np.ndarray) -> np.ndarray:
        base = self.model.tcp_pose(arm, self.controller.aux)
        epsilon = 1e-5
        jacobian = np.empty((6, len(arm)), dtype=float)
        for index in range(len(arm)):
            displaced = arm.copy()
            displaced[index] = np.clip(
                displaced[index] + epsilon,
                self.model.lower[index],
                self.model.upper[index],
            )
            actual_step = displaced[index] - arm[index]
            if abs(actual_step) < epsilon / 2:
                displaced[index] = np.clip(
                    arm[index] - epsilon,
                    self.model.lower[index],
                    self.model.upper[index],
                )
                actual_step = displaced[index] - arm[index]
            pose = self.model.tcp_pose(displaced, self.controller.aux)
            jacobian[:3, index] = (
                pose[:3, 3] - base[:3, 3]
            ) / actual_step
            jacobian[3:, index] = Rotation.from_matrix(
                pose[:3, :3] @ base[:3, :3].T
            ).as_rotvec() / actual_step
        direction = np.linalg.svd(jacobian, full_matrices=True)[2][-1]
        direction /= np.linalg.norm(direction)
        largest = int(np.argmax(np.abs(direction)))
        if direction[largest] < 0:
            direction *= -1.0
        return direction

    def step(self, delta_degrees: float) -> IKSolution:
        if self._locked_target is None or self._null_direction is None:
            raise ValueError("零空间运动尚未锁定 TCP")
        if not np.isfinite(delta_degrees):
            raise ValueError("零空间关节增量无效")

        guide = self.controller.guide.copy()
        requested = guide + (
            self._null_direction * np.deg2rad(delta_degrees)
        )
        guide = np.clip(
            requested,
            self.model.lower,
            self.model.upper,
        )
        if np.allclose(guide, self.controller.guide, atol=1e-12):
            raise ValueError("零空间运动已到关节参考限位")

        solution = self.controller.solver.solve(
            self._locked_target,
            self.controller.arm,
            self.controller.aux,
            lock_orientation=True,
            guide=guide,
            guide_strength=max(0.08, self.settings.guide_strength),
            multi_start=False,
        )
        if (
            solution.position_error_m > 0.002
            or solution.orientation_error_rad > np.deg2rad(1.0)
        ):
            raise ValueError(
                "零空间运动停止：无法继续保持 TCP 位置和姿态"
            )

        self.controller.guide = guide
        self.controller.arm = solution.arm.copy()
        self.controller.target = self._locked_target.copy()
        self.controller.solution = solution
        self.controller.save()
        self.events.guide_changed()
        self.events.solution_changed(solution)
        self.events.scene_changed()
        self.events.motion_sample(self.controller.arm.copy())
        return solution

    def end(self) -> None:
        self._locked_target = None
        self._null_direction = None
