"""Generic bounded numerical IK with multi-start recovery."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .model_protocol import RobotModelProtocol


@dataclass
class IKSolution:
    arm: np.ndarray
    position_error_m: float
    orientation_error_rad: float
    reachable: bool
    attempts: int
    recovered: bool
    joint_names: tuple[str, ...] = ()

    def joint_dict(self) -> dict[str, float]:
        names = self.joint_names or tuple(
            f"joint_{index + 1}" for index in range(len(self.arm))
        )
        return {name: float(value) for name, value in zip(names, self.arm)}


class IKSolver:
    backend_name = "SciPy fallback"

    def __init__(self, model: RobotModelProtocol) -> None:
        self.model = model
        self.last_good: np.ndarray | None = None
        self.batch_index = 0

    @staticmethod
    def pose_errors(actual: np.ndarray, target: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        position = actual[:3, 3] - target[:3, 3]
        orientation = Rotation.from_matrix(
            actual[:3, :3] @ target[:3, :3].T
        ).as_rotvec()
        return position, orientation

    def solve(
        self,
        target: np.ndarray,
        seed: np.ndarray,
        aux: dict[str, float],
        *,
        lock_orientation: bool = True,
        guide: np.ndarray | None = None,
        guide_strength: float = 0.05,
        multi_start: bool = True,
        recovery_seeds: int = 10,
        force_recovery: bool = False,
    ) -> IKSolution:
        seed = np.clip(np.asarray(seed, dtype=float), self.model.lower, self.model.upper)
        guide_value = seed if guide is None else np.asarray(guide, dtype=float)

        def residual(arm: np.ndarray) -> np.ndarray:
            position, orientation = self.pose_errors(
                self.model.tcp_pose(arm, aux), target
            )
            parts = [position / 0.01]
            if lock_orientation:
                parts.append(orientation / np.deg2rad(5.0))
            if guide is not None and guide_strength > 0:
                parts.append((arm - guide_value) * np.sqrt(guide_strength))
            return np.concatenate(parts)

        def run(start: np.ndarray) -> IKSolution:
            result = least_squares(
                residual,
                np.clip(start, self.model.lower, self.model.upper),
                bounds=(self.model.lower, self.model.upper),
                max_nfev=350,
                xtol=1e-10,
                ftol=1e-10,
                gtol=1e-10,
            )
            position, orientation = self.pose_errors(
                self.model.tcp_pose(result.x, aux), target
            )
            position_norm = float(np.linalg.norm(position))
            orientation_norm = float(np.linalg.norm(orientation)) if lock_orientation else 0.0
            return IKSolution(
                arm=result.x,
                position_error_m=position_norm,
                orientation_error_rad=orientation_norm,
                reachable=bool(
                    position_norm <= 0.010
                    and (
                        not lock_orientation
                        or orientation_norm <= np.deg2rad(10.0)
                    )
                ),
                attempts=1,
                recovered=False,
                joint_names=self.model.arm_joint_names,
            )

        best = run(seed)
        strict_failure = best.position_error_m > 0.002 or (
            lock_orientation and best.orientation_error_rad > np.deg2rad(1.0)
        )
        if multi_start and (strict_failure or force_recovery):
            starts = self._recovery_starts(seed, guide_value, recovery_seeds)
            for start in starts:
                candidate = run(start)
                best.attempts += 1
                if self._score(candidate, lock_orientation) < self._score(
                    best, lock_orientation
                ):
                    candidate.attempts = best.attempts
                    candidate.recovered = True
                    best = candidate

        if best.position_error_m <= 0.002 and (
            not lock_orientation or best.orientation_error_rad <= np.deg2rad(1.0)
        ):
            self.last_good = best.arm.copy()
        if force_recovery:
            self.batch_index += 1
        return best

    @staticmethod
    def _score(solution: IKSolution, lock_orientation: bool) -> float:
        score = solution.position_error_m / 0.002
        if lock_orientation:
            score += solution.orientation_error_rad / np.deg2rad(1.0)
        return score

    def _recovery_starts(
        self, seed: np.ndarray, guide: np.ndarray, count: int
    ) -> list[np.ndarray]:
        midpoint = (self.model.lower + self.model.upper) / 2.0
        starts = [seed, guide, midpoint]
        if self.last_good is not None:
            starts.insert(0, self.last_good)
        mirrored = seed.copy()
        mirrored[np.arange(0, len(mirrored), 2)] *= -1.0
        starts.append(np.clip(mirrored, self.model.lower, self.model.upper))
        rng = np.random.default_rng(260714 + self.batch_index)
        for _ in range(max(0, count)):
            starts.append(rng.uniform(self.model.lower, self.model.upper))
        return starts

    def assert_aux_unchanged(
        self,
        before: dict[str, float],
        after: dict[str, float],
    ) -> None:
        for name in self.model.aux_joint_names:
            if before[name] != after[name]:
                raise AssertionError(f"IK changed locked auxiliary joint {name}")
