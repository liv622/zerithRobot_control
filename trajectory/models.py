"""Robot trajectory result types and domain errors."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from robot_framework.solver import IKSolution


class TrajectoryError(RuntimeError):
    """Raised when a requested robot trajectory is unsafe or infeasible."""


@dataclass
class JointTrajectory:
    """A time-stamped joint-space trajectory.

    ``velocities`` and ``accelerations`` are part of the result rather than
    derived by the caller because real hardware needs them: a PD feedforward
    controller consumes commanded velocity directly, and the safety check that
    guards a move needs acceleration without re-differentiating positions.
    """

    times: np.ndarray
    arms: list[np.ndarray]
    velocities: list[np.ndarray] = field(default_factory=list)
    accelerations: list[np.ndarray] = field(default_factory=list)
    profile_name: str = ""

    @property
    def duration_s(self) -> float:
        return float(self.times[-1]) if len(self.times) else 0.0

    def peak_velocity(self) -> np.ndarray:
        if not self.velocities:
            return np.zeros_like(self.arms[0]) if self.arms else np.zeros(0)
        return np.max(np.abs(np.asarray(self.velocities)), axis=0)

    def peak_acceleration(self) -> np.ndarray:
        if not self.accelerations:
            return np.zeros_like(self.arms[0]) if self.arms else np.zeros(0)
        return np.max(np.abs(np.asarray(self.accelerations)), axis=0)


@dataclass
class Trajectory:
    """A Cartesian trajectory together with its solved joint configurations."""

    times: np.ndarray
    poses: list[np.ndarray]
    solutions: list[IKSolution]
    path_velocities: np.ndarray | None = None
    profile_name: str = ""

    @property
    def duration_s(self) -> float:
        return float(self.times[-1]) if len(self.times) else 0.0

    def joint_trajectory(self) -> JointTrajectory:
        """Expose the solved arm samples as a joint-space trajectory.

        Joint velocity is differentiated from the IK solutions rather than
        mapped through the Jacobian: the samples are what will actually be
        commanded, so differentiating them is what reveals the joint speed the
        robot will really see near a singularity.
        """
        arms = [solution.arm.copy() for solution in self.solutions]
        velocities: list[np.ndarray] = []
        accelerations: list[np.ndarray] = []
        if len(arms) >= 2:
            stacked = np.asarray(arms)
            velocities = list(
                np.gradient(stacked, self.times, axis=0, edge_order=1)
            )
            accelerations = list(
                np.gradient(
                    np.asarray(velocities), self.times, axis=0, edge_order=1
                )
            )
        return JointTrajectory(
            times=self.times,
            arms=arms,
            velocities=velocities,
            accelerations=accelerations,
            profile_name=self.profile_name,
        )
