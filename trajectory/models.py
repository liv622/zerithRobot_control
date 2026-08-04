"""Robot trajectory result types and domain errors."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from robot_framework.solver import IKSolution


@dataclass
class Trajectory:
    times: np.ndarray
    poses: list[np.ndarray]
    solutions: list[IKSolution]


@dataclass
class JointTrajectory:
    times: np.ndarray
    arms: list[np.ndarray]


class TrajectoryError(RuntimeError):
    """Raised when a requested robot trajectory is unsafe or infeasible."""
