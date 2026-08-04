"""MOVJ smooth joint-space interpolation."""

from __future__ import annotations

import numpy as np

from .models import JointTrajectory, TrajectoryError
from .profiles import quintic


def plan_joint_trajectory(
    start: np.ndarray,
    end: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    duration_s: float,
    frequency_hz: float,
) -> JointTrajectory:
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    lower = np.asarray(lower, dtype=float)
    upper = np.asarray(upper, dtype=float)
    if (
        start.shape != end.shape
        or start.shape != lower.shape
        or start.shape != upper.shape
    ):
        raise ValueError("joint vectors and limits must have matching shapes")
    if not all(np.all(np.isfinite(value)) for value in (start, end, lower, upper)):
        raise ValueError("joint trajectory contains non-finite values")
    if duration_s <= 0 or frequency_hz <= 0:
        raise ValueError("duration and frequency must be positive")
    if np.any(end < lower) or np.any(end > upper):
        raise TrajectoryError("MOVJ target exceeds a joint limit")

    count = max(2, int(np.ceil(duration_s * frequency_hz)) + 1)
    times = np.linspace(0.0, duration_s, count)
    smooth = quintic(np.linspace(0.0, 1.0, count))
    arms = [start + amount * (end - start) for amount in smooth]
    return JointTrajectory(times=times, arms=arms)
