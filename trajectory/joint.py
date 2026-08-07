"""MOVJ joint-space interpolation using a double-S velocity profile."""

from __future__ import annotations

import numpy as np

from robot_logging import get_logger
from .double_s import plan_double_s, sample_double_s
from .limits import DoubleSLimits
from .models import JointTrajectory, TrajectoryError

_logger = get_logger("trajectory.joint")

# Derived from the joint speed an operator configures, in the same way the
# Cartesian planner derives its ramps: reach full speed in this many seconds,
# and full acceleration in the jerk time below.
DEFAULT_ACCELERATION_TIME_S = 0.30
DEFAULT_JERK_TIME_S = 0.15


def joint_limits_from_speed(
    joint_count: int,
    max_joint_speed_rad_s: float,
    *,
    acceleration_time_s: float = DEFAULT_ACCELERATION_TIME_S,
    jerk_time_s: float = DEFAULT_JERK_TIME_S,
) -> DoubleSLimits:
    """Build per-joint double-S limits from a single configured joint speed."""
    speed = float(max_joint_speed_rad_s)
    if not np.isfinite(speed) or speed <= 0.0:
        raise ValueError("最大关节速度必须是有限正数")
    ramp = float(acceleration_time_s)
    jerk_ramp = float(jerk_time_s)
    if not np.isfinite(ramp) or ramp <= 0.0:
        raise ValueError("关节加速时间必须是有限正数")
    if not np.isfinite(jerk_ramp) or jerk_ramp <= 0.0:
        raise ValueError("关节加加速时间必须是有限正数")
    acceleration = speed / ramp
    return DoubleSLimits.build(
        joint_count, speed, acceleration, acceleration / jerk_ramp
    )


def plan_joint_trajectory(
    start: np.ndarray,
    end: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    duration_s: float,
    frequency_hz: float,
    *,
    limits: DoubleSLimits | None = None,
) -> JointTrajectory:
    """Plan a MOVJ move with continuous velocity and acceleration.

    ``duration_s`` acts as a *minimum* cycle time.  When the configured
    ``limits`` cannot cover the distance that quickly the move is stretched
    rather than silently exceeding a joint speed limit, so the returned
    trajectory always satisfies both the requested pacing and the limits.

    Passing ``limits=None`` keeps the historical behaviour of honouring
    ``duration_s`` exactly, with limits derived from that duration.
    """
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
    if np.any(lower > upper):
        raise TrajectoryError("关节下限不得大于上限")
    if np.any(end < lower) or np.any(end > upper):
        raise TrajectoryError("MOVJ target exceeds a joint limit")
    if np.any(start < lower - 1e-9) or np.any(start > upper + 1e-9):
        raise TrajectoryError("MOVJ 起点已超出关节限位")

    joint_count = int(start.size)
    if limits is None:
        # No explicit limits: size the profile so the move takes exactly the
        # requested duration.  A double-S needs its peak velocity above the
        # mean, hence the 2x headroom, and the minimum duration below pins the
        # result to duration_s.
        travel = float(np.max(np.abs(end - start)))
        reference = max(travel, 1e-6) / duration_s
        limits = joint_limits_from_speed(joint_count, reference * 2.0)

    profile = plan_double_s(
        start, end, limits, minimum_duration_s=float(duration_s)
    )
    samples = sample_double_s(profile, float(frequency_hz))

    arms: list[np.ndarray] = []
    velocities: list[np.ndarray] = []
    accelerations: list[np.ndarray] = []
    for sample in samples:
        # Clamping is a safety net, not the primary guard: the target and start
        # are both verified against the limits above, and a double-S never
        # overshoots a monotonic move.  Clamp anyway so a planner regression
        # can never emit an out-of-limit joint command.
        arms.append(np.clip(sample.position, lower, upper))
        velocities.append(sample.velocity)
        accelerations.append(sample.acceleration)
    # Land exactly on the commanded target rather than within sampling epsilon.
    arms[-1] = np.clip(end, lower, upper)

    trajectory = JointTrajectory(
        times=np.asarray([sample.time_s for sample in samples], dtype=float),
        arms=arms,
        velocities=velocities,
        accelerations=accelerations,
        profile_name=type(profile).backend_name,
    )
    limits.check_within(
        trajectory.peak_velocity(),
        trajectory.peak_acceleration(),
        label="MOVJ 关节轨迹",
    )
    _logger.debug(
        "MOVJ doubleS: %d 点, %.3f s, 峰值速度 %.3f rad/s",
        len(arms),
        trajectory.duration_s,
        float(np.max(trajectory.peak_velocity())),
    )
    return trajectory
