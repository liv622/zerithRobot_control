"""MOVL Cartesian interpolation with a double-S path profile and IK preflight."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from robot_framework.solver import IKSolution, IKSolver
from robot_logging import get_logger
from .double_s import DoubleSProfile, plan_double_s, sample_double_s
from .limits import CartesianLimits, DoubleSLimits
from .models import Trajectory, TrajectoryError

_logger = get_logger("trajectory.cartesian")

# IK preflight acceptance thresholds.  A MOVL that cannot hold the commanded
# pose this closely is refused rather than executed with silent drift.
MAX_POSITION_ERROR_M = 0.002
MAX_ORIENTATION_ERROR_RAD = np.deg2rad(1.0)
# Largest per-sample joint step tolerated before a move is treated as passing
# through a singularity or flipping arm configuration.
MAX_JOINT_STEP_RAD = np.deg2rad(15.0)


def pose_from_position_and_rotation(
    position: np.ndarray, rotation: Rotation
) -> np.ndarray:
    """Compose a 4x4 homogeneous transform."""
    pose = np.eye(4)
    pose[:3, :3] = rotation.as_matrix()
    pose[:3, 3] = np.asarray(position, dtype=float)
    return pose


def segment_distance_and_angle(start: np.ndarray, end: np.ndarray) -> tuple[float, float]:
    """Return the linear (m) and angular (rad) extent of one MOVL segment."""
    distance = float(np.linalg.norm(end[:3, 3] - start[:3, 3]))
    angle = float(
        np.linalg.norm(
            Rotation.from_matrix(end[:3, :3] @ start[:3, :3].T).as_rotvec()
        )
    )
    return distance, angle


def interpolate_cartesian_segment(
    start: np.ndarray,
    end: np.ndarray,
    count: int,
    *,
    progress: np.ndarray | None = None,
) -> list[np.ndarray]:
    """Interpolate poses between two frames.

    ``progress`` supplies the normalised path parameter for each sample, which
    is how a double-S profile shapes the motion.  When omitted the samples are
    spaced by a quintic smooth-step, preserving the previous behaviour for
    callers that only need geometry.
    """
    if progress is None:
        value = np.linspace(0.0, 1.0, max(2, count))
        progress = 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5
    progress = np.clip(np.asarray(progress, dtype=float), 0.0, 1.0)
    positions = start[:3, 3] + progress[:, None] * (end[:3, 3] - start[:3, 3])
    rotations = Slerp(
        [0.0, 1.0],
        Rotation.from_matrix(np.stack([start[:3, :3], end[:3, :3]])),
    )(progress)
    return [
        pose_from_position_and_rotation(position, rotation)
        for position, rotation in zip(positions, rotations)
    ]


def plan_cartesian_path_profile(
    waypoints: list[np.ndarray],
    limits: CartesianLimits,
    *,
    minimum_duration_s: float = 0.0,
) -> tuple[DoubleSProfile, np.ndarray, np.ndarray]:
    """Build a double-S profile over the arc length of a multi-segment path.

    Returns the profile plus the cumulative and per-segment normalised lengths.
    Planning one profile across the whole path — instead of one per segment —
    is what avoids a full stop at every intermediate waypoint while still
    bounding velocity, acceleration and jerk.
    """
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    distances = np.empty(len(waypoints) - 1)
    angles = np.empty(len(waypoints) - 1)
    for index, (first, second) in enumerate(zip(waypoints[:-1], waypoints[1:])):
        distances[index], angles[index] = segment_distance_and_angle(first, second)
    total_distance = float(distances.sum())
    total_angle = float(angles.sum())
    # Weight segments by whichever motion dominates so a pure reorientation is
    # still allocated sampling time.
    weights = distances + angles * 0.1
    if float(weights.sum()) <= 1e-12:
        weights = np.ones_like(weights)
    fractions = weights / float(weights.sum())
    path_limits = limits.path_limits(total_distance, total_angle)
    profile = plan_double_s(
        np.zeros(1),
        np.ones(1),
        path_limits,
        minimum_duration_s=minimum_duration_s,
    )
    return profile, np.concatenate([[0.0], np.cumsum(fractions)]), fractions


def plan_cartesian_trajectory(
    solver: IKSolver,
    waypoints: list[np.ndarray],
    initial_arm: np.ndarray,
    aux: dict[str, float],
    duration_s: float,
    frequency_hz: float,
    *,
    multi_start: bool = True,
    limits: CartesianLimits | None = None,
) -> Trajectory:
    """Plan and IK-preflight a MOVL trajectory with a double-S path profile.

    Every sample is solved before the first one is returned.  A trajectory that
    cannot be solved end to end is refused as a whole, so the robot never starts
    a MOVL that would fail partway through.
    """
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    if duration_s <= 0 or frequency_hz <= 0:
        raise ValueError("duration and frequency must be positive")
    if limits is None:
        # Derive limits that make the requested duration achievable, keeping the
        # historical contract that duration_s governs when no limits are given.
        total_distance = 0.0
        total_angle = 0.0
        for first, second in zip(waypoints[:-1], waypoints[1:]):
            distance, angle = segment_distance_and_angle(first, second)
            total_distance += distance
            total_angle += angle
        limits = CartesianLimits.from_pendant_units(
            max_linear_speed_mm_s=max(
                1.0, total_distance / duration_s * 2.0 * 1000.0
            ),
            max_angular_speed_deg_s=max(
                1.0, np.rad2deg(total_angle / duration_s) * 2.0
            ),
        )

    profile, boundaries, _ = plan_cartesian_path_profile(
        waypoints, limits, minimum_duration_s=float(duration_s)
    )
    samples = sample_double_s(profile, float(frequency_hz))
    times = np.asarray([sample.time_s for sample in samples], dtype=float)
    path_velocities = np.asarray(
        [float(sample.velocity[0]) for sample in samples], dtype=float
    )

    poses: list[np.ndarray] = []
    for sample in samples:
        progress = float(np.clip(sample.position[0], 0.0, 1.0))
        # Locate the segment this path position falls in, then interpolate
        # within it using the segment-local parameter.
        index = int(np.searchsorted(boundaries, progress, side="right") - 1)
        index = int(np.clip(index, 0, len(waypoints) - 2))
        span = boundaries[index + 1] - boundaries[index]
        local = 0.0 if span <= 1e-12 else (progress - boundaries[index]) / span
        poses.append(
            interpolate_cartesian_segment(
                waypoints[index],
                waypoints[index + 1],
                2,
                progress=np.asarray([np.clip(local, 0.0, 1.0)]),
            )[0]
        )
    # Guarantee the commanded endpoints are exact.
    poses[0] = np.asarray(waypoints[0], dtype=float).copy()
    poses[-1] = np.asarray(waypoints[-1], dtype=float).copy()

    solutions: list[IKSolution] = []
    arm = np.asarray(initial_arm, dtype=float).copy()
    for index, pose in enumerate(poses):
        solution = solver.solve(
            pose,
            arm,
            aux,
            lock_orientation=True,
            multi_start=multi_start,
            recovery_seeds=6 if multi_start else 0,
        )
        if (
            solution.position_error_m > MAX_POSITION_ERROR_M
            or solution.orientation_error_rad > MAX_ORIENTATION_ERROR_RAD
        ):
            raise TrajectoryError(
                f"sample {index} failed: {solution.position_error_m * 1000:.2f} mm, "
                f"{np.rad2deg(solution.orientation_error_rad):.2f} deg"
            )
        if solutions and np.max(np.abs(solution.arm - arm)) > MAX_JOINT_STEP_RAD:
            raise TrajectoryError(
                f"sample {index} contains a joint jump above "
                f"{np.rad2deg(MAX_JOINT_STEP_RAD):.0f} deg"
            )
        solutions.append(solution)
        arm = solution.arm
    _logger.debug(
        "MOVL doubleS: %d 点, %.3f s, %d 段",
        len(poses),
        float(times[-1]) if len(times) else 0.0,
        len(waypoints) - 1,
    )
    return Trajectory(
        times=times,
        poses=poses,
        solutions=solutions,
        path_velocities=path_velocities,
        profile_name=type(profile).backend_name,
    )
