"""MOVL Cartesian interpolation with complete IK preflight."""

from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

from robot_framework.solver import IKSolution, IKSolver
from .models import Trajectory, TrajectoryError
from .profiles import quintic


def interpolate_cartesian_segment(
    start: np.ndarray,
    end: np.ndarray,
    count: int,
) -> list[np.ndarray]:
    u = np.linspace(0.0, 1.0, max(2, count))
    smooth = quintic(u)
    positions = start[:3, 3] + smooth[:, None] * (end[:3, 3] - start[:3, 3])
    rotations = Slerp(
        [0.0, 1.0],
        Rotation.from_matrix(np.stack([start[:3, :3], end[:3, :3]])),
    )(smooth)
    poses: list[np.ndarray] = []
    for position, rotation in zip(positions, rotations):
        pose = np.eye(4)
        pose[:3, :3] = rotation.as_matrix()
        pose[:3, 3] = position
        poses.append(pose)
    return poses


def plan_cartesian_trajectory(
    solver: IKSolver,
    waypoints: list[np.ndarray],
    initial_arm: np.ndarray,
    aux: dict[str, float],
    duration_s: float,
    frequency_hz: float,
    *,
    multi_start: bool = True,
) -> Trajectory:
    if len(waypoints) < 2:
        raise ValueError("at least two waypoints are required")
    if duration_s <= 0 or frequency_hz <= 0:
        raise ValueError("duration and frequency must be positive")

    distances = np.array(
        [
            np.linalg.norm(second[:3, 3] - first[:3, 3]) + 1e-6
            for first, second in zip(waypoints[:-1], waypoints[1:])
        ]
    )
    ratios = distances / distances.sum()
    total_samples = max(len(waypoints), int(np.ceil(duration_s * frequency_hz)) + 1)
    poses: list[np.ndarray] = []
    for index, (start, end, ratio) in enumerate(
        zip(waypoints[:-1], waypoints[1:], ratios)
    ):
        count = max(2, int(round((total_samples - 1) * ratio)) + 1)
        segment = interpolate_cartesian_segment(start, end, count)
        poses.extend(segment if index == 0 else segment[1:])

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
            solution.position_error_m > 0.002
            or solution.orientation_error_rad > np.deg2rad(1.0)
        ):
            raise TrajectoryError(
                f"sample {index} failed: {solution.position_error_m * 1000:.2f} mm, "
                f"{np.rad2deg(solution.orientation_error_rad):.2f} deg"
            )
        if solutions and np.max(np.abs(solution.arm - arm)) > np.deg2rad(15.0):
            raise TrajectoryError(f"sample {index} contains a joint jump above 15 deg")
        solutions.append(solution)
        arm = solution.arm
    times = np.linspace(0.0, duration_s, len(poses))
    return Trajectory(times=times, poses=poses, solutions=solutions)
