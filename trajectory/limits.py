"""Kinematic limit descriptions shared by every trajectory planner."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .models import TrajectoryError


def _as_positive_array(value: np.ndarray | list[float] | float, size: int, label: str) -> np.ndarray:
    array = np.atleast_1d(np.asarray(value, dtype=float))
    if array.size == 1:
        array = np.repeat(array, size)
    if array.shape != (size,):
        raise ValueError(f"{label} 必须是标量或长度为 {size} 的向量")
    if not np.all(np.isfinite(array)) or np.any(array <= 0.0):
        raise ValueError(f"{label} 必须是有限正数")
    return array


@dataclass(frozen=True)
class DoubleSLimits:
    """Per-degree-of-freedom velocity, acceleration and jerk ceilings.

    A double-S (S-curve) profile is defined by all three: bounding jerk is what
    makes acceleration continuous, which is what keeps commanded velocity
    continuous and free of the steps a trapezoidal profile produces.
    """

    max_velocity: np.ndarray
    max_acceleration: np.ndarray
    max_jerk: np.ndarray

    @classmethod
    def build(
        cls,
        size: int,
        max_velocity: np.ndarray | list[float] | float,
        max_acceleration: np.ndarray | list[float] | float,
        max_jerk: np.ndarray | list[float] | float,
    ) -> "DoubleSLimits":
        if size <= 0:
            raise ValueError("自由度数量必须为正")
        return cls(
            max_velocity=_as_positive_array(max_velocity, size, "最大速度"),
            max_acceleration=_as_positive_array(
                max_acceleration, size, "最大加速度"
            ),
            max_jerk=_as_positive_array(max_jerk, size, "最大加加速度"),
        )

    @property
    def size(self) -> int:
        return int(self.max_velocity.size)

    def scaled(self, factor: float) -> "DoubleSLimits":
        """Return the same limits scaled by an operator speed override.

        Velocity scales linearly with the override, acceleration with its
        square and jerk with its cube.  This is the standard scaling that keeps
        the *shape* of the S-curve identical while stretching it in time, so a
        50% override yields the same motion executed twice as slowly instead of
        a differently-shaped profile.
        """
        value = float(factor)
        if not np.isfinite(value) or not 0.0 < value <= 1.0:
            raise ValueError("速度倍率必须位于 (0, 1] 区间")
        return DoubleSLimits(
            max_velocity=self.max_velocity * value,
            max_acceleration=self.max_acceleration * value**2,
            max_jerk=self.max_jerk * value**3,
        )

    def check_within(
        self,
        velocity: np.ndarray,
        acceleration: np.ndarray | None = None,
        *,
        tolerance: float = 1.05,
        label: str = "轨迹",
    ) -> None:
        """Raise :class:`TrajectoryError` when a sampled profile exceeds limits.

        Used as an independent post-check on planner output: a planner bug must
        surface as a refused motion rather than as an over-speed command.
        """
        speed = np.max(np.abs(np.asarray(velocity, dtype=float)) / self.max_velocity)
        if speed > tolerance:
            raise TrajectoryError(
                f"{label}超出速度限制：达到限值的 {speed * 100:.1f}%"
            )
        if acceleration is None:
            return
        accel = np.max(
            np.abs(np.asarray(acceleration, dtype=float)) / self.max_acceleration
        )
        if accel > tolerance:
            raise TrajectoryError(
                f"{label}超出加速度限制：达到限值的 {accel * 100:.1f}%"
            )


@dataclass(frozen=True)
class CartesianLimits:
    """Task-space ceilings used to time a Cartesian double-S path.

    Linear values are metres based; angular values are radians based.  The
    pendant works in mm/s and deg/s, so conversion happens once at the
    application boundary rather than being repeated in the planners.
    """

    max_linear_velocity: float
    max_linear_acceleration: float
    max_linear_jerk: float
    max_angular_velocity: float
    max_angular_acceleration: float
    max_angular_jerk: float

    @classmethod
    def from_pendant_units(
        cls,
        *,
        max_linear_speed_mm_s: float,
        max_angular_speed_deg_s: float,
        linear_acceleration_time_s: float = 0.35,
        jerk_time_s: float = 0.20,
    ) -> "CartesianLimits":
        """Derive full double-S limits from the two speeds an operator sets.

        Operators configure speeds, not jerk.  Acceleration is derived as
        "reach full speed in ``linear_acceleration_time_s``" and jerk as "reach
        full acceleration in ``jerk_time_s``", which is the usual way an
        industrial pendant exposes a single speed dial while still producing a
        jerk-limited profile underneath.
        """
        linear = float(max_linear_speed_mm_s) / 1000.0
        angular = np.deg2rad(float(max_angular_speed_deg_s))
        for value, label in ((linear, "最大线速度"), (angular, "最大角速度")):
            if not np.isfinite(value) or value <= 0.0:
                raise ValueError(f"{label} 必须是有限正数")
        ramp = float(linear_acceleration_time_s)
        jerk_ramp = float(jerk_time_s)
        if not np.isfinite(ramp) or ramp <= 0.0 or not np.isfinite(jerk_ramp) or jerk_ramp <= 0.0:
            raise ValueError("加速时间与加加速时间必须是有限正数")
        linear_acceleration = linear / ramp
        angular_acceleration = angular / ramp
        return cls(
            max_linear_velocity=linear,
            max_linear_acceleration=linear_acceleration,
            max_linear_jerk=linear_acceleration / jerk_ramp,
            max_angular_velocity=angular,
            max_angular_acceleration=angular_acceleration,
            max_angular_jerk=angular_acceleration / jerk_ramp,
        )

    def path_limits(self, distance_m: float, angle_rad: float) -> DoubleSLimits:
        """Collapse linear and angular ceilings onto one path parameter s∈[0,1].

        A Cartesian MOVL is planned as a single scalar double-S along the path
        so that position and orientation stay synchronised.  Whichever of the
        two components is more restrictive for this particular segment governs
        the profile.
        """
        distance = float(abs(distance_m))
        angle = float(abs(angle_rad))
        if not np.isfinite(distance) or not np.isfinite(angle):
            raise ValueError("笛卡尔段长度必须是有限数值")
        candidates: list[tuple[float, float, float]] = []
        if distance > 1e-9:
            candidates.append(
                (
                    self.max_linear_velocity / distance,
                    self.max_linear_acceleration / distance,
                    self.max_linear_jerk / distance,
                )
            )
        if angle > 1e-9:
            candidates.append(
                (
                    self.max_angular_velocity / angle,
                    self.max_angular_acceleration / angle,
                    self.max_angular_jerk / angle,
                )
            )
        if not candidates:
            # Degenerate zero-length segment: keep a well-formed profile so the
            # caller still receives at least a start and an end sample.
            candidates.append((1.0, 4.0, 40.0))
        velocity = min(item[0] for item in candidates)
        acceleration = min(item[1] for item in candidates)
        jerk = min(item[2] for item in candidates)
        return DoubleSLimits.build(1, velocity, acceleration, jerk)
