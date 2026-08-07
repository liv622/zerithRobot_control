"""Double-S (jerk-limited, S-curve) time profiles.

This module owns the framework's default velocity profile.  Two backends
implement the same :class:`DoubleSProfile` interface:

- ``ruckig`` — the primary backend.  A well-tested jerk-limited OTG library
  with time synchronisation across degrees of freedom.
- analytic — a pure NumPy/SciPy seven-phase fallback used when Ruckig is not
  installed, so the framework never loses jerk limiting because of a missing
  optional dependency.

Both produce position, velocity and acceleration that are continuous, with
acceleration starting and ending at zero.  That continuity is the reason to use
double-S rather than a trapezoidal profile: a discontinuous acceleration
command shows up as a torque step and mechanical shock on real hardware.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np

from robot_logging import get_logger
from .limits import DoubleSLimits
from .models import TrajectoryError

_logger = get_logger("trajectory.double_s")


@dataclass(frozen=True)
class ProfileSample:
    """One synchronised sample of a multi-DOF double-S profile."""

    time_s: float
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


class DoubleSProfile(Protocol):
    duration_s: float

    def at(self, time_s: float) -> ProfileSample: ...


def _validate_states(
    start: np.ndarray,
    end: np.ndarray,
    limits: DoubleSLimits,
) -> tuple[np.ndarray, np.ndarray]:
    start_array = np.atleast_1d(np.asarray(start, dtype=float))
    end_array = np.atleast_1d(np.asarray(end, dtype=float))
    if start_array.shape != end_array.shape:
        raise ValueError("起点与终点维度不一致")
    if start_array.shape != (limits.size,):
        raise ValueError(f"起点与终点必须包含 {limits.size} 个数值")
    if not np.all(np.isfinite(start_array)) or not np.all(np.isfinite(end_array)):
        raise ValueError("轨迹端点包含非有限数值")
    return start_array, end_array


class _AnalyticDoubleS:
    """Seven-phase double-S profile computed per DOF and time-synchronised.

    Each DOF is first timed independently under its own velocity, acceleration
    and jerk ceilings.  The slowest DOF sets the trajectory duration, then every
    DOF is re-planned to exactly that duration by scaling its own limits down.
    The result reaches all targets simultaneously, which is what keeps a
    multi-axis MOVJ moving along a straight line in joint space.
    """

    backend_name = "analytic double-S"

    def __init__(
        self,
        start: np.ndarray,
        end: np.ndarray,
        limits: DoubleSLimits,
        *,
        minimum_duration_s: float = 0.0,
    ) -> None:
        self.start, self.end = _validate_states(start, end, limits)
        self.limits = limits
        self.displacement = self.end - self.start
        distances = np.abs(self.displacement)
        # Time every axis independently first; the slowest one owns the move.
        fastest_phases = [
            self._plan_phases(
                float(distances[index]),
                float(limits.max_velocity[index]),
                float(limits.max_acceleration[index]),
                float(limits.max_jerk[index]),
            )
            for index in range(limits.size)
        ]
        self.duration_s = max(
            [self._phase_duration(phase) for phase in fastest_phases]
            + [float(max(0.0, minimum_duration_s))]
        )
        if self.duration_s <= 0.0:
            # Zero-length move: keep a finite duration so sampling still yields
            # a start and an end sample instead of dividing by zero.
            self.duration_s = 1e-3
        # Then re-time every axis onto that duration so they arrive together.
        self._phases = [
            self._plan_with_duration(
                float(distances[index]),
                float(limits.max_velocity[index]),
                float(limits.max_acceleration[index]),
                float(limits.max_jerk[index]),
                self.duration_s,
            )
            for index in range(limits.size)
        ]
        self._signs = np.sign(self.displacement)

    @staticmethod
    def _plan_phases(
        distance: float,
        max_velocity: float,
        max_acceleration: float,
        max_jerk: float,
    ) -> dict[str, float]:
        """Solve the symmetric seven-phase profile for one degree of freedom.

        The unknown is the cruise velocity actually reachable within
        ``distance``.  Three cases are solved in closed form:

        1. the axis reaches ``max_velocity`` and cruises,
        2. it reaches ``max_acceleration`` but not ``max_velocity``,
        3. it is so short that acceleration is still ramping at the midpoint.

        Returning the phase durations rather than a duration lets ``_evaluate``
        stay a pure lookup, so per-sample cost is constant.
        """
        if distance <= 0.0:
            return {
                "distance": 0.0,
                "jerk_time_s": 0.0,
                "ramp_time_s": 0.0,
                "cruise_time_s": 0.0,
                "peak_acceleration": 0.0,
                "peak_velocity": 0.0,
                "jerk": 0.0,
            }

        def ramp(velocity: float) -> tuple[float, float, float]:
            """Return (jerk_time, ramp_time, peak_acceleration) for a velocity."""
            if velocity * max_jerk >= max_acceleration**2:
                # Trapezoidal acceleration: max_acceleration is reached.
                jerk_time = max_acceleration / max_jerk
                return (
                    jerk_time,
                    velocity / max_acceleration + jerk_time,
                    max_acceleration,
                )
            # Triangular acceleration: jerk reverses before max_acceleration.
            jerk_time = float(np.sqrt(velocity / max_jerk))
            return jerk_time, 2.0 * jerk_time, max_jerk * jerk_time

        jerk_time, ramp_time, peak_acceleration = ramp(max_velocity)
        if max_velocity * ramp_time <= distance:
            # Case 1: room to cruise at max_velocity.
            peak_velocity = max_velocity
            cruise_time = (distance - peak_velocity * ramp_time) / peak_velocity
        else:
            cruise_time = 0.0
            # Case 2: distance covered by a full accelerate/decelerate pair that
            # still reaches max_acceleration.  Solve
            # distance = v*(v/a_max + a_max/j_max) for v.
            jerk_time = max_acceleration / max_jerk
            discriminant = jerk_time**2 + 4.0 * distance / max_acceleration
            peak_velocity = (
                max_acceleration * (-jerk_time + float(np.sqrt(discriminant))) / 2.0
            )
            if peak_velocity * max_jerk < max_acceleration**2:
                # Case 3: acceleration never reaches max_acceleration, so the
                # profile is four jerk phases and distance = 2*j*t_j^3.
                jerk_time = float(np.cbrt(distance / (2.0 * max_jerk)))
                peak_velocity = max_jerk * jerk_time**2
            jerk_time, ramp_time, peak_acceleration = ramp(peak_velocity)
        return {
            "distance": distance,
            "jerk_time_s": jerk_time,
            "ramp_time_s": ramp_time,
            "cruise_time_s": max(0.0, cruise_time),
            "peak_acceleration": peak_acceleration,
            "peak_velocity": peak_velocity,
            "jerk": max_jerk,
        }

    @classmethod
    def _phase_duration(cls, phase: dict[str, float]) -> float:
        return 2.0 * phase["ramp_time_s"] + phase["cruise_time_s"]

    @classmethod
    def _plan_with_duration(
        cls,
        distance: float,
        max_velocity: float,
        max_acceleration: float,
        max_jerk: float,
        duration_s: float,
    ) -> dict[str, float]:
        """Plan a profile that takes exactly ``duration_s`` where possible.

        Time synchronisation across degrees of freedom, and honouring an
        operator cycle time, both require stretching a move.  Velocity is
        bisected downward because profile duration is monotonically decreasing
        in cruise velocity, which makes the search well behaved for every case
        above.  This runs once per motion, never per sample.
        """
        if distance <= 0.0:
            phase = cls._plan_phases(0.0, max_velocity, max_acceleration, max_jerk)
            phase["cruise_time_s"] = max(0.0, duration_s)
            return phase
        fastest = cls._plan_phases(
            distance, max_velocity, max_acceleration, max_jerk
        )
        if cls._phase_duration(fastest) >= duration_s - 1e-12:
            # Already at or slower than the requested duration at full speed.
            return fastest
        low, high = 1e-12, max_velocity
        best = fastest
        for _ in range(90):
            middle = 0.5 * (low + high)
            candidate = cls._plan_phases(
                distance, middle, max_acceleration, max_jerk
            )
            if cls._phase_duration(candidate) > duration_s:
                low = middle
            else:
                high = middle
                best = candidate
        return best

    @staticmethod
    def _evaluate(phase: dict[str, float], time_s: float) -> tuple[float, float, float]:
        """Return (position, velocity, acceleration) for one DOF at ``time_s``.

        The deceleration half is evaluated as the mirror of the acceleration
        half about the move midpoint, which guarantees the profile ends exactly
        on ``distance`` with zero velocity and zero acceleration regardless of
        floating point accumulation.
        """
        distance = phase["distance"]
        jerk_time = phase["jerk_time_s"]
        ramp_time = phase["ramp_time_s"]
        cruise_time = phase["cruise_time_s"]
        jerk = phase["jerk"]
        peak_acceleration = phase["peak_acceleration"]
        peak_velocity = phase["peak_velocity"]
        total = 2.0 * ramp_time + cruise_time
        if distance <= 0.0 or total <= 0.0:
            return 0.0, 0.0, 0.0
        constant_time = max(0.0, ramp_time - 2.0 * jerk_time)

        def accelerating(elapsed: float) -> tuple[float, float, float]:
            """Position/velocity/acceleration inside the acceleration half."""
            if elapsed <= 0.0:
                return 0.0, 0.0, 0.0
            # Phase 1: jerk ramps acceleration up.
            if elapsed < jerk_time:
                return (
                    jerk * elapsed**3 / 6.0,
                    jerk * elapsed**2 / 2.0,
                    jerk * elapsed,
                )
            position = jerk * jerk_time**3 / 6.0
            velocity = jerk * jerk_time**2 / 2.0
            # Phase 2: constant acceleration.
            if elapsed < jerk_time + constant_time:
                step = elapsed - jerk_time
                return (
                    position + velocity * step + peak_acceleration * step**2 / 2.0,
                    velocity + peak_acceleration * step,
                    peak_acceleration,
                )
            position += (
                velocity * constant_time
                + peak_acceleration * constant_time**2 / 2.0
            )
            velocity += peak_acceleration * constant_time
            # Phase 3: jerk ramps acceleration back to zero at peak velocity.
            step = min(elapsed, ramp_time) - jerk_time - constant_time
            return (
                position
                + velocity * step
                + peak_acceleration * step**2 / 2.0
                - jerk * step**3 / 6.0,
                velocity + peak_acceleration * step - jerk * step**2 / 2.0,
                peak_acceleration - jerk * step,
            )

        elapsed = float(np.clip(time_s, 0.0, total))
        if elapsed <= ramp_time:
            return accelerating(elapsed)
        if elapsed < ramp_time + cruise_time:
            # Phase 4: cruise.
            ramp_distance, _, _ = accelerating(ramp_time)
            return (
                ramp_distance + peak_velocity * (elapsed - ramp_time),
                peak_velocity,
                0.0,
            )
        # Phases 5-7: mirror of the acceleration half.
        remaining = total - elapsed
        mirrored_position, velocity, acceleration = accelerating(remaining)
        return distance - mirrored_position, velocity, -acceleration

    def at(self, time_s: float) -> ProfileSample:
        position = np.empty(self.limits.size)
        velocity = np.empty(self.limits.size)
        acceleration = np.empty(self.limits.size)
        for index, phase in enumerate(self._phases):
            p, v, a = self._evaluate(phase, time_s)
            sign = self._signs[index]
            position[index] = self.start[index] + sign * p
            velocity[index] = sign * v
            acceleration[index] = sign * a
        return ProfileSample(
            time_s=float(np.clip(time_s, 0.0, self.duration_s)),
            position=position,
            velocity=velocity,
            acceleration=acceleration,
        )


class _RuckigDoubleS:
    """Double-S profile delegated to the Ruckig online trajectory generator."""

    backend_name = "ruckig double-S"

    def __init__(
        self,
        start: np.ndarray,
        end: np.ndarray,
        limits: DoubleSLimits,
        *,
        minimum_duration_s: float = 0.0,
    ) -> None:
        from ruckig import InputParameter, Result, Ruckig, Trajectory

        self.start, self.end = _validate_states(start, end, limits)
        self.limits = limits
        size = limits.size
        parameters = InputParameter(size)
        parameters.current_position = [float(value) for value in self.start]
        parameters.current_velocity = [0.0] * size
        parameters.current_acceleration = [0.0] * size
        parameters.target_position = [float(value) for value in self.end]
        parameters.target_velocity = [0.0] * size
        parameters.target_acceleration = [0.0] * size
        parameters.max_velocity = [float(value) for value in limits.max_velocity]
        parameters.max_acceleration = [
            float(value) for value in limits.max_acceleration
        ]
        parameters.max_jerk = [float(value) for value in limits.max_jerk]
        if minimum_duration_s > 0.0:
            parameters.minimum_duration = float(minimum_duration_s)
        trajectory = Trajectory(size)
        result = Ruckig(size).calculate(parameters, trajectory)
        if result not in {Result.Working, Result.Finished}:
            raise TrajectoryError(f"doubleS 速度规划失败：Ruckig {result}")
        self._trajectory = trajectory
        self.duration_s = max(float(trajectory.duration), 1e-9)

    def at(self, time_s: float) -> ProfileSample:
        clamped = float(np.clip(time_s, 0.0, self.duration_s))
        position, velocity, acceleration = self._trajectory.at_time(clamped)
        return ProfileSample(
            time_s=clamped,
            position=np.asarray(position, dtype=float),
            velocity=np.asarray(velocity, dtype=float),
            acceleration=np.asarray(acceleration, dtype=float),
        )


def double_s_backend_name() -> str:
    """Report which double-S implementation this installation will use."""
    try:
        import ruckig  # noqa: F401
    except ModuleNotFoundError:
        return _AnalyticDoubleS.backend_name
    return _RuckigDoubleS.backend_name


def plan_double_s(
    start: np.ndarray,
    end: np.ndarray,
    limits: DoubleSLimits,
    *,
    minimum_duration_s: float = 0.0,
    prefer_library: bool = True,
) -> DoubleSProfile:
    """Build a jerk-limited double-S profile from ``start`` to ``end``.

    Ruckig is used when installed; otherwise the analytic backend runs.  Both
    honour ``minimum_duration_s``, which is how an operator speed override or a
    per-point cycle time stretches a move without changing its shape.
    """
    if prefer_library:
        try:
            return _RuckigDoubleS(
                start, end, limits, minimum_duration_s=minimum_duration_s
            )
        except ModuleNotFoundError:
            _logger.debug("未安装 ruckig，改用解析 doubleS 实现")
        except TrajectoryError:
            raise
    return _AnalyticDoubleS(
        start, end, limits, minimum_duration_s=minimum_duration_s
    )


def sample_double_s(
    profile: DoubleSProfile,
    frequency_hz: float,
) -> list[ProfileSample]:
    """Sample ``profile`` on the controller's exact ``1 / frequency`` grid.

    A previous implementation used ``linspace(0, duration, ceil(duration*f))``.
    That includes the endpoint, but subtly changes every control interval
    (e.g. 4.9878 ms at a requested 200 Hz).  Hardware still estimated velocity
    with a 5 ms period, which made position differences and reported velocity
    disagree.  Here every timestamp is an exact controller tick.  When the
    physical profile ends between ticks, the final tick holds the target with
    zero velocity and acceleration.
    """
    if not np.isfinite(frequency_hz) or frequency_hz <= 0.0:
        raise ValueError("采样频率必须为正")
    period_s = 1.0 / float(frequency_hz)
    intervals = max(1, int(np.ceil(profile.duration_s / period_s)))
    times = np.arange(intervals + 1, dtype=float) * period_s
    result: list[ProfileSample] = []
    for scheduled_time in times:
        state = profile.at(min(float(scheduled_time), profile.duration_s))
        result.append(
            ProfileSample(
                time_s=float(scheduled_time),
                position=state.position,
                velocity=state.velocity,
                acceleration=state.acceleration,
            )
        )
    return result
