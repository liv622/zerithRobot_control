"""Final safety gate applied to every joint command before it is delivered.

Planners already respect the configured limits, so this is a second,
independent check.  That redundancy is the point: a planner regression, a bad
IK solution, or a mis-scaled operator setting must surface as a refused motion
rather than as a fast joint command on real hardware.

The guard is stateful because velocity and acceleration only exist between
successive samples.  It is deliberately cheap — a handful of vectorised
comparisons — because it runs once per control period.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

import numpy as np

from robot_logging import get_logger

_logger = get_logger("realtime.safety")


class SafetyViolation(ValueError):
    """Raised when a joint command fails a limit check and must not be sent."""


@dataclass(frozen=True)
class JointSafetyLimits:
    """Absolute ceilings enforced on commanded joint motion."""

    lower: np.ndarray
    upper: np.ndarray
    max_velocity: np.ndarray
    max_acceleration: np.ndarray
    # Tolerance multiplier applied to rate limits.  Sampling a continuous
    # profile at a fixed rate produces small numeric overshoot at the peak, so
    # a hard 1.0 comparison would reject valid trajectories.
    tolerance: float = 1.20

    @classmethod
    def build(
        cls,
        lower: np.ndarray,
        upper: np.ndarray,
        max_velocity: np.ndarray | float,
        max_acceleration: np.ndarray | float,
        *,
        tolerance: float = 1.20,
    ) -> "JointSafetyLimits":
        lower_array = np.asarray(lower, dtype=float)
        upper_array = np.asarray(upper, dtype=float)
        if lower_array.shape != upper_array.shape:
            raise ValueError("关节上下限维度不一致")
        if np.any(lower_array > upper_array):
            raise ValueError("关节下限不得大于上限")
        size = lower_array.size
        velocity = np.broadcast_to(
            np.asarray(max_velocity, dtype=float), (size,)
        ).astype(float)
        acceleration = np.broadcast_to(
            np.asarray(max_acceleration, dtype=float), (size,)
        ).astype(float)
        if np.any(velocity <= 0.0) or np.any(acceleration <= 0.0):
            raise ValueError("速度与加速度上限必须为正")
        return cls(
            lower=lower_array,
            upper=upper_array,
            max_velocity=velocity,
            max_acceleration=acceleration,
            tolerance=float(tolerance),
        )


class JointCommandGuard:
    """Validate a stream of joint commands against position and rate limits.

    Thread safe: the streamer's sender thread and the motion thread may both
    touch the guard when a move is interrupted.
    """

    def __init__(self, limits: JointSafetyLimits) -> None:
        self.limits = limits
        self._lock = threading.Lock()
        self._previous_position: np.ndarray | None = None
        self._previous_velocity: np.ndarray | None = None
        self._previous_time_s: float | None = None
        self.rejections = 0

    def reset(self) -> None:
        """Forget history, e.g. when a new trajectory starts after a pause.

        Required so the position step across an idle gap is not mistaken for a
        single-period velocity spike.
        """
        with self._lock:
            self._previous_position = None
            self._previous_velocity = None
            self._previous_time_s = None

    def check(self, joints: np.ndarray, timestamp_s: float) -> np.ndarray:
        """Validate one command, returning the array that is safe to send.

        Raises :class:`SafetyViolation` on any breach.  Callers treat that as
        "stop the move", which is why nothing is clamped silently here: a
        command outside the limits means an upstream bug, and continuing with a
        quietly modified target would hide it.
        """
        values = np.asarray(joints, dtype=float)
        with self._lock:
            limits = self.limits
            if values.shape != limits.lower.shape:
                self.rejections += 1
                raise SafetyViolation(
                    f"关节指令必须包含 {limits.lower.size} 个数值，"
                    f"实际 {values.shape}"
                )
            if not np.all(np.isfinite(values)):
                self.rejections += 1
                raise SafetyViolation("关节指令包含非有限数值")
            below = values < limits.lower - 1e-9
            above = values > limits.upper + 1e-9
            if np.any(below | above):
                index = int(np.argmax(below | above))
                self.rejections += 1
                raise SafetyViolation(
                    f"关节 {index + 1} 指令 {np.rad2deg(values[index]):.2f}° "
                    f"超出限位 "
                    f"[{np.rad2deg(limits.lower[index]):.2f}°, "
                    f"{np.rad2deg(limits.upper[index]):.2f}°]"
                )

            time_s = float(timestamp_s)
            previous_position = self._previous_position
            previous_time = self._previous_time_s
            velocity = np.zeros_like(values)
            if previous_position is not None and previous_time is not None:
                interval = time_s - previous_time
                if interval <= 0.0:
                    # Out-of-order or duplicate timestamp: keep the position
                    # check that already passed, skip the undefined rate check.
                    self._previous_position = values.copy()
                    return values
                velocity = (values - previous_position) / interval
                ceiling = limits.max_velocity * limits.tolerance
                if np.any(np.abs(velocity) > ceiling):
                    index = int(np.argmax(np.abs(velocity) - ceiling))
                    self.rejections += 1
                    raise SafetyViolation(
                        f"关节 {index + 1} 指令速度 "
                        f"{np.rad2deg(abs(velocity[index])):.1f}°/s 超过上限 "
                        f"{np.rad2deg(limits.max_velocity[index]):.1f}°/s"
                    )
                previous_velocity = self._previous_velocity
                if previous_velocity is not None:
                    acceleration = (velocity - previous_velocity) / interval
                    accel_ceiling = limits.max_acceleration * limits.tolerance
                    if np.any(np.abs(acceleration) > accel_ceiling):
                        index = int(np.argmax(np.abs(acceleration) - accel_ceiling))
                        self.rejections += 1
                        raise SafetyViolation(
                            f"关节 {index + 1} 指令加速度 "
                            f"{np.rad2deg(abs(acceleration[index])):.0f}°/s² "
                            f"超过上限 "
                            f"{np.rad2deg(limits.max_acceleration[index]):.0f}°/s²"
                        )
                self._previous_velocity = velocity
            self._previous_position = values.copy()
            self._previous_time_s = time_s
            return values

    def state(self) -> dict[str, int]:
        with self._lock:
            return {"rejections": self.rejections}


class GuardedJointSink:
    """Wrap a joint sink so every sample passes the guard before delivery.

    Composing this around the hardware adapter means the check cannot be
    bypassed by a new caller that forgets to validate: the sink itself refuses.
    """

    def __init__(
        self,
        sink,
        guard: JointCommandGuard,
        *,
        clock=None,
    ) -> None:
        self._sink = sink
        self._guard = guard
        if clock is None:
            import time

            clock = time.monotonic
        self._clock = clock

    def send_joint_radians(self, joints: np.ndarray) -> None:
        checked = self._guard.check(joints, self._clock())
        self._sink.send_joint_radians(checked)
