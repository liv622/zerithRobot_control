"""Deterministic pacing for fixed-rate motion loops.

A control loop that sleeps for ``period - elapsed`` accumulates the scheduler's
error on every iteration, so a nominal 200 Hz loop slowly drifts away from real
time.  :class:`PacedLoop` instead derives every deadline from one fixed origin,
which keeps sample *n* at ``origin + n * period`` no matter how long any single
iteration took.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

from robot_logging import get_logger

_logger = get_logger("realtime.clock")

MIN_FREQUENCY_HZ = 1.0
MAX_FREQUENCY_HZ = 2000.0
# Leave only a short final interval to a monotonic-clock spin.  This avoids the
# millisecond-scale oversleep that appears as a velocity jump when positions are
# differentiated by their real delivery timestamps.
PRECISE_WAIT_WINDOW_S = 0.0002


@dataclass
class LoopStatistics:
    """Timing behaviour of one motion loop, for diagnostics and audit."""

    samples: int = 0
    overruns: int = 0
    max_jitter_s: float = 0.0
    total_jitter_s: float = 0.0
    max_lateness_s: float = 0.0

    @property
    def mean_jitter_s(self) -> float:
        return self.total_jitter_s / self.samples if self.samples else 0.0

    def as_json(self) -> dict[str, float | int]:
        return {
            "samples": self.samples,
            "overruns": self.overruns,
            "max_jitter_ms": self.max_jitter_s * 1000.0,
            "mean_jitter_ms": self.mean_jitter_s * 1000.0,
            "max_lateness_ms": self.max_lateness_s * 1000.0,
        }


def validate_frequency(frequency_hz: float) -> float:
    """Clamp-check a loop frequency, rejecting values a robot cannot track."""
    value = float(frequency_hz)
    if not MIN_FREQUENCY_HZ <= value <= MAX_FREQUENCY_HZ:
        raise ValueError(
            f"插补频率必须在 {MIN_FREQUENCY_HZ:g} 到 {MAX_FREQUENCY_HZ:g} Hz 之间"
        )
    return value


class PacedLoop:
    """Wait until each successive deadline on a fixed, non-drifting grid.

    ``cancel`` is waited on rather than slept through, so a stop request takes
    effect within one period instead of at the end of the trajectory.
    """

    def __init__(
        self,
        frequency_hz: float,
        cancel: threading.Event | None = None,
        *,
        name: str = "motion",
    ) -> None:
        self.period_s = 1.0 / validate_frequency(frequency_hz)
        self.cancel = cancel or threading.Event()
        self.name = name
        self.statistics = LoopStatistics()
        self.origin_s = time.monotonic()

    def reset(self, origin_s: float | None = None) -> None:
        """Re-anchor the deadline grid, e.g. at the start of a new trajectory."""
        self.origin_s = time.monotonic() if origin_s is None else float(origin_s)

    def deadline_for(self, timestamp_s: float) -> float:
        return self.origin_s + float(timestamp_s)

    def wait_until(self, timestamp_s: float) -> bool:
        """Sleep until ``timestamp_s`` after the origin.

        Returns ``True`` when the loop was cancelled and the caller should stop.
        A deadline already in the past is reported as an overrun and returns
        immediately, so a slow loop degrades into running late rather than
        drifting silently.
        """
        deadline = self.deadline_for(timestamp_s)
        remaining = deadline - time.monotonic()
        self.statistics.samples += 1
        if remaining <= 0.0:
            lateness = -remaining
            self.statistics.overruns += 1
            self.statistics.max_lateness_s = max(
                self.statistics.max_lateness_s, lateness
            )
            if lateness > self.period_s:
                _logger.warning(
                    "%s 循环滞后 %.1f ms（超过一个控制周期 %.1f ms）",
                    self.name,
                    lateness * 1000.0,
                    self.period_s * 1000.0,
                )
            return self.cancel.is_set()
        coarse_wait = max(0.0, remaining - PRECISE_WAIT_WINDOW_S)
        if coarse_wait > 0.0 and self.cancel.wait(coarse_wait):
            return True
        while time.monotonic() < deadline:
            if self.cancel.is_set():
                return True
        jitter = abs(time.monotonic() - deadline)
        self.statistics.max_jitter_s = max(self.statistics.max_jitter_s, jitter)
        self.statistics.total_jitter_s += jitter
        return False
