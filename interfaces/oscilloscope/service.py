"""OscilloscopeService — background sampler + JointMonitor for real-time charts.

The service owns a :class:`~realtime.monitor.JointMonitor` and a dedicated
sampler thread.  The sampler periodically invokes a caller-supplied callback
to read the current joint positions (radians), then hands each sample to the
monitor via its non-blocking ``submit()``.  The monitor's own background
thread computes velocity and acceleration via finite differences.

This design keeps the motion loop completely unaffected: the sampler is the
*only* writer, and it does not share a lock with the motion thread.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from typing import Any

import numpy as np

from realtime.monitor import JointMonitor, JointStreamFrame
from robot_logging import get_logger

_logger = get_logger("interfaces.oscilloscope")


class OscilloscopeService:
    """Background joint-state sampler that feeds a JointMonitor.

    Parameters
    ----------
    joint_count:
        Number of actuated joints.
    capacity:
        Ring-buffer size (samples).  At 50 Hz, 2000 ≈ 40 seconds.
    sample_hz:
        How often to poll ``get_joints``.  The monitor's internal
        finite-difference derivatives use the actual wall-clock delta,
        so this rate only needs to be high enough to capture dynamics.
    """

    def __init__(
        self,
        joint_count: int,
        capacity: int = 2000,
        sample_hz: float = 50.0,
    ) -> None:
        if joint_count < 1:
            raise ValueError("joint_count must be >= 1")
        if sample_hz <= 0:
            raise ValueError("sample_hz must be positive")
        self._joint_count = int(joint_count)
        self._sample_hz = float(sample_hz)
        self._monitor = JointMonitor(
            capacity=int(capacity),
            joint_count=self._joint_count,
        )
        self._get_joints: Callable[[], np.ndarray] | None = None
        self._thread: threading.Thread | None = None
        self._closed = False
        self._last_motion_sample = 0.0
        self._motion_clock_lock = threading.Lock()
        self._motion_logical_time = 0.0
        self._motion_wall_time = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    @property
    def monitor(self) -> JointMonitor:
        """The underlying JointMonitor (for direct access if needed)."""
        return self._monitor

    @property
    def joint_count(self) -> int:
        return self._joint_count

    def start(self, get_joints: Callable[[], np.ndarray]) -> None:
        """Begin sampling.

        ``get_joints()`` must return a ``(joint_count,)`` float64 array of
        current joint positions in **radians**.  It is called from the sampler
        thread at *sample_hz* and must be safe to invoke concurrently with
        the motion loop (typically it just reads a shared array).

        Raises ``RuntimeError`` if already started.
        """
        if self._thread is not None:
            raise RuntimeError("OscilloscopeService already started")
        self._get_joints = get_joints
        self._closed = False
        self._thread = threading.Thread(
            target=self._run,
            name="oscilloscope-sampler",
            daemon=True,
        )
        self._thread.start()
        _logger.info(
            "oscilloscope sampler started at %.1f Hz for %d joints",
            self._sample_hz,
            self._joint_count,
        )

    def stop(self) -> None:
        """Stop the sampler thread and the underlying monitor.  Idempotent."""
        if self._closed:
            return
        self._closed = True
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            if self._thread.is_alive():  # pragma: no cover
                _logger.warning("oscilloscope sampler did not exit in time")
            self._thread = None
        self._monitor.close()
        _logger.info("oscilloscope sampler stopped")

    def latest(self) -> JointStreamFrame | None:
        """Most recent computed frame (positions + velocities + accelerations)."""
        return self._monitor.latest()

    def submit_recorded_positions(
        self,
        positions: np.ndarray,
        period_s: float,
    ) -> None:
        """Record one position sample emitted by the real control loop.

        Velocity and acceleration are intentionally *not* accepted here. The
        monitor derives both from these recorded positions and their monotonic
        timestamps, matching exported oscilloscope data with what was actually
        published to the robot.
        """
        period = float(period_s)
        if not np.isfinite(period) or period <= 0.0:
            raise ValueError("示波器控制采样周期必须为有限正数")
        wall_time = time.monotonic()
        with self._motion_clock_lock:
            # Position is sampled in real time, while its differentiation time
            # is the controller tick. Scheduler lateness is jitter, not robot
            # velocity, and must not be placed in the derivative denominator.
            if (
                self._motion_logical_time == 0.0
                or wall_time - self._motion_wall_time
                > max(3.0 * period, 2.0 / self._sample_hz)
            ):
                logical_time = wall_time
            else:
                logical_time = self._motion_logical_time + period
            self._motion_logical_time = logical_time
            self._motion_wall_time = wall_time
            self._last_motion_sample = wall_time
        self._monitor.submit(positions, t=logical_time)

    def statistics(self) -> dict[str, Any]:
        """Human-readable statistics for diagnostics."""
        return self._monitor.statistics()

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _run(self) -> None:
        interval = 1.0 / self._sample_hz
        get_joints = self._get_joints
        monitor = self._monitor
        while not self._closed:
            started = time.monotonic()
            try:
                if get_joints is not None:
                    # Control-loop position samples own the stream during any
                    # motion. Avoid mixing them with phase-shifted polling
                    # samples, which previously caused false velocity jumps.
                    if started - self._last_motion_sample < 2.0 * interval:
                        time.sleep(max(0.0, interval - (time.monotonic() - started)))
                        continue
                    joints = np.asarray(get_joints(), dtype=float)
                    if joints.shape == (self._joint_count,):
                        monitor.submit(joints, t=started)
            except Exception:
                # A transient read failure must not kill the sampler.
                _logger.exception("oscilloscope sampler read error")
            elapsed = time.monotonic() - started
            sleep_for = max(0.0, interval - elapsed)
            if sleep_for > 0:
                time.sleep(sleep_for)
