"""Real-time joint-state monitor with derived velocity / acceleration.

A dedicated background thread consumes joint-position samples produced by
the motion loop, maintains a fixed-size ring buffer, and computes first and
second finite-difference derivatives.  The motion thread only performs a
non-blocking queue handoff, so the monitor never adds jitter to the control
loop.

Read-side consumers (SSE, diagnostics) access the latest computed frame
through a lock-guarded cache.
"""

from __future__ import annotations

import queue
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np

from robot_logging import get_logger

_logger = get_logger("realtime.monitor")

# Upper bound so a stalled monitor thread doesn't consume unbounded memory.
_MAX_QUEUE_SIZE = 4096


@dataclass
class JointSample:
    """One snapshot of joint positions with a wall-clock timestamp."""

    t: float
    positions: np.ndarray


@dataclass
class JointStreamFrame:
    """The latest computed frame ready for streaming to a consumer."""

    t: float
    dt: float                     # seconds since previous sample
    positions: np.ndarray         # radians
    velocities: np.ndarray        # rad / s
    accelerations: np.ndarray     # rad / s²

    def as_json(self) -> dict[str, Any]:
        return {
            "t": self.t,
            "dt": self.dt,
            "positions": self.positions.tolist(),
            "velocities": self.velocities.tolist(),
            "accelerations": self.accelerations.tolist(),
        }


class JointMonitor:
    """Bounded history of joint positions with finite-difference derivatives.

    ``submit()`` is non-blocking: it hands the sample off to a bounded queue
    and returns immediately.  A dedicated background thread drains the queue,
    updates the ring buffer, and publishes the latest computed frame.

    Parameters
    ----------
    capacity:
        Number of samples to retain.  At 200 Hz, 2000 samples ≈ 10 seconds.
    joint_count:
        Number of actuated joints.  Determined from the first sample if 0.
    """

    def __init__(
        self,
        capacity: int = 2000,
        joint_count: int = 0,
    ) -> None:
        self._capacity = max(32, int(capacity))
        self._joint_count = int(joint_count)
        self._ring: deque[JointSample] = deque(maxlen=self._capacity)
        self._inbox: queue.Queue = queue.Queue(maxsize=_MAX_QUEUE_SIZE)
        self._lock = threading.Lock()
        self._latest: JointStreamFrame | None = None
        self._closed = False
        self._thread = threading.Thread(
            target=self._run,
            name="joint-monitor",
            daemon=True,
        )
        self._thread.start()

    # ------------------------------------------------------------------
    # Write side (motion thread) — non-blocking
    # ------------------------------------------------------------------

    def submit(self, positions: np.ndarray, t: float | None = None) -> None:
        """Record one joint-position vector.  Never blocks.

        Called by the motion loop at the interpolation frequency (typically
        200 Hz).  If the inbox is full the sample is silently dropped; this
        is a back-pressure safety valve and should never happen in practice.
        """
        if self._closed:
            return
        if self._joint_count == 0:
            self._joint_count = len(positions)
        try:
            self._inbox.put_nowait(
                JointSample(
                    t=t if t is not None else time.monotonic(),
                    positions=np.array(positions, dtype=float, copy=True),
                )
            )
        except queue.Full:
            _logger.warning(
                "monitor inbox full (%d items) — dropping sample",
                _MAX_QUEUE_SIZE,
            )

    # ------------------------------------------------------------------
    # Read side (I/O / SSE thread)
    # ------------------------------------------------------------------

    def latest(self) -> JointStreamFrame | None:
        """Return the most recent computed frame (or ``None`` before any sample)."""
        with self._lock:
            return self._latest

    def snapshot(self) -> list[JointSample]:
        """Return a shallow copy of the full ring buffer."""
        with self._lock:
            return list(self._ring)

    @property
    def joint_count(self) -> int:
        with self._lock:
            return self._joint_count

    # ------------------------------------------------------------------
    # Background processing thread
    # ------------------------------------------------------------------

    def _run(self) -> None:
        while True:
            try:
                sample = self._inbox.get(timeout=0.5)
            except queue.Empty:
                if self._closed:
                    return
                continue
            if self._closed and self._inbox.empty():
                return
            with self._lock:
                self._ring.append(sample)
                self._latest = self._compute_frame_locked()

    # ------------------------------------------------------------------
    # Derivative computation
    # ------------------------------------------------------------------

    def _compute_frame_locked(self) -> JointStreamFrame | None:
        """Requires ``_lock`` to be held."""
        if len(self._ring) < 2:
            if not self._ring:
                return None
            sample = self._ring[-1]
            zeros = np.zeros(self._joint_count, dtype=float)
            return JointStreamFrame(
                t=sample.t,
                dt=0.0,
                positions=sample.positions.copy(),
                velocities=zeros,
                accelerations=zeros,
            )

        items = list(self._ring)
        positions = np.array([s.positions for s in items])  # (N, J)
        times = np.array([s.t for s in items])              # (N,)

        dt = times[-1] - times[-2]
        if dt <= 0:
            dt = 0.005  # fallback: assume 200 Hz

        # Velocity: first-order backward difference on the two most recent
        vel = (positions[-1] - positions[-2]) / dt

        # Acceleration: three-point central / two-point backward difference
        if len(items) >= 3:
            dt0 = times[-2] - times[-3]
            if dt0 <= 0:
                dt0 = dt
            vel_prev = (positions[-2] - positions[-3]) / dt0
            acc = (vel - vel_prev) / ((dt + dt0) / 2.0)
        else:
            acc = np.zeros(self._joint_count, dtype=float)

        return JointStreamFrame(
            t=items[-1].t,
            dt=float(dt),
            positions=positions[-1].copy(),
            velocities=vel,
            accelerations=acc,
        )

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def close(self) -> None:
        """Signal the monitor thread to stop.  Idempotent."""
        if self._closed:
            return
        self._closed = True
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():  # pragma: no cover
            _logger.warning("monitor thread did not exit within timeout")

    # ------------------------------------------------------------------
    # Statistics
    # ------------------------------------------------------------------

    def statistics(self) -> dict[str, Any]:
        with self._lock:
            return {
                "samples": len(self._ring),
                "capacity": self._capacity,
                "joint_count": self._joint_count,
                "latest_t": self._latest.t if self._latest else None,
                "latest_dt": self._latest.dt if self._latest else None,
                "inbox_size": self._inbox.qsize(),
            }
