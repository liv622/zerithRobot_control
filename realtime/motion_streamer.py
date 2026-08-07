"""Deliver interpolated joint samples to hardware without blocking motion.

The motion loop produces one joint sample per control period.  Sending it can
block: a socket write, a vendor SDK call, or a JSON state file write all take
unpredictable time, and a raised exception would otherwise kill the motion
thread mid-trajectory and leave the robot commanded to a stale position.

:class:`MotionStreamer` puts a bounded FIFO between the two.  Every Double-S
sample matters: dropping one doubles the position increment seen by a drive
whose velocity estimator uses a fixed control period.  The streamer therefore
preserves order and latches a fault on overflow instead of silently creating a
commanded velocity step.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Callable, Protocol

import numpy as np

from robot_logging import get_logger

_logger = get_logger("realtime.motion_streamer")


class JointSampleSink(Protocol):
    """Anything that accepts a joint-space command, in radians."""

    def send_joint_radians(self, joints: np.ndarray) -> None: ...


@dataclass
class StreamStatistics:
    """Delivery behaviour of the streamer, surfaced in the pendant diagnostics."""

    submitted: int = 0
    delivered: int = 0
    dropped: int = 0
    failures: int = 0
    last_error: str = ""
    max_send_latency_s: float = 0.0
    _lock: threading.Lock = field(default_factory=threading.Lock, repr=False)

    def as_json(self) -> dict[str, float | int | str]:
        with self._lock:
            return {
                "submitted": self.submitted,
                "delivered": self.delivered,
                "dropped": self.dropped,
                "failures": self.failures,
                "last_error": self.last_error,
                "max_send_latency_ms": self.max_send_latency_s * 1000.0,
            }


class MotionStreamer:
    """Bounded-latency bridge between a motion loop and a joint sink.

    ``queue_depth`` is intentionally small.  A deep queue would only let the
    robot fall further behind the commanded trajectory before anyone noticed;
    a shallow one surfaces a slow sink as visible drops instead.
    """

    def __init__(
        self,
        sink: JointSampleSink | None = None,
        *,
        queue_depth: int = 4,
        minimum_send_period_s: float = 0.0,
        on_failure: Callable[[Exception], None] | None = None,
        name: str = "robot-motion-streamer",
    ) -> None:
        self.statistics = StreamStatistics()
        self._sink = sink
        self._queue: deque[np.ndarray] = deque(maxlen=max(1, int(queue_depth)))
        self._condition = threading.Condition()
        self._closed = False
        self._on_failure = on_failure
        self._minimum_send_period_s = max(0.0, float(minimum_send_period_s))
        self._faulted = False
        self._thread = threading.Thread(target=self._run, name=name, daemon=True)
        self._thread.start()

    @property
    def sink(self) -> JointSampleSink | None:
        return self._sink

    def set_sink(self, sink: JointSampleSink | None) -> None:
        """Swap the destination, e.g. when real hardware connects or drops.

        Queued samples are discarded: they were computed for the previous sink
        and must not be replayed into a newly connected robot.
        """
        with self._condition:
            self._sink = sink
            self._queue.clear()
            self._faulted = False
            self._condition.notify()

    def set_minimum_send_period(self, period_s: float) -> None:
        """Apply a new controller sampling period to subsequent samples."""
        value = float(period_s)
        if not np.isfinite(value) or value < 0.0:
            raise ValueError("关节发送周期必须是有限非负数")
        with self._condition:
            self._minimum_send_period_s = value

    def submit(self, joints: np.ndarray) -> None:
        """Hand off one joint sample.  Never blocks and never raises."""
        if self._closed:
            return
        sample = np.array(joints, dtype=float, copy=True)
        with self._condition:
            if self._faulted:
                return
            if self._sink is None:
                # Nothing to deliver to; count it so simulation-only runs still
                # report an accurate submitted total.
                self.statistics.submitted += 1
                return
            if len(self._queue) == self._queue.maxlen:
                self._faulted = True
                self.statistics.dropped += 1
                self.statistics.failures += 1
                self.statistics.last_error = (
                    "关节下发队列溢出；已停止接受轨迹点，未丢弃中间关节值"
                )
                _logger.error(self.statistics.last_error)
                if self._on_failure is not None:
                    try:
                        self._on_failure(RuntimeError(self.statistics.last_error))
                    except Exception:  # noqa: BLE001
                        _logger.exception("下发溢出回调本身抛出异常")
                return
            self._queue.append(sample)
            self.statistics.submitted += 1
            self._condition.notify()

    def _run(self) -> None:
        previous_send_started = 0.0
        while True:
            with self._condition:
                while not self._queue and not self._closed:
                    self._condition.wait(timeout=0.5)
                if self._closed and not self._queue:
                    return
                sample = self._queue.popleft()
                sink = self._sink
            if sink is None:
                continue
            remaining = (
                previous_send_started
                + self._minimum_send_period_s
                - time.monotonic()
            )
            if remaining > 0.0:
                # Avoid a full-period oversleep at high controller rates.
                coarse = max(0.0, remaining - 0.0002)
                if coarse > 0.0:
                    time.sleep(coarse)
                deadline = previous_send_started + self._minimum_send_period_s
                while time.monotonic() < deadline:
                    pass
            started = time.monotonic()
            previous_send_started = started
            try:
                sink.send_joint_radians(sample)
                self.statistics.delivered += 1
            except Exception as exc:  # noqa: BLE001 - a sink must never kill motion
                self.statistics.failures += 1
                self.statistics.last_error = f"{type(exc).__name__}: {exc}"
                _logger.error("关节指令下发失败：%s", exc)
                if self._on_failure is not None:
                    try:
                        self._on_failure(exc)
                    except Exception:  # noqa: BLE001 - callback must not propagate
                        _logger.exception("下发失败回调本身抛出异常")
            latency = time.monotonic() - started
            self.statistics.max_send_latency_s = max(
                self.statistics.max_send_latency_s, latency
            )

    def drain(self, timeout_s: float = 1.0) -> bool:
        """Wait until the queue empties.  Returns ``False`` on timeout."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        while time.monotonic() < deadline:
            with self._condition:
                if not self._queue:
                    return True
            time.sleep(0.002)
        return False

    def close(self) -> None:
        """Stop the sender thread.  Idempotent and safe to call from any thread."""
        if self._closed:
            return
        with self._condition:
            self._closed = True
            self._condition.notify_all()
        self._thread.join(timeout=2.0)
        if self._thread.is_alive():  # pragma: no cover - sink wedged in a syscall
            _logger.warning("下发线程未在超时内退出")
