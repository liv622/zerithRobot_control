"""Append-only motion audit trail.

Every accepted or rejected motion decision is recorded as one JSON object per
line (JSONL).  This is separate from :mod:`robot_logging.setup` on purpose:
operator log text is for reading, the audit trail is for reconstructing what
the robot was commanded to do and why a move was refused.

Writes happen on a dedicated background thread.  A motion loop running at
200 Hz must never block on disk I/O, and a full or read-only disk must never
be able to stop the robot loop, so ``record`` only ever appends to a bounded
in-memory queue and returns immediately.
"""

from __future__ import annotations

import json
import queue
import threading
import time
from pathlib import Path
from typing import Any

from .setup import get_logger

_logger = get_logger("audit")

_STOP = object()


class NullMotionAuditLog:
    """Audit sink used by tests and by installations without an audit file."""

    def record(self, event: str, **fields: Any) -> None:
        return

    def flush(self, timeout_s: float = 1.0) -> None:
        return

    def close(self) -> None:
        return


class MotionAuditLog:
    """Buffered JSONL audit writer.

    ``max_pending`` bounds memory if the consumer thread cannot keep up.  When
    the queue is full the event is dropped and counted rather than blocking the
    caller; the drop count is reported in the next successfully written record
    so a gap in the trail is always visible.
    """

    def __init__(
        self,
        path: Path,
        *,
        max_pending: int = 8192,
        max_bytes: int = 32 * 1024 * 1024,
    ) -> None:
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._max_bytes = int(max_bytes)
        self._queue: queue.Queue[Any] = queue.Queue(maxsize=max(1, max_pending))
        self._dropped = 0
        self._drop_lock = threading.Lock()
        self._closed = threading.Event()
        self._thread = threading.Thread(
            target=self._consume,
            name="robot-motion-audit",
            daemon=True,
        )
        self._thread.start()

    def record(self, event: str, **fields: Any) -> None:
        """Queue one audit record.  Never raises, never blocks."""
        if self._closed.is_set():
            return
        payload = {
            "time": time.time(),
            "monotonic": time.monotonic(),
            "event": str(event),
            **fields,
        }
        try:
            self._queue.put_nowait(payload)
        except queue.Full:
            with self._drop_lock:
                self._dropped += 1

    def _take_dropped(self) -> int:
        with self._drop_lock:
            value, self._dropped = self._dropped, 0
        return value

    def _consume(self) -> None:
        while True:
            item = self._queue.get()
            if item is _STOP:
                self._queue.task_done()
                return
            try:
                self._write(item)
            except OSError as exc:
                # A failing audit disk must not stop motion.  Report once per
                # failure through the normal log and keep draining the queue.
                _logger.warning("运动审计写入失败：%s", exc)
            finally:
                self._queue.task_done()

    def _write(self, payload: dict[str, Any]) -> None:
        dropped = self._take_dropped()
        if dropped:
            payload = {**payload, "dropped_records": dropped}
        line = json.dumps(payload, ensure_ascii=False, default=str)
        self._rotate_if_needed(len(line) + 1)
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(line + "\n")

    def _rotate_if_needed(self, incoming_bytes: int) -> None:
        if self._max_bytes <= 0 or not self.path.exists():
            return
        if self.path.stat().st_size + incoming_bytes <= self._max_bytes:
            return
        backup = self.path.with_suffix(self.path.suffix + ".1")
        backup.unlink(missing_ok=True)
        self.path.replace(backup)

    def flush(self, timeout_s: float = 1.0) -> None:
        """Block until queued records are written, bounded by ``timeout_s``."""
        deadline = time.monotonic() + max(0.0, timeout_s)
        while not self._queue.empty() and time.monotonic() < deadline:
            time.sleep(0.005)

    def close(self) -> None:
        if self._closed.is_set():
            return
        self._closed.set()
        try:
            self._queue.put_nowait(_STOP)
        except queue.Full:
            # Drain one slot so the sentinel is guaranteed to be delivered.
            try:
                self._queue.get_nowait()
                self._queue.task_done()
                self._queue.put_nowait(_STOP)
            except (queue.Empty, queue.Full):
                return
        self._thread.join(timeout=2.0)
