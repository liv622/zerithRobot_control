"""Safe no-op implementation of the optional real-robot output port."""

from __future__ import annotations

from typing import Any

import numpy as np


class NullRobotHardware:
    """Keeps simulator installations independent of the vendor SDK."""

    def connect(self, ip: str) -> None:
        raise ValueError("当前进程未启用 Marvin 真机接口")

    def disconnect(self) -> None:
        return

    def enable(self, control_period_ms: int = 5) -> None:
        raise ValueError("当前进程未启用 Marvin 真机接口")

    def disable(self) -> None:
        return

    def release_brake(self) -> None:
        raise ValueError("当前进程未启用 Marvin 真机接口")

    def apply_brake(self) -> None:
        return

    def send_joint_radians(self, joints: np.ndarray) -> None:
        return

    def state(self) -> dict[str, Any]:
        return {
            "available": False,
            "connected": False,
            "enabled": False,
            "brake_released": False,
            "arm": "B",
            "ip": "",
        }
