"""Robot teach-point domain entity."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


MOTION_TYPES = ("MOVL", "MOVJ")
# Retained as public metadata for clients that consume the active execution
# vector; each persisted point now stores both vectors.
VALUE_COUNTS = {"MOVL": 6, "MOVJ": 7}


@dataclass
class TeachPoint:
    point_id: int
    name: str
    motion_type: str
    joint_values: list[float]
    cartesian_values: list[float]
    speed_percent: float = 30.0
    checked: bool = True

    @property
    def values(self) -> list[float]:
        """Execution vector retained for MOVL/MOVJ trajectory code."""
        return self.cartesian_values if self.motion_type == "MOVL" else self.joint_values

    def validate(self, joint_count: int = 7) -> None:
        if self.motion_type not in MOTION_TYPES:
            raise ValueError("运动类型必须是 MOVL 或 MOVJ")
        joints = np.asarray(self.joint_values, dtype=float)
        cartesian = np.asarray(self.cartesian_values, dtype=float)
        if joints.shape != (joint_count,) or not np.all(np.isfinite(joints)):
            raise ValueError(
                f"示教点必须包含 {joint_count} 个有限关节值"
            )
        if cartesian.shape != (6,) or not np.all(np.isfinite(cartesian)):
            raise ValueError("示教点必须包含 6 个有限笛卡尔位姿值")
        if not np.isfinite(self.speed_percent) or not 1.0 <= float(self.speed_percent) <= 100.0:
            raise ValueError("示教点速度百分比必须在 1 到 100 之间")
        self.joint_values = [float(value) for value in joints]
        self.cartesian_values = [float(value) for value in cartesian]
        self.speed_percent = float(self.speed_percent)
        self.name = self.name.strip() or f"P{self.point_id:03d}"
