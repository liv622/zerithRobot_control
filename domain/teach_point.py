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
    checked: bool = True

    @property
    def values(self) -> list[float]:
        """Execution vector retained for MOVL/MOVJ trajectory code."""
        return self.cartesian_values if self.motion_type == "MOVL" else self.joint_values

    def validate(self) -> None:
        if self.motion_type not in MOTION_TYPES:
            raise ValueError("运动类型必须是 MOVL 或 MOVJ")
        joints = np.asarray(self.joint_values, dtype=float)
        cartesian = np.asarray(self.cartesian_values, dtype=float)
        if joints.shape != (7,) or not np.all(np.isfinite(joints)):
            raise ValueError(
                "示教点必须包含 7 个有限关节角度"
            )
        if cartesian.shape != (6,) or not np.all(np.isfinite(cartesian)):
            raise ValueError("示教点必须包含 6 个有限笛卡尔位姿值")
        self.joint_values = [float(value) for value in joints]
        self.cartesian_values = [float(value) for value in cartesian]
        self.name = self.name.strip() or f"P{self.point_id:03d}"
