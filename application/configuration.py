"""Configuration-profile use cases for the teach pendant."""

from __future__ import annotations

from dataclasses import asdict
from typing import Any

import numpy as np

from .contracts import ApplicationSettings
from .ports import ConfigurationRepository


PROFILE_FIELDS = (
    "live_solve",
    "orientation_lock",
    "auto_recovery",
    "recovery_count",
    "guide_enabled",
    "guide_strength",
    "point_duration_s",
    "trajectory_frequency_hz",
    "loop_teach_program",
    "speed_percent",
    "max_linear_speed_mm_s",
    "max_angular_speed_deg_s",
    "max_joint_speed_deg_s",
    "command_delay_s",
)


class ConfigurationService:
    def __init__(
        self,
        repository: ConfigurationRepository | None,
        settings: ApplicationSettings,
    ) -> None:
        self.repository = repository
        self.settings = settings
        self.active_name = ""

    def _require_repository(self) -> ConfigurationRepository:
        if self.repository is None:
            raise ValueError("当前应用未配置配置文件仓储")
        return self.repository

    def validate_motion_values(self, values: dict[str, Any]) -> dict[str, float]:
        result = {
            "speed_percent": float(values["speed_percent"]),
            "max_linear_speed_mm_s": float(values["max_linear_speed_mm_s"]),
            "max_angular_speed_deg_s": float(values["max_angular_speed_deg_s"]),
            "max_joint_speed_deg_s": float(values["max_joint_speed_deg_s"]),
            "command_delay_s": float(values["command_delay_s"]),
            "trajectory_frequency_hz": float(
                values.get(
                    "trajectory_frequency_hz",
                    self.settings.trajectory_frequency_hz,
                )
            ),
        }
        if not all(np.isfinite(value) for value in result.values()):
            raise ValueError("运动参数必须是有限数值")
        if not 1.0 <= result["speed_percent"] <= 100.0:
            raise ValueError("速度百分比必须在 1% 到 100% 之间")
        if not 1.0 <= result["max_linear_speed_mm_s"] <= 2000.0:
            raise ValueError("最大线速度必须在 1 到 2000 mm/s 之间")
        if not 1.0 <= result["max_angular_speed_deg_s"] <= 360.0:
            raise ValueError("最大角速度必须在 1 到 360 deg/s 之间")
        if not 1.0 <= result["max_joint_speed_deg_s"] <= 360.0:
            raise ValueError("最大关节速度必须在 1 到 360 deg/s 之间")
        if not 0.0 <= result["command_delay_s"] <= 60.0:
            raise ValueError("动作间延时必须在 0 到 60 s 之间")
        if not 50.0 <= result["trajectory_frequency_hz"] <= 1000.0:
            raise ValueError("示教控制器采样频率必须在 50 到 1000 Hz 之间")
        return result

    def save(self, name: str, guide: np.ndarray) -> None:
        values = {
            key: value
            for key, value in asdict(self.settings).items()
            if key in PROFILE_FIELDS
        }
        values["guide_degrees"] = [
            float(value) for value in np.rad2deg(guide)
        ]
        self._require_repository().save(name, values)
        self.active_name = name.strip()

    def load(self, name: str, joint_count: int) -> np.ndarray | None:
        values = self._require_repository().get(name)
        motion = self.validate_motion_values(values)
        for key in PROFILE_FIELDS:
            if key in values:
                setattr(self.settings, key, values[key])
        for key, value in motion.items():
            setattr(self.settings, key, value)
        guide = values.get("guide_degrees")
        self.active_name = name
        if guide is None:
            return None
        array = np.asarray(guide, dtype=float)
        if array.shape != (joint_count,) or not np.all(np.isfinite(array)):
            raise ValueError("配置文件中的臂形参考数据无效")
        return np.deg2rad(array)

    def delete(self, name: str) -> None:
        self._require_repository().delete(name)
        if self.active_name == name:
            self.active_name = ""

    def state(self) -> dict[str, Any]:
        names = [] if self.repository is None else self.repository.names()
        return {"active": self.active_name, "profiles": names}
