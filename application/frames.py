"""Selected user base and TCP coordinate-frame conversions."""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from .ports import CoordinateFrameRepository


def _pose(values: list[float] | np.ndarray) -> np.ndarray:
    data = np.asarray(values, dtype=float)
    if data.shape != (6,) or not np.all(np.isfinite(data)):
        raise ValueError("坐标系位姿必须包含 6 个有限数值")
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler("xyz", data[3:], degrees=True).as_matrix()
    result[:3, 3] = data[:3]
    return result


def _values(transform: np.ndarray) -> list[float]:
    return [float(value) for value in np.r_[
        transform[:3, 3],
        Rotation.from_matrix(transform[:3, :3]).as_euler("xyz", degrees=True),
    ]]


class CoordinateFrameService:
    """Stores user frames relative to base_link and TCP frames relative to flange."""

    def __init__(
        self,
        default_tcp: np.ndarray,
        repository: CoordinateFrameRepository | None = None,
        arm_base_transform: Callable[[], np.ndarray] | None = None,
    ) -> None:
        self.default_tcp = default_tcp.copy()
        self.repository = repository
        self.arm_base_transform = arm_base_transform
        self.bases: dict[str, np.ndarray] = {
            "base_link": np.eye(4),
            "arm_base_link": np.eye(4),
        }
        self.tcps: dict[str, np.ndarray] = {"URDF_TCP": default_tcp.copy()}
        self.active_base = "arm_base_link"
        self.active_tcp = "URDF_TCP"
        self._restore()

    def _restore(self) -> None:
        if self.repository is None:
            return
        saved = self.repository.load()
        for item in saved.get("bases", []):
            if isinstance(item, dict) and isinstance(item.get("name"), str):
                try:
                    self.create_base(item["name"], item["values"])
                except (TypeError, ValueError):
                    continue
        for item in saved.get("tcps", []):
            if isinstance(item, dict) and isinstance(item.get("name"), str):
                try:
                    self.create_tcp(item["name"], item["values"])
                except (TypeError, ValueError):
                    continue
        base, tcp = saved.get("active_base"), saved.get("active_tcp")
        if base in self.bases and tcp in self.tcps:
            self.active_base, self.active_tcp = base, tcp

    def _save(self) -> None:
        if self.repository is not None:
            self.repository.save(self.state())

    def create_base(self, name: str, values: list[float]) -> None:
        name = name.strip()
        if not name or name in {"base_link", "arm_base_link"}:
            raise ValueError("用户基坐标系名称无效")
        self.bases[name] = _pose(values)
        self._save()

    def create_tcp(self, name: str, values: list[float]) -> None:
        name = name.strip()
        if not name or name == "URDF_TCP":
            raise ValueError("用户 TCP 名称无效")
        self.tcps[name] = _pose(values)
        self._save()

    def select(self, base: str, tcp: str) -> None:
        if base not in self.bases or tcp not in self.tcps:
            raise ValueError("所选用户坐标系或 TCP 不存在")
        self.active_base, self.active_tcp = base, tcp
        self._save()

    def _base_transform(self, name: str) -> np.ndarray:
        if name == "arm_base_link" and self.arm_base_transform is not None:
            return self.arm_base_transform()
        return self.bases[name]

    def display_from_default(self, default_pose: np.ndarray) -> np.ndarray:
        selected_tcp = default_pose @ np.linalg.inv(self.default_tcp) @ self.tcps[self.active_tcp]
        return np.linalg.inv(self._base_transform(self.active_base)) @ selected_tcp

    def default_from_display(self, values: list[float] | np.ndarray) -> np.ndarray:
        selected_tcp = self._base_transform(self.active_base) @ _pose(values)
        return selected_tcp @ np.linalg.inv(self.tcps[self.active_tcp]) @ self.default_tcp

    def pose_values(self, default_pose: np.ndarray) -> list[float]:
        return _values(self.display_from_default(default_pose))

    def state(self) -> dict[str, Any]:
        return {
            "active_base": self.active_base,
            "active_tcp": self.active_tcp,
            "bases": [
                {"name": name, "values": _values(self._base_transform(name))}
                for name in self.bases
            ],
            "tcps": [{"name": name, "values": _values(frame)} for name, frame in self.tcps.items()],
        }
