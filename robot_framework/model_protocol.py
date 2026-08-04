"""Protocol implemented by every robot-model plugin."""

from __future__ import annotations

from pathlib import Path
from typing import Protocol

import numpy as np
from scipy.spatial.transform import Rotation


class RobotModelProtocol(Protocol):
    urdf_path: Path | None
    arm_joint_names: tuple[str, ...]
    aux_joint_names: tuple[str, ...]
    initial_configuration: dict[str, float]
    tcp_link_name: str
    auxiliary_limits: dict[str, tuple[float, float]]
    auxiliary_labels: dict[str, str]
    lower: np.ndarray
    upper: np.ndarray

    def arm_vector(self, configuration: dict[str, float]) -> np.ndarray: ...

    def aux_configuration(
        self,
        configuration: dict[str, float],
    ) -> dict[str, float]: ...

    def tcp_pose(
        self,
        arm: np.ndarray,
        aux: dict[str, float] | None = None,
    ) -> np.ndarray: ...

    def pose(self, position: np.ndarray, rotation: Rotation) -> np.ndarray: ...
