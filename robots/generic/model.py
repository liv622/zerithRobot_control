"""Serial-arm model built from any URDF via the shared URDF layer."""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from urdf import UrdfChain, parse_urdf_chain


class GenericUrdfRobotModel:
    """A movable root-to-leaf chain selected from any valid serial-arm URDF.

    URDF parsing and terminal-link selection live in :mod:`urdf.kinematics`;
    this class only adds the forward kinematics and the configuration mapping
    required by :class:`robot_framework.RobotModelProtocol`.
    """

    auxiliary_limits: dict[str, tuple[float, float]] = {}
    auxiliary_labels: dict[str, str] = {}
    aux_joint_names: tuple[str, ...] = ()

    def __init__(self, path: Path) -> None:
        self.chain: UrdfChain = parse_urdf_chain(Path(path))
        self.urdf_path = self.chain.urdf_path
        self.robot_name = self.chain.robot_name
        self.arm_joint_names = self.chain.joint_names
        self.tcp_link_name = self.chain.terminal_link
        self.lower = self.chain.lower_limits
        self.upper = self.chain.upper_limits
        self.initial_configuration = {
            name: float(np.clip(0.0, lower, upper))
            for name, lower, upper in zip(
                self.arm_joint_names, self.lower, self.upper
            )
        }
        self.arm_base_origin = self.chain.base_transform()
        self.tcp_transform = self.chain.tool_transform()

    @classmethod
    def from_urdf(cls, path: Path) -> "GenericUrdfRobotModel":
        return cls(path)

    def joint_velocity_limits(self, fallback_rad_s: float) -> np.ndarray:
        """Per-joint velocity ceilings declared by the URDF."""
        return self.chain.velocity_limits(fallback_rad_s)

    def arm_vector(self, configuration: dict[str, float]) -> np.ndarray:
        return np.asarray(
            [configuration[name] for name in self.arm_joint_names], dtype=float
        )

    def aux_configuration(self, configuration: dict[str, float]) -> dict[str, float]:
        return {}

    def configuration(self, arm: np.ndarray, aux: dict[str, float]) -> dict[str, float]:
        return {
            name: float(value) for name, value in zip(self.arm_joint_names, arm)
        }

    def _forward_kinematics(
        self, arm: np.ndarray, *, stop_after_last_joint: bool = False
    ) -> np.ndarray:
        """Accumulate the chain transform for ``arm``.

        ``stop_after_last_joint`` returns the final movable-joint frame (the
        mechanical flange) instead of the terminal link, which is what a user
        TCP offset is measured from.
        """
        values = np.asarray(arm, dtype=float)
        if values.shape != (len(self.arm_joint_names),):
            raise ValueError(
                f"机械臂关节值必须包含 {len(self.arm_joint_names)} 项"
            )
        if not np.all(np.isfinite(values)):
            raise ValueError("机械臂关节值必须为有限数值")
        result = np.eye(4)
        index = 0
        for joint in self.chain.joints:
            result = result @ joint.origin
            if not joint.movable:
                continue
            value = float(values[index])
            index += 1
            motion = np.eye(4)
            if joint.joint_type == "prismatic":
                motion[:3, 3] = joint.axis * value
            else:
                motion[:3, :3] = Rotation.from_rotvec(joint.axis * value).as_matrix()
            result = result @ motion
            if stop_after_last_joint and index == len(self.arm_joint_names):
                return result
        return result

    def flange_pose(
        self, arm: np.ndarray, aux: dict[str, float] | None = None
    ) -> np.ndarray:
        return self._forward_kinematics(arm, stop_after_last_joint=True)

    def tcp_pose(
        self, arm: np.ndarray, aux: dict[str, float] | None = None
    ) -> np.ndarray:
        return self._forward_kinematics(arm)

    @staticmethod
    def pose(position: np.ndarray, rotation: Rotation) -> np.ndarray:
        value = np.eye(4)
        value[:3, :3] = rotation.as_matrix()
        value[:3, 3] = position
        return value
