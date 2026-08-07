"""Dual-arm kinematic model built from a shared-mast dual-arm URDF.

Unlike :class:`robots.generic.GenericUrdfRobotModel`, which models a single
root-to-leaf chain, this model treats a dual-arm URDF as a shared mast plus two
independently controlled six-joint arms.  It implements
:class:`robot_framework.RobotModelProtocol` *dynamically*: every property
follows the currently active arm, so the single-arm framework components
(controller, continuous jog, teach program) operate on whichever arm the
operator selected without any per-arm branching.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from urdf import DualArmChains, detect_dual_arm_chains, fixed_chain_transform

_ARMS = ("left", "right")


class DualArmUrdfModel:
    """A shared mast plus two symmetric arms, one active at a time.

    The mast joints (``mid``) are exposed as auxiliary joints: they stay out of
    IK solving but move both arms, since the arms hang from the mast's tip.
    Each arm is a six-joint serial chain with its own TCP computed from the
    URDF base frame.
    """

    dual_arm = True
    auxiliary_limits: dict[str, tuple[float, float]] = {}
    auxiliary_labels: dict[str, str] = {}

    def __init__(self, chain: DualArmChains) -> None:
        self.chain = chain
        self.urdf_path = chain.urdf_path
        self.robot_name = chain.robot_name
        self.arm_base_origin = fixed_chain_transform(
            tuple(joint for joint in chain.mid_joints if not joint.movable)
        )
        self.tcp_transform = np.eye(4)
        # Label arms geometrically: "left" is the arm whose zero-configuration
        # terminal lies on the negative world-Y side, matching a human operator.
        # The world Y is taken from the full mast + arm chain, because the mast
        # origins may rotate the frame the arms hang from.
        if self._terminal_y(chain) > 0.0:
            self._left_chain, self._right_chain = chain.right_joints, chain.left_joints
            self._left_terminal, self._right_terminal = (
                chain.right_terminal,
                chain.left_terminal,
            )
        else:
            self._left_chain = chain.left_joints
            self._right_chain = chain.right_joints
            self._left_terminal = chain.left_terminal
            self._right_terminal = chain.right_terminal
        self.mid_movable_names = tuple(
            joint.name for joint in chain.mid_joints if joint.movable
        )
        self.aux_joint_names = self.mid_movable_names
        self.auxiliary_limits = {
            joint.name: (joint.lower, joint.upper)
            for joint in chain.mid_joints
            if joint.movable
        }
        self.auxiliary_labels = {
            name: f"中柱/{name}" for name in self.mid_movable_names
        }
        self.initial_configuration = self._build_initial_configuration()
        self._active = "left"
        self._stored = {
            side: self._arm_initial_values(side) for side in _ARMS
        }

    @classmethod
    def from_urdf(cls, path: Path) -> "DualArmUrdfModel":
        chain = detect_dual_arm_chains(path)
        if chain is None:
            raise ValueError(f"不是双臂 URDF：{path}")
        return cls(chain)

    @staticmethod
    def pose(position: np.ndarray, rotation: Rotation) -> np.ndarray:
        value = np.eye(4)
        value[:3, :3] = rotation.as_matrix()
        value[:3, 3] = position
        return value

    # ------------------------------------------------------------------
    # Active-arm protocol (RobotModelProtocol)
    # ------------------------------------------------------------------

    @property
    def active_arm(self) -> str:
        return self._active

    @property
    def arm_joint_names(self) -> tuple[str, ...]:
        return self._movable_names(self._active)

    @property
    def tcp_link_name(self) -> str:
        return (
            self._left_terminal if self._active == "left" else self._right_terminal
        )

    @property
    def lower(self) -> np.ndarray:
        return np.asarray(
            [joint.lower for joint in self._chain(self._active) if joint.movable],
            dtype=float,
        )

    @property
    def upper(self) -> np.ndarray:
        return np.asarray(
            [joint.upper for joint in self._chain(self._active) if joint.movable],
            dtype=float,
        )

    def arm_vector(self, configuration: dict[str, float]) -> np.ndarray:
        return np.asarray(
            [configuration[name] for name in self.arm_joint_names], dtype=float
        )

    def aux_configuration(self, configuration: dict[str, float]) -> dict[str, float]:
        return {name: float(configuration[name]) for name in self.mid_movable_names}

    def tcp_pose(
        self,
        arm: np.ndarray,
        aux: dict[str, float] | None = None,
    ) -> np.ndarray:
        aux_values = [
            float(aux.get(name, 0.0)) for name in self.mid_movable_names
        ] if aux else [0.0] * len(self.mid_movable_names)
        mast = self._chain_pose(self.chain.mid_joints, aux_values)
        return mast @ self._chain_pose(self._chain(self._active), np.asarray(arm))

    def tcp_pose_of(
        self,
        side: str,
        arm: np.ndarray,
        aux: dict[str, float] | None = None,
    ) -> np.ndarray:
        """World-frame TCP of an explicit arm, used for rendering both arms."""
        aux_values = [
            float(aux.get(name, 0.0)) for name in self.mid_movable_names
        ] if aux else [0.0] * len(self.mid_movable_names)
        mast = self._chain_pose(self.chain.mid_joints, aux_values)
        return mast @ self._chain_pose(self._chain(side), np.asarray(arm))

    # ------------------------------------------------------------------
    # Dual-arm API
    # ------------------------------------------------------------------

    def set_active_arm(self, side: str) -> None:
        if side not in _ARMS:
            raise ValueError("手臂必须是 left 或 right")
        self._active = side

    def commit_active_arm(self, arm: np.ndarray) -> None:
        self._stored[self._active] = np.asarray(arm, dtype=float).copy()

    def stored_arm(self, side: str) -> np.ndarray:
        if side not in _ARMS:
            raise ValueError("手臂必须是 left 或 right")
        return self._stored[side].copy()

    def full_configuration(
        self, aux: dict[str, float] | None = None
    ) -> dict[str, float]:
        """All mast + arm joint values as a name-keyed dict for rendering."""
        configuration: dict[str, float] = {}
        for name in self.mid_movable_names:
            configuration[name] = (
                0.0 if aux is None else float(aux.get(name, 0.0))
            )
        for side in _ARMS:
            for name, value in zip(
                self._movable_names(side), self._stored[side]
            ):
                configuration[name] = float(value)
        return configuration

    def reset(self) -> None:
        self._stored = {
            side: self._arm_initial_values(side) for side in _ARMS
        }

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _chain(self, side: str) -> tuple:
        return self._left_chain if side == "left" else self._right_chain

    def _movable_names(self, side: str) -> tuple[str, ...]:
        return tuple(
            joint.name for joint in self._chain(side) if joint.movable
        )

    def _arm_initial_values(self, side: str) -> np.ndarray:
        return np.asarray(
            [
                float(self.initial_configuration[name])
                for name in self._movable_names(side)
            ],
            dtype=float,
        )

    def _build_initial_configuration(self) -> dict[str, float]:
        """Zero mast, arms spread horizontally so the dual arm is visible.

        Each arm's shoulder joint receives an opposite rotation, unfolding the
        two arms symmetrically instead of leaving them hanging against the mast.
        """
        configuration: dict[str, float] = {
            name: 0.0 for name in self.mid_movable_names
        }
        for side, sign in (("left", 1.0), ("right", -1.0)):
            for index, name in enumerate(self._movable_names(side)):
                configuration[name] = (
                    sign * np.deg2rad(90.0) if index == 0 else 0.0
                )
        return configuration

    @staticmethod
    def _terminal_y(chain: DualArmChains) -> float:
        mid_pose = DualArmUrdfModel._chain_pose(
            chain.mid_joints,
            np.zeros(sum(joint.movable for joint in chain.mid_joints)),
        )
        left_pose = DualArmUrdfModel._chain_pose(
            chain.left_joints,
            np.zeros(sum(joint.movable for joint in chain.left_joints)),
        )
        return float((mid_pose @ left_pose)[1, 3])

    @staticmethod
    def _chain_pose(chain: tuple, values: np.ndarray) -> np.ndarray:
        result = np.eye(4)
        index = 0
        for joint in chain:
            result = result @ joint.origin
            if not joint.movable:
                continue
            value = float(values[index])
            index += 1
            motion = np.eye(4)
            if joint.joint_type == "prismatic":
                motion[:3, 3] = joint.axis * value
            else:
                motion[:3, :3] = (
                    Rotation.from_rotvec(joint.axis * value).as_matrix()
                )
            result = result @ motion
        return result
