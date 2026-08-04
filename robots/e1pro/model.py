"""E1-PRO kinematic model used by the generic robot framework."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from scipy.spatial.transform import Rotation

ARM_JOINTS = tuple(f"Joint{i}_J" for i in range(1, 8))
AUX_JOINTS = (
    "platform_joint",
    "box01_joint",
    "left_dainchi_joint",
    "right_dianchi_joint",
    "boxs02_joint",
)

INITIAL_CONFIGURATION: dict[str, float] = {
    "Joint1_J": 0.0,
    "Joint2_J": -0.5,
    "Joint3_J": 0.0,
    "Joint4_J": -2.0,
    "Joint5_J": 0.0,
    "Joint6_J": -0.61,
    "Joint7_J": 0.0,
    "platform_joint": 0.0,
    "box01_joint": 0.0,
    "left_dainchi_joint": 0.0,
    "right_dianchi_joint": 0.0,
    "boxs02_joint": 0.0,
}

TCP_EXTENSION_M = 0.17167


def transform(rotation: Rotation | None = None, translation=(0.0, 0.0, 0.0)) -> np.ndarray:
    value = np.eye(4)
    if rotation is not None:
        value[:3, :3] = rotation.as_matrix()
    value[:3, 3] = translation
    return value


def axis_rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    return transform(Rotation.from_rotvec(np.asarray(axis, dtype=float) * angle))


@dataclass(frozen=True)
class JointSpec:
    name: str
    axis: np.ndarray
    origin: np.ndarray
    lower: float
    upper: float


class RobotModel:
    """Minimal 7-DoF serial arm plus independently controlled mechanisms."""

    arm_joint_names = ARM_JOINTS
    aux_joint_names = AUX_JOINTS
    initial_configuration = INITIAL_CONFIGURATION
    tcp_link_name = "zhijian_link"
    auxiliary_limits = {
        "platform_joint": (-0.1, 0.5),
        "box01_joint": (-0.7, 0.7),
        "left_dainchi_joint": (0.0, 0.5),
        "right_dianchi_joint": (0.0, 0.5),
        "boxs02_joint": (0.0, 0.5),
    }
    auxiliary_labels = {
        "platform_joint": "机械臂/平台升降",
        "box01_joint": "料箱 01 横移",
        "left_dainchi_joint": "左电池升降",
        "right_dianchi_joint": "右电池升降",
        "boxs02_joint": "料箱 02 斜向移动",
    }

    def __init__(self, urdf_path: Path | None = None) -> None:
        self.urdf_path = urdf_path
        # ``from_urdf`` replaces these fallback values by parsing the active
        # runtime-selected URDF, keeping that file authoritative.
        self.arm_base_origin = transform(translation=(0.51352, -0.031, 0.31))
        origins = (
            (0.0, 0.0, 0.0),
            (0.0, 0.0, 0.1745),
            (0.0, 0.0, 0.1485),
            (-0.096769, 0.0, 0.23962),
            (0.018, 0.00325, 0.1215),
            (0.0, 0.0, 0.2925),
            (0.0, 0.0, 0.0),
        )
        axes = (
            (0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, 1.0, 0.0),
            (1.0, 0.0, 0.0),
        )
        limits = (
            (-2.9671, 2.9671),
            (-2.0944, 2.0944),
            (-2.9671, 2.9671),
            (-3.1416, 1.047197),
            (-2.9671, 2.9671),
            (-1.0472, 1.0472),
            (-1.5708, 1.5708),
        )
        self.joints = tuple(
            JointSpec(name, np.array(axis), transform(translation=origin), *limit)
            for name, axis, origin, limit in zip(ARM_JOINTS, axes, origins, limits)
        )
        self.lower = np.array([joint.lower for joint in self.joints])
        self.upper = np.array([joint.upper for joint in self.joints])
        self.tcp_transform = transform(translation=(0.0, 0.0, TCP_EXTENSION_M))

    @classmethod
    def from_urdf(cls, path: Path) -> "RobotModel":
        """Parse the real arm chain, fixed base transform, TCP, and limits."""
        root = ElementTree.parse(path).getroot()
        joint_nodes = {node.attrib["name"]: node for node in root.findall("joint")}
        joint_names = set(joint_nodes)
        arm_names = next(
            (
                names
                for names in (
                    tuple(f"Joint{i}_J" for i in range(1, 8)),
                    tuple(f"Joint{i}_L" for i in range(1, 8)),
                    tuple(f"Joint{i}_R" for i in range(1, 8)),
                )
                if set(names).issubset(joint_names)
            ),
            None,
        )
        if arm_names is None:
            raise ValueError("URDF missing a complete seven-joint E1-PRO arm chain")
        tcp_joint = next(
            (name for name in ("zhijian_joint", "end_effector_joint") if name in joint_names),
            None,
        )
        if tcp_joint is None:
            raise ValueError("URDF missing terminal fixed TCP joint")
        fixed = joint_nodes[tcp_joint].find("origin")
        if fixed is None:
            raise ValueError("URDF terminal TCP joint has no origin")
        xyz = np.fromstring(fixed.attrib.get("xyz", ""), sep=" ")
        if xyz.shape != (3,):
            raise ValueError("URDF terminal TCP joint has invalid origin")
        def values(text: str | None, default: tuple[float, float, float]) -> np.ndarray:
            if text is None:
                return np.asarray(default, dtype=float)
            result = np.fromstring(text, sep=" ")
            if result.shape != (3,):
                raise ValueError(f"invalid URDF vector: {text!r}")
            return result

        def origin_matrix(node: ElementTree.Element) -> np.ndarray:
            origin = node.find("origin")
            if origin is None:
                return np.eye(4)
            xyz_value = values(origin.get("xyz"), (0.0, 0.0, 0.0))
            rpy_value = values(origin.get("rpy"), (0.0, 0.0, 0.0))
            return transform(Rotation.from_euler("xyz", rpy_value), xyz_value)

        model = cls(path)
        model.arm_joint_names = arm_names
        # Fixed joints are visual-only; only movable auxiliary joints are
        # exposed to the pendant.  New URDF releases may omit old mechanisms.
        auxiliary = tuple(
            name
            for name in AUX_JOINTS
            if name in joint_nodes and joint_nodes[name].get("type") != "fixed"
        )
        model.aux_joint_names = auxiliary
        model.auxiliary_limits = {
            name: (
                float(joint_nodes[name].find("limit").get("lower")),
                float(joint_nodes[name].find("limit").get("upper")),
            )
            for name in auxiliary
            if joint_nodes[name].find("limit") is not None
        }
        model.auxiliary_labels = {
            name: RobotModel.auxiliary_labels.get(name, name) for name in auxiliary
        }
        initial_values = [INITIAL_CONFIGURATION[f"Joint{i}_J"] for i in range(1, 8)]
        model.initial_configuration = {
            **{name: value for name, value in zip(arm_names, initial_values)},
            **{name: 0.0 for name in auxiliary},
        }
        model.arm_base_origin = origin_matrix(joint_nodes["arm_base_joint"])
        parsed: list[JointSpec] = []
        for name in arm_names:
            node = joint_nodes[name]
            axis_node = node.find("axis")
            limit_node = node.find("limit")
            if axis_node is None or limit_node is None:
                raise ValueError(f"URDF joint {name} is missing axis or limits")
            parsed.append(
                JointSpec(
                    name=name,
                    axis=values(axis_node.get("xyz"), (0.0, 0.0, 1.0)),
                    origin=origin_matrix(node),
                    lower=float(limit_node.attrib["lower"]),
                    upper=float(limit_node.attrib["upper"]),
                )
            )
        model.joints = tuple(parsed)
        model.lower = np.array([joint.lower for joint in model.joints])
        model.upper = np.array([joint.upper for joint in model.joints])
        terminal_origin = origin_matrix(joint_nodes[tcp_joint])
        model.tcp_transform = terminal_origin
        model.tcp_link_name = joint_nodes[tcp_joint].find("child").get("link")
        initial = model.arm_vector(model.initial_configuration)
        if np.any(initial < model.lower) or np.any(initial > model.upper):
            raise ValueError("initial configuration violates arm limits")
        return model

    def arm_vector(self, configuration: dict[str, float]) -> np.ndarray:
        return np.array(
            [configuration[name] for name in self.arm_joint_names], dtype=float
        )

    def aux_configuration(self, configuration: dict[str, float]) -> dict[str, float]:
        return {name: float(configuration[name]) for name in self.aux_joint_names}

    def configuration(self, arm: np.ndarray, aux: dict[str, float]) -> dict[str, float]:
        result = {
            name: float(value) for name, value in zip(self.arm_joint_names, arm)
        }
        result.update({name: float(aux[name]) for name in self.aux_joint_names})
        return result

    def link_transforms(
        self, arm: np.ndarray, aux: dict[str, float] | None = None
    ) -> list[np.ndarray]:
        arm = np.asarray(arm, dtype=float)
        if arm.shape != (7,):
            raise ValueError("arm configuration must contain 7 values")
        platform = 0.0 if aux is None else float(aux["platform_joint"])
        current = transform(translation=(0.0, 0.0, platform)) @ self.arm_base_origin
        links = [current.copy()]
        for value, joint in zip(arm, self.joints):
            current = current @ joint.origin
            current = current @ axis_rotation(joint.axis, float(value))
            links.append(current.copy())
        return links

    def flange_pose(
        self, arm: np.ndarray, aux: dict[str, float] | None = None
    ) -> np.ndarray:
        return self.link_transforms(arm, aux)[-1]

    def tcp_pose(
        self, arm: np.ndarray, aux: dict[str, float] | None = None
    ) -> np.ndarray:
        return self.flange_pose(arm, aux) @ self.tcp_transform

    def tcp_jacobian(
        self, arm: np.ndarray, aux: dict[str, float] | None = None
    ) -> np.ndarray:
        """Return a numerical world-frame 6x7 [translation, rotation] Jacobian."""
        arm = np.asarray(arm, dtype=float)
        base = self.tcp_pose(arm, aux)
        epsilon = 1e-6
        jacobian = np.zeros((6, 7))
        for index in range(7):
            shifted = arm.copy()
            shifted[index] += epsilon
            pose = self.tcp_pose(shifted, aux)
            jacobian[:3, index] = (pose[:3, 3] - base[:3, 3]) / epsilon
            delta = Rotation.from_matrix(pose[:3, :3] @ base[:3, :3].T)
            jacobian[3:, index] = delta.as_rotvec() / epsilon
        return jacobian

    @staticmethod
    def pose(position: np.ndarray, rotation: Rotation) -> np.ndarray:
        return transform(rotation, np.asarray(position, dtype=float))
