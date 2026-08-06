"""Serial-arm model discovered directly from a URDF kinematic tree."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from scipy.spatial.transform import Rotation


def _transform(xyz: np.ndarray = np.zeros(3), rpy: np.ndarray = np.zeros(3)) -> np.ndarray:
    value = np.eye(4)
    value[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    value[:3, 3] = xyz
    return value


def _vector(value: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if value is None:
        return np.asarray(default, dtype=float)
    parsed = np.fromstring(value, sep=" ")
    if parsed.shape != (3,):
        raise ValueError(f"URDF 三维向量无效：{value!r}")
    return parsed


def _origin(node: ElementTree.Element) -> np.ndarray:
    origin = node.find("origin")
    if origin is None:
        return np.eye(4)
    return _transform(
        _vector(origin.get("xyz"), (0.0, 0.0, 0.0)),
        _vector(origin.get("rpy"), (0.0, 0.0, 0.0)),
    )


@dataclass(frozen=True)
class _Joint:
    name: str
    kind: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float

    @property
    def movable(self) -> bool:
        return self.kind in {"revolute", "continuous", "prismatic"}


class GenericUrdfRobotModel:
    """A movable root-to-leaf chain selected from any valid serial-arm URDF.

    The terminal link is the deepest leaf with the most movable joints.  Names
    containing tcp/tool/end/ee are preferred when equally capable.  Therefore
    no vendor-specific joint, flange, or TCP name is required.
    """

    auxiliary_limits: dict[str, tuple[float, float]] = {}
    auxiliary_labels: dict[str, str] = {}
    aux_joint_names: tuple[str, ...] = ()

    def __init__(self, path: Path) -> None:
        self.urdf_path = path
        root = ElementTree.parse(path).getroot()
        if root.tag != "robot":
            raise ValueError("URDF 根节点必须是 <robot>")
        links = {node.get("name") for node in root.findall("link") if node.get("name")}
        raw: list[tuple[_Joint, str, str]] = []
        child_joint: dict[str, tuple[_Joint, str]] = {}
        parent_links: set[str] = set()
        child_links: set[str] = set()
        for node in root.findall("joint"):
            parent, child = node.find("parent"), node.find("child")
            name = node.get("name")
            if not name or parent is None or child is None or not parent.get("link") or not child.get("link"):
                raise ValueError("URDF joint 缺少 name、parent 或 child")
            kind = node.get("type", "fixed")
            axis_node, limit_node = node.find("axis"), node.find("limit")
            axis = _vector(axis_node.get("xyz") if axis_node is not None else None, (0.0, 0.0, 1.0))
            if kind == "continuous":
                lower, upper = -2.0 * np.pi, 2.0 * np.pi
            elif kind in {"revolute", "prismatic"}:
                if limit_node is None or limit_node.get("lower") is None or limit_node.get("upper") is None:
                    raise ValueError(f"可动关节 {name} 缺少 lower/upper 限位")
                lower, upper = float(limit_node.get("lower")), float(limit_node.get("upper"))
            else:
                lower = upper = 0.0
            joint = _Joint(name, kind, _origin(node), axis, lower, upper)
            parent_name, child_name = parent.get("link"), child.get("link")
            if child_name in child_joint:
                raise ValueError(f"URDF 不是树形结构：{child_name} 有多个父关节")
            raw.append((joint, parent_name, child_name))
            child_joint[child_name] = (joint, parent_name)
            parent_links.add(parent_name)
            child_links.add(child_name)
        leaves = links - parent_links
        if not leaves:
            raise ValueError("URDF 未找到末端 link")

        def chain_to(link: str) -> list[_Joint]:
            chain: list[_Joint] = []
            while link in child_joint:
                joint, link = child_joint[link]
                chain.append(joint)
            return list(reversed(chain))

        def score(link: str) -> tuple[int, int, int, str]:
            chain = chain_to(link)
            movable = sum(item.movable for item in chain)
            hint = int(any(word in link.lower() for word in ("tcp", "tool", "end", "ee")))
            return movable, hint, len(chain), link

        terminal = max(leaves, key=score)
        self._chain = tuple(chain_to(terminal))
        self.arm_joint_names = tuple(joint.name for joint in self._chain if joint.movable)
        if not self.arm_joint_names:
            raise ValueError("URDF 的末端链不包含可动关节")
        self.tcp_link_name = terminal
        movable = [joint for joint in self._chain if joint.movable]
        self.lower = np.asarray([joint.lower for joint in movable], dtype=float)
        self.upper = np.asarray([joint.upper for joint in movable], dtype=float)
        self.initial_configuration = {
            name: float(np.clip(0.0, lower, upper))
            for name, lower, upper in zip(self.arm_joint_names, self.lower, self.upper)
        }
        first = next(index for index, joint in enumerate(self._chain) if joint.movable)
        self.arm_base_origin = self._fixed_transform(self._chain[:first])
        last = max(index for index, joint in enumerate(self._chain) if joint.movable)
        self.tcp_transform = self._fixed_transform(self._chain[last + 1 :])

    @classmethod
    def from_urdf(cls, path: Path) -> "GenericUrdfRobotModel":
        return cls(path)

    @staticmethod
    def _fixed_transform(chain: tuple[_Joint, ...] | list[_Joint]) -> np.ndarray:
        result = np.eye(4)
        for joint in chain:
            result = result @ joint.origin
        return result

    def arm_vector(self, configuration: dict[str, float]) -> np.ndarray:
        return np.asarray([configuration[name] for name in self.arm_joint_names], dtype=float)

    def aux_configuration(self, configuration: dict[str, float]) -> dict[str, float]:
        return {}

    def configuration(self, arm: np.ndarray, aux: dict[str, float]) -> dict[str, float]:
        return {name: float(value) for name, value in zip(self.arm_joint_names, arm)}

    def _pose(self, arm: np.ndarray) -> np.ndarray:
        arm = np.asarray(arm, dtype=float)
        if arm.shape != (len(self.arm_joint_names),):
            raise ValueError(f"机械臂关节值必须包含 {len(self.arm_joint_names)} 项")
        values = iter(arm)
        result = np.eye(4)
        for joint in self._chain:
            result = result @ joint.origin
            if joint.kind in {"revolute", "continuous"}:
                rotation = np.eye(4)
                rotation[:3, :3] = Rotation.from_rotvec(joint.axis * float(next(values))).as_matrix()
                result = result @ rotation
            elif joint.kind == "prismatic":
                translation = np.eye(4)
                translation[:3, 3] = joint.axis * float(next(values))
                result = result @ translation
        return result

    def flange_pose(self, arm: np.ndarray, aux: dict[str, float] | None = None) -> np.ndarray:
        # The final movable-joint frame is retained for user TCP offsets.
        values = np.asarray(arm, dtype=float)
        result = np.eye(4)
        index = 0
        for joint in self._chain:
            result = result @ joint.origin
            if joint.movable:
                value = values[index]
                index += 1
                motion = np.eye(4)
                if joint.kind in {"revolute", "continuous"}:
                    motion[:3, :3] = Rotation.from_rotvec(joint.axis * value).as_matrix()
                else:
                    motion[:3, 3] = joint.axis * value
                result = result @ motion
            if index == len(self.arm_joint_names):
                return result
        return result

    def tcp_pose(self, arm: np.ndarray, aux: dict[str, float] | None = None) -> np.ndarray:
        return self._pose(arm)

    @staticmethod
    def pose(position: np.ndarray, rotation: Rotation) -> np.ndarray:
        value = np.eye(4)
        value[:3, :3] = rotation.as_matrix()
        value[:3, 3] = position
        return value
