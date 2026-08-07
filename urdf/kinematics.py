"""Parse a URDF into the joint chain the kinematics layers consume.

This is deliberately independent of any particular robot model: it reads the
XML, validates that the tree is well formed, and exposes the movable chain from
the root to the most capable leaf.  Robot plugins build their model on top of
this instead of re-parsing XML.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

import numpy as np
from scipy.spatial.transform import Rotation

MOVABLE_JOINT_TYPES = frozenset({"revolute", "continuous", "prismatic"})
# Leaf-link name fragments that suggest a tool frame, used only to break ties
# between leaves that are otherwise equally deep and equally actuated.
TCP_NAME_HINTS = ("tcp", "tool", "end", "ee")


def transform_from_translation_and_rpy(
    xyz: np.ndarray | None = None,
    rpy: np.ndarray | None = None,
) -> np.ndarray:
    """Build a 4x4 transform from a URDF ``origin`` pair."""
    value = np.eye(4)
    if rpy is not None:
        value[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    if xyz is not None:
        value[:3, 3] = xyz
    return value


def _parse_vector(value: str | None, default: tuple[float, float, float]) -> np.ndarray:
    if value is None:
        return np.asarray(default, dtype=float)
    parsed = np.fromstring(value, sep=" ")
    if parsed.shape != (3,):
        raise ValueError(f"URDF 三维向量无效：{value!r}")
    if not np.all(np.isfinite(parsed)):
        raise ValueError(f"URDF 三维向量包含非有限数值：{value!r}")
    return parsed


def _parse_origin(node: ElementTree.Element) -> np.ndarray:
    origin = node.find("origin")
    if origin is None:
        return np.eye(4)
    return transform_from_translation_and_rpy(
        _parse_vector(origin.get("xyz"), (0.0, 0.0, 0.0)),
        _parse_vector(origin.get("rpy"), (0.0, 0.0, 0.0)),
    )


@dataclass(frozen=True)
class UrdfJoint:
    """One URDF joint with the fields the kinematics layers need."""

    name: str
    joint_type: str
    origin: np.ndarray
    axis: np.ndarray
    lower: float
    upper: float
    velocity_limit: float = 0.0
    effort_limit: float = 0.0

    @property
    def movable(self) -> bool:
        return self.joint_type in MOVABLE_JOINT_TYPES


@dataclass(frozen=True)
class UrdfChain:
    """The movable root-to-leaf chain selected from a serial-arm URDF."""

    urdf_path: Path
    robot_name: str
    joints: tuple[UrdfJoint, ...]
    terminal_link: str
    link_names: tuple[str, ...]

    @property
    def movable_joints(self) -> tuple[UrdfJoint, ...]:
        return tuple(joint for joint in self.joints if joint.movable)

    @property
    def joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.movable_joints)

    @property
    def lower_limits(self) -> np.ndarray:
        return np.asarray(
            [joint.lower for joint in self.movable_joints], dtype=float
        )

    @property
    def upper_limits(self) -> np.ndarray:
        return np.asarray(
            [joint.upper for joint in self.movable_joints], dtype=float
        )

    def velocity_limits(self, fallback_rad_s: float) -> np.ndarray:
        """Per-joint velocity ceilings, substituting ``fallback_rad_s`` when a
        URDF omits or zeroes the ``<limit velocity=...>`` attribute.

        Honouring the URDF's own velocity limits matters for real-robot work:
        it means the double-S planner is bounded by the manufacturer's numbers
        rather than by one global speed the operator happened to type.
        """
        values = np.asarray(
            [joint.velocity_limit for joint in self.movable_joints], dtype=float
        )
        replacement = float(fallback_rad_s)
        values[~np.isfinite(values) | (values <= 0.0)] = replacement
        return values

    def base_transform(self) -> np.ndarray:
        """Fixed transform from the URDF root to the first movable joint."""
        first = next(
            (index for index, joint in enumerate(self.joints) if joint.movable),
            len(self.joints),
        )
        return fixed_chain_transform(self.joints[:first])

    def tool_transform(self) -> np.ndarray:
        """Fixed transform from the last movable joint to the terminal link."""
        movable_indices = [
            index for index, joint in enumerate(self.joints) if joint.movable
        ]
        if not movable_indices:
            return fixed_chain_transform(self.joints)
        return fixed_chain_transform(self.joints[movable_indices[-1] + 1 :])


def fixed_chain_transform(joints: tuple[UrdfJoint, ...] | list[UrdfJoint]) -> np.ndarray:
    """Accumulate the origins of a run of joints into one transform."""
    result = np.eye(4)
    for joint in joints:
        result = result @ joint.origin
    return result


def _parse_joint(node: ElementTree.Element) -> tuple[UrdfJoint, str, str]:
    parent, child = node.find("parent"), node.find("child")
    name = node.get("name")
    if (
        not name
        or parent is None
        or child is None
        or not parent.get("link")
        or not child.get("link")
    ):
        raise ValueError("URDF joint 缺少 name、parent 或 child")
    joint_type = node.get("type", "fixed")
    axis_node, limit_node = node.find("axis"), node.find("limit")
    axis = _parse_vector(
        axis_node.get("xyz") if axis_node is not None else None,
        (0.0, 0.0, 1.0),
    )
    norm = float(np.linalg.norm(axis))
    if joint_type in MOVABLE_JOINT_TYPES:
        if norm <= 1e-12:
            raise ValueError(f"可动关节 {name} 的轴向量为零")
        axis = axis / norm
    velocity_limit = 0.0
    effort_limit = 0.0
    if joint_type == "continuous":
        lower, upper = -2.0 * np.pi, 2.0 * np.pi
    elif joint_type in MOVABLE_JOINT_TYPES:
        if (
            limit_node is None
            or limit_node.get("lower") is None
            or limit_node.get("upper") is None
        ):
            raise ValueError(f"可动关节 {name} 缺少 lower/upper 限位")
        lower, upper = float(limit_node.get("lower")), float(limit_node.get("upper"))
        if not np.isfinite(lower) or not np.isfinite(upper) or lower > upper:
            raise ValueError(f"可动关节 {name} 的限位无效")
    else:
        lower = upper = 0.0
    if limit_node is not None:
        velocity_limit = float(limit_node.get("velocity") or 0.0)
        effort_limit = float(limit_node.get("effort") or 0.0)
    return (
        UrdfJoint(
            name=name,
            joint_type=joint_type,
            origin=_parse_origin(node),
            axis=axis,
            lower=lower,
            upper=upper,
            velocity_limit=velocity_limit,
            effort_limit=effort_limit,
        ),
        parent.get("link"),
        child.get("link"),
    )


def parse_urdf_chain(urdf_path: Path) -> UrdfChain:
    """Read ``urdf_path`` and return its most capable movable chain.

    The terminal link is the deepest leaf with the most movable joints, with
    tool-like names preferred only as a tie-break.  No vendor-specific joint,
    flange or TCP name is required, so any valid serial-arm URDF loads.
    """
    path = Path(urdf_path)
    root = ElementTree.parse(path).getroot()
    if root.tag != "robot":
        raise ValueError("URDF 根节点必须是 <robot>")
    link_names = tuple(
        node.get("name") for node in root.findall("link") if node.get("name")
    )
    links = set(link_names)
    parent_of: dict[str, tuple[UrdfJoint, str]] = {}
    parent_links: set[str] = set()
    for node in root.findall("joint"):
        joint, parent_name, child_name = _parse_joint(node)
        if child_name in parent_of:
            raise ValueError(f"URDF 不是树形结构：{child_name} 有多个父关节")
        parent_of[child_name] = (joint, parent_name)
        parent_links.add(parent_name)
    leaves = links - parent_links
    if not leaves:
        raise ValueError("URDF 未找到末端 link")

    def chain_to(link: str) -> list[UrdfJoint]:
        chain: list[UrdfJoint] = []
        visited: set[str] = set()
        current = link
        while current in parent_of:
            if current in visited:
                raise ValueError(f"URDF 关节链存在环：{current}")
            visited.add(current)
            joint, current = parent_of[current]
            chain.append(joint)
        return list(reversed(chain))

    def rank(link: str) -> tuple[int, int, int, str]:
        chain = chain_to(link)
        movable = sum(joint.movable for joint in chain)
        hint = int(any(word in link.lower() for word in TCP_NAME_HINTS))
        return movable, hint, len(chain), link

    terminal = max(leaves, key=rank)
    joints = tuple(chain_to(terminal))
    if not any(joint.movable for joint in joints):
        raise ValueError("URDF 的末端链不包含可动关节")
    return UrdfChain(
        urdf_path=path,
        robot_name=root.get("name") or path.stem,
        joints=joints,
        terminal_link=terminal,
        link_names=link_names,
    )
