"""URDF layer: parsing, mesh resolution and operator-facing discovery.

Everything that reads a URDF file lives here.  Robot plugins build kinematic
models on :func:`parse_urdf_chain`, the simulator adapter renders meshes through
:func:`load_urdf_with_local_meshes`, and the application layer offers URDF
selection through :class:`UrdfCatalog`.

This layer depends only on the standard library, NumPy/SciPy and the logging
layer, so it never pulls UI or transport code into a headless install.
"""

from .catalog import (
    MAX_SCAN_DEPTH,
    MAX_SCAN_RESULTS,
    URDF_SUFFIXES,
    UrdfAccessError,
    UrdfCatalog,
    UrdfEntry,
)
from .kinematics import (
    MOVABLE_JOINT_TYPES,
    DualArmChains,
    UrdfChain,
    UrdfJoint,
    detect_dual_arm_chains,
    fixed_chain_transform,
    is_dual_arm_urdf,
    parse_urdf_chain,
    transform_from_translation_and_rpy,
)
from .mesh_resolver import (
    load_urdf_with_local_meshes,
    missing_mesh_references,
    resolve_package_mesh_path,
)

__all__ = [
    "MAX_SCAN_DEPTH",
    "MAX_SCAN_RESULTS",
    "MOVABLE_JOINT_TYPES",
    "URDF_SUFFIXES",
    "DualArmChains",
    "UrdfAccessError",
    "UrdfCatalog",
    "UrdfChain",
    "UrdfEntry",
    "UrdfJoint",
    "detect_dual_arm_chains",
    "fixed_chain_transform",
    "is_dual_arm_urdf",
    "load_urdf_with_local_meshes",
    "missing_mesh_references",
    "parse_urdf_chain",
    "resolve_package_mesh_path",
    "transform_from_translation_and_rpy",
]
