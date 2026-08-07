"""Backward-compatible shim for the URDF layer.

URDF parsing and mesh resolution moved to the top-level :mod:`urdf` package so
they are no longer owned by the simulator adapter.  This module remains only so
existing imports keep working; new code should import from :mod:`urdf`.
"""

from __future__ import annotations

from urdf import (
    load_urdf_with_local_meshes,
    missing_mesh_references,
    resolve_package_mesh_path,
)

# Historical private alias retained for the existing mesh-resolution test.
_package_mesh_path = resolve_package_mesh_path

__all__ = [
    "load_urdf_with_local_meshes",
    "missing_mesh_references",
    "resolve_package_mesh_path",
]
