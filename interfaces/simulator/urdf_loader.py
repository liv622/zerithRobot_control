"""URDF loading helpers for resource folders that are not ROS workspaces."""

from __future__ import annotations

from pathlib import Path
from tempfile import NamedTemporaryFile
from xml.etree import ElementTree


def _package_mesh_path(uri: str, urdf_path: Path) -> Path | None:
    """Resolve a ROS ``package://`` mesh URI from nearby URDF resources.

    Exported URDF bundles commonly preserve their ROS URI even after the
    package directory is flattened, e.g. ``urdf/robot.urdf`` beside
    ``visual/`` and ``collision/``.  Each URI suffix is tried below the URDF
    directory and its parents, so both layouts work without ROS installed.
    """
    parts = Path(uri.removeprefix("package://")).parts
    bases = (urdf_path.parent, *urdf_path.parents)
    for base in bases:
        for start in range(len(parts)):
            candidate = base.joinpath(*parts[start:])
            if candidate.is_file():
                return candidate.resolve()
    return None


def load_urdf_with_local_meshes(urdf_path: Path):
    """Load an URDF after resolving local ``package://`` mesh references."""
    import yourdfpy

    root = ElementTree.parse(urdf_path).getroot()
    missing: list[str] = []
    replaced = False
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename is None or not filename.startswith("package://"):
            continue
        local_path = _package_mesh_path(filename, urdf_path)
        if local_path is None:
            missing.append(filename)
            continue
        mesh.set("filename", str(local_path))
        replaced = True
    if missing:
        details = "\n".join(f"- {value}" for value in missing)
        raise FileNotFoundError(
            f"无法从 {urdf_path.parent} 附近解析以下 URDF 网格文件：\n{details}"
        )
    if not replaced:
        return yourdfpy.URDF.load(str(urdf_path))
    # yourdfpy loads mesh data eagerly.  The temporary XML only supplies the
    # rewritten, absolute mesh filenames and is removed immediately after.
    with NamedTemporaryFile(suffix=".urdf", mode="wb") as temporary:
        ElementTree.ElementTree(root).write(
            temporary,
            encoding="utf-8",
            xml_declaration=True,
        )
        temporary.flush()
        return yourdfpy.URDF.load(temporary.name)
