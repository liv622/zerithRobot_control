"""Resolve URDF mesh references for bundles that are not ROS workspaces."""

from __future__ import annotations

from pathlib import Path
from tempfile import NamedTemporaryFile
from xml.etree import ElementTree

from robot_logging import get_logger

_logger = get_logger("urdf.mesh")

PACKAGE_URI_PREFIX = "package://"


def resolve_package_mesh_path(uri: str, urdf_path: Path) -> Path | None:
    """Resolve a ROS ``package://`` mesh URI from nearby URDF resources.

    Exported URDF bundles commonly preserve their ROS URI even after the
    package directory is flattened, e.g. ``urdf/robot.urdf`` beside ``visual/``
    and ``collision/``.  Each URI suffix is tried below the URDF directory and
    its parents, so both layouts work without ROS installed.
    """
    parts = Path(uri.removeprefix(PACKAGE_URI_PREFIX)).parts
    bases = (urdf_path.parent, *urdf_path.parents)
    for base in bases:
        for start in range(len(parts)):
            candidate = base.joinpath(*parts[start:])
            if candidate.is_file():
                return candidate.resolve()
    return None


def missing_mesh_references(urdf_path: Path) -> list[str]:
    """Return the ``package://`` mesh URIs that cannot be resolved locally.

    Used to validate a user-selected URDF before it is loaded, so an
    unusable file is reported as a message instead of an exception traceback
    from deep inside the mesh loader.
    """
    root = ElementTree.parse(urdf_path).getroot()
    missing: list[str] = []
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename is None or not filename.startswith(PACKAGE_URI_PREFIX):
            continue
        if resolve_package_mesh_path(filename, urdf_path) is None:
            missing.append(filename)
    return missing


def load_urdf_with_local_meshes(urdf_path: Path):
    """Load a URDF after rewriting local ``package://`` mesh references.

    ``yourdfpy`` is imported lazily so headless installations without the
    visualisation dependencies can still use the rest of the URDF layer.
    """
    import yourdfpy

    root = ElementTree.parse(urdf_path).getroot()
    missing: list[str] = []
    replaced = False
    for mesh in root.findall(".//mesh"):
        filename = mesh.get("filename")
        if filename is None or not filename.startswith(PACKAGE_URI_PREFIX):
            continue
        local_path = resolve_package_mesh_path(filename, urdf_path)
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
        _logger.debug("已重写 %s 的网格路径用于加载", urdf_path.name)
        return yourdfpy.URDF.load(temporary.name)
