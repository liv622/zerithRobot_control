"""Discover, validate and safely resolve URDF files chosen by an operator.

The teach pendant lets an operator load a URDF from any folder.  That makes this
module a security boundary: the requested path arrives over HTTP, so it is
resolved against an explicit allow-list of search roots and checked for symlink
escapes before anything reads it.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

from robot_logging import get_logger
from .kinematics import parse_urdf_chain
from .mesh_resolver import missing_mesh_references

_logger = get_logger("urdf.catalog")

URDF_SUFFIXES = (".urdf", ".URDF")
# Bounds on directory scanning.  A scan is triggered by an HTTP request, so it
# must terminate promptly regardless of how large the chosen folder is.
MAX_SCAN_DEPTH = 6
MAX_SCAN_RESULTS = 400
MAX_URDF_BYTES = 64 * 1024 * 1024


class UrdfAccessError(ValueError):
    """Raised when a requested URDF path is outside the permitted roots."""


@dataclass(frozen=True)
class UrdfEntry:
    """One discovered URDF file and the result of validating it."""

    path: Path
    display_name: str
    robot_name: str = ""
    joint_count: int = 0
    terminal_link: str = ""
    valid: bool = False
    detail: str = ""
    missing_meshes: tuple[str, ...] = ()

    def as_json(self) -> dict:
        return {
            "path": str(self.path),
            "display_name": self.display_name,
            "robot_name": self.robot_name,
            "joint_count": self.joint_count,
            "terminal_link": self.terminal_link,
            "valid": self.valid,
            "detail": self.detail,
            "missing_meshes": list(self.missing_meshes),
        }


@dataclass
class UrdfCatalog:
    """An allow-list of directories the operator may load URDFs from."""

    search_roots: list[Path] = field(default_factory=list)

    def normalised_roots(self) -> list[Path]:
        roots: list[Path] = []
        for root in self.search_roots:
            try:
                resolved = Path(root).expanduser().resolve()
            except OSError:
                continue
            if resolved.is_dir() and resolved not in roots:
                roots.append(resolved)
        return roots

    def add_root(self, directory: Path | str) -> Path:
        """Permit ``directory`` as a URDF source and return its resolved path."""
        candidate = Path(directory).expanduser()
        try:
            resolved = candidate.resolve(strict=True)
        except (OSError, RuntimeError) as exc:
            raise UrdfAccessError(f"URDF 目录不可访问：{candidate}") from exc
        if not resolved.is_dir():
            raise UrdfAccessError(f"不是目录：{resolved}")
        if resolved not in self.normalised_roots():
            self.search_roots.append(resolved)
        _logger.info("已添加 URDF 搜索目录：%s", resolved)
        return resolved

    def remove_root(self, directory: Path | str) -> None:
        target = Path(directory).expanduser()
        try:
            resolved = target.resolve()
        except OSError:
            resolved = target
        self.search_roots = [
            root
            for root in self.search_roots
            if Path(root).expanduser() != target
            and self._safe_resolve(root) != resolved
        ]

    @staticmethod
    def _safe_resolve(value: Path | str) -> Path | None:
        try:
            return Path(value).expanduser().resolve()
        except (OSError, RuntimeError):
            return None

    def resolve_urdf_path(self, requested: Path | str) -> Path:
        """Resolve an operator-supplied URDF path inside the permitted roots.

        ``Path.resolve`` follows symlinks before the containment test, so a
        symlink inside a permitted directory that points outside it is rejected
        rather than followed.  Rejecting is the right default here because the
        path originates from a network request.
        """
        candidate = Path(requested).expanduser()
        try:
            resolved = candidate.resolve(strict=True)
        except (OSError, RuntimeError) as exc:
            raise UrdfAccessError(f"URDF 不存在或不可访问：{candidate}") from exc
        if not resolved.is_file():
            raise UrdfAccessError(f"URDF 不是文件：{resolved}")
        if resolved.suffix not in URDF_SUFFIXES:
            raise UrdfAccessError("只允许加载 .urdf 文件")
        try:
            size = resolved.stat().st_size
        except OSError as exc:
            raise UrdfAccessError(f"无法读取 URDF：{resolved}") from exc
        if size > MAX_URDF_BYTES:
            raise UrdfAccessError("URDF 文件过大，拒绝加载")
        roots = self.normalised_roots()
        if not roots:
            raise UrdfAccessError("尚未配置任何 URDF 搜索目录")
        if not any(resolved == root or root in resolved.parents for root in roots):
            allowed = "、".join(str(root) for root in roots)
            raise UrdfAccessError(
                f"URDF 必须位于已授权目录内（当前授权：{allowed}）"
            )
        return resolved

    def discover(self, *, validate: bool = True) -> list[UrdfEntry]:
        """List every URDF below the permitted roots, newest first."""
        found: dict[Path, UrdfEntry] = {}
        for root in self.normalised_roots():
            for path in self._scan(root):
                if path in found:
                    continue
                found[path] = (
                    self.describe(path)
                    if validate
                    else UrdfEntry(
                        path=path,
                        display_name=self._display_name(path, root),
                        valid=True,
                    )
                )
        entries = list(found.values())
        entries.sort(key=lambda entry: entry.display_name.lower())
        return entries

    def _scan(self, root: Path) -> list[Path]:
        results: list[Path] = []
        # An explicit breadth-first walk with a depth cap, rather than rglob, so
        # a deep or symlink-looped tree cannot stall the request thread.
        frontier = [(root, 0)]
        visited: set[Path] = set()
        while frontier and len(results) < MAX_SCAN_RESULTS:
            directory, depth = frontier.pop(0)
            resolved = self._safe_resolve(directory)
            if resolved is None or resolved in visited:
                continue
            visited.add(resolved)
            try:
                children = sorted(directory.iterdir())
            except (OSError, PermissionError):
                continue
            for child in children:
                if len(results) >= MAX_SCAN_RESULTS:
                    break
                try:
                    if child.is_file() and child.suffix in URDF_SUFFIXES:
                        results.append(child.resolve())
                    elif child.is_dir() and depth < MAX_SCAN_DEPTH:
                        frontier.append((child, depth + 1))
                except OSError:
                    continue
        if len(results) >= MAX_SCAN_RESULTS:
            _logger.warning(
                "URDF 扫描结果达到上限 %d, %s 下可能还有未列出的文件",
                MAX_SCAN_RESULTS,
                root,
            )
        return results

    @staticmethod
    def _display_name(path: Path, root: Path) -> str:
        try:
            relative = path.relative_to(root)
        except ValueError:
            return path.name
        # Robot resource bundles normally use ``<robot-name>/urdf/*.urdf``.
        # Present the bundle name first so operators select "ur10" or
        # "marvin6" rather than having to infer it from an export filename.
        parts = relative.parts
        if "urdf" in parts:
            urdf_index = parts.index("urdf")
            if urdf_index > 0:
                return f"{parts[urdf_index - 1]} / {path.stem}"
        return str(relative)

    def describe(self, path: Path) -> UrdfEntry:
        """Validate one URDF and summarise it for the pendant.

        Parse failures are reported as an invalid entry rather than raised: the
        operator should see *why* a file cannot be used next to the file name,
        and one bad URDF must not hide every other file in the folder.
        """
        resolved = self._safe_resolve(path) or Path(path)
        roots = self.normalised_roots()
        display = resolved.name
        for root in roots:
            if root in resolved.parents:
                display = self._display_name(resolved, root)
                break
        try:
            chain = parse_urdf_chain(resolved)
        except (OSError, ValueError, SyntaxError) as exc:
            return UrdfEntry(
                path=resolved,
                display_name=display,
                valid=False,
                detail=f"{type(exc).__name__}: {exc}",
            )
        try:
            missing = tuple(missing_mesh_references(resolved))
        except (OSError, ValueError, SyntaxError):
            missing = ()
        joint_count = len(chain.joint_names)
        detail = f"{joint_count} 轴 · 末端 {chain.terminal_link}"
        if missing:
            detail += f" · 缺少 {len(missing)} 个网格文件"
        return UrdfEntry(
            path=resolved,
            display_name=display,
            robot_name=chain.robot_name,
            joint_count=joint_count,
            terminal_link=chain.terminal_link,
            # Missing visual meshes do not prevent kinematic use, so the entry
            # stays selectable and the gap is reported in the detail text.
            valid=True,
            detail=detail,
            missing_meshes=missing,
        )
