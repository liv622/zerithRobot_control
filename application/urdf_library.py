"""Operator-facing URDF selection use case.

The pendant needs three things: browse the URDFs it is allowed to load, add a
new folder to that allow-list, and select one to load.  Selection is recorded
here and reported to the pendant; the simulator process performs the actual
reload, because swapping a kinematic model underneath a running motion is not
something an HTTP request may do directly.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any, Callable

from robot_logging import get_logger
from urdf import UrdfAccessError, UrdfCatalog, UrdfEntry
from .ports import UrdfPreferenceRepository

_logger = get_logger("application.urdf_library")

MAX_SEARCH_ROOTS = 24


class UrdfLibraryService:
    """Browse and select URDF files from operator-authorised directories."""

    def __init__(
        self,
        catalog: UrdfCatalog,
        *,
        preferences: UrdfPreferenceRepository | None = None,
        active_path: Path | None = None,
        motion_blocked: Callable[[], bool] = lambda: False,
        on_selected: Callable[[Path], None] | None = None,
    ) -> None:
        self.catalog = catalog
        self.preferences = preferences
        self.active_path = Path(active_path) if active_path else None
        self.motion_blocked = motion_blocked
        self.on_selected = on_selected
        self.pending_path: Path | None = None
        self.status = ""
        self._entries: list[UrdfEntry] = []
        self._restore_preferences()

    def _restore_preferences(self) -> None:
        if self.preferences is None:
            return
        try:
            stored = self.preferences.load()
        except (OSError, ValueError) as exc:
            _logger.warning("URDF 目录偏好读取失败：%s", exc)
            return
        for value in stored.get("search_roots", []):
            try:
                self.catalog.add_root(value)
            except UrdfAccessError:
                # A folder that has since been removed or unmounted is skipped
                # rather than blocking startup.
                _logger.info("忽略不可用的历史 URDF 目录：%s", value)

    def _persist_preferences(self) -> None:
        if self.preferences is None:
            return
        try:
            self.preferences.save(
                {
                    "search_roots": [
                        str(root) for root in self.catalog.normalised_roots()
                    ],
                    "active_path": (
                        str(self.active_path) if self.active_path else ""
                    ),
                }
            )
        except (OSError, ValueError) as exc:
            _logger.warning("URDF 目录偏好保存失败：%s", exc)

    def add_search_root(self, directory: str) -> Path:
        """Authorise a directory as a URDF source."""
        text = str(directory).strip()
        if not text:
            raise ValueError("URDF 目录不能为空")
        if len(self.catalog.normalised_roots()) >= MAX_SEARCH_ROOTS:
            raise ValueError(f"URDF 搜索目录数量不能超过 {MAX_SEARCH_ROOTS} 个")
        resolved = self.catalog.add_root(text)
        self._entries = []
        self._persist_preferences()
        self.status = f"已添加 URDF 目录：{resolved}"
        return resolved

    def remove_search_root(self, directory: str) -> None:
        self.catalog.remove_root(str(directory))
        self._entries = []
        self._persist_preferences()
        self.status = f"已移除 URDF 目录：{directory}"

    def refresh(self) -> list[UrdfEntry]:
        """Re-scan the authorised directories and validate what is found."""
        self._entries = self.catalog.discover()
        return self._entries

    def entries(self) -> list[UrdfEntry]:
        if not self._entries:
            self.refresh()
        return self._entries

    def select(self, path: str) -> Path:
        """Choose a URDF to load, refusing while the robot is moving.

        Loading a different kinematic model mid-motion would leave the running
        trajectory referring to joints that no longer exist, so the request is
        rejected rather than queued.
        """
        if self.motion_blocked():
            raise ValueError("机器人运动期间不能切换 URDF")
        resolved = self.catalog.resolve_urdf_path(path)
        entry = self.catalog.describe(resolved)
        if not entry.valid:
            raise ValueError(f"URDF 无法加载：{entry.detail}")
        self.pending_path = resolved
        self.active_path = resolved
        self._persist_preferences()
        self.status = (
            f"已选择 {entry.display_name}（{entry.joint_count} 轴）；"
            "重启仿真后生效"
        )
        _logger.info("已选择 URDF：%s", resolved)
        if self.on_selected is not None:
            self.on_selected(resolved)
        return resolved

    def state(self) -> dict[str, Any]:
        entries = self.entries()
        return {
            "active_path": str(self.active_path) if self.active_path else "",
            "pending_path": str(self.pending_path) if self.pending_path else "",
            "status": self.status,
            "search_roots": [
                str(root) for root in self.catalog.normalised_roots()
            ],
            "entries": [entry.as_json() for entry in entries],
        }
