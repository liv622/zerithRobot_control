"""URDF 目录授权与选择结果的 JSON 持久化实现。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from robot_logging import get_logger

_logger = get_logger("infrastructure.urdf_preference")

# 单个偏好文件的大小上限。该文件只保存若干条目录路径，
# 超过此大小说明文件已被破坏或被替换，直接拒绝解析。
MAX_PREFERENCE_BYTES = 256 * 1024


class JsonUrdfPreferenceRepository:
    """保存操作员授权过的 URDF 目录以及当前选择的 URDF 路径。

    读取失败一律降级为空偏好而不是抛出异常：偏好只是使用便利，
    文件损坏不应导致示教器无法启动。
    """

    def __init__(self, path: Path) -> None:
        self.path = Path(path)

    def load(self) -> dict[str, Any]:
        """读取偏好；文件不存在或内容非法时返回空字典。"""
        if not self.path.exists():
            return {}
        try:
            if self.path.stat().st_size > MAX_PREFERENCE_BYTES:
                _logger.warning("URDF 偏好文件过大，已忽略：%s", self.path)
                return {}
            value = json.loads(self.path.read_text(encoding="utf-8"))
        except (OSError, ValueError) as exc:
            _logger.warning("URDF 偏好文件解析失败，已忽略：%s", exc)
            return {}
        if not isinstance(value, dict):
            return {}
        # 逐字段校验类型，避免把非法结构直接交给上层使用。
        roots = value.get("search_roots", [])
        if not isinstance(roots, list):
            roots = []
        active = value.get("active_path", "")
        return {
            "search_roots": [str(item) for item in roots if isinstance(item, str)],
            "active_path": active if isinstance(active, str) else "",
        }

    def save(self, value: dict[str, Any]) -> None:
        """原子写入偏好文件，避免读取方看到写了一半的内容。"""
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.path.with_suffix(".tmp")
        try:
            temporary.write_text(
                json.dumps(value, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            temporary.replace(self.path)
        except OSError as exc:
            _logger.warning("URDF 偏好文件写入失败：%s", exc)
            temporary.unlink(missing_ok=True)
