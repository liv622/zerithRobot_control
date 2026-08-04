"""Named snapshots of complete teach-point tables."""

from __future__ import annotations

import json
from pathlib import Path


class JsonTeachPointProfileRepository:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._profiles: dict[str, list[dict]] = {}
        if path.exists():
            payload = json.loads(path.read_text(encoding="utf-8"))
            raw = payload.get("profiles", {})
            if isinstance(raw, dict):
                self._profiles = {
                    str(name): list(points)
                    for name, points in raw.items()
                    if isinstance(points, list)
                }

    def _write(self) -> None:
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps({"version": 1, "profiles": self._profiles}, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.path)

    def names(self) -> list[str]:
        return sorted(self._profiles)

    def get(self, name: str) -> list[dict]:
        if name not in self._profiles:
            raise ValueError(f"示教点位配置 {name!r} 不存在")
        return json.loads(json.dumps(self._profiles[name]))

    def save(self, name: str, points: list[dict]) -> None:
        clean = name.strip()
        if not clean:
            raise ValueError("示教点位配置名称不能为空")
        self._profiles[clean] = json.loads(json.dumps(points))
        self._write()

    def delete(self, name: str) -> None:
        if name not in self._profiles:
            raise ValueError(f"示教点位配置 {name!r} 不存在")
        del self._profiles[name]
        self._write()
