"""JSON-backed robot installation/configuration profiles."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any


class JsonConfigurationRepository:
    def __init__(self, path: Path) -> None:
        self.path = path
        self._profiles: dict[str, dict[str, Any]] = {}
        self._load()

    def _load(self) -> None:
        if not self.path.exists():
            return
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        profiles = payload.get("profiles", {})
        if not isinstance(profiles, dict):
            raise ValueError("robot_profiles.json 中的 profiles 必须是对象")
        self._profiles = {
            str(name): dict(values)
            for name, values in profiles.items()
            if isinstance(values, dict)
        }

    def _write(self) -> None:
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(
                {"version": 1, "profiles": self._profiles},
                ensure_ascii=False,
                indent=2,
            )
            + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.path)

    def names(self) -> list[str]:
        return sorted(self._profiles)

    def get(self, name: str) -> dict[str, Any]:
        try:
            return dict(self._profiles[name])
        except KeyError as exc:
            raise ValueError(f"配置文件 {name!r} 不存在") from exc

    def save(self, name: str, values: dict[str, Any]) -> None:
        clean_name = name.strip()
        if not clean_name:
            raise ValueError("配置文件名称不能为空")
        self._profiles[clean_name] = dict(values)
        self._write()

    def delete(self, name: str) -> None:
        if name not in self._profiles:
            raise ValueError(f"配置文件 {name!r} 不存在")
        del self._profiles[name]
        self._write()
