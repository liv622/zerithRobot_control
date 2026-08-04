"""Ports required by application use cases."""

from __future__ import annotations

from typing import Any, Protocol

from domain import TeachPoint


class TeachPointRepository(Protocol):
    points: list[TeachPoint]

    def add(
        self,
        motion_type: str,
        joint_values: list[float],
        cartesian_values: list[float],
        name: str = "",
    ) -> TeachPoint: ...

    def get(self, point_id: int) -> TeachPoint: ...

    def update(
        self,
        point_id: int,
        name: str,
        motion_type: str,
        joint_values: list[float],
        cartesian_values: list[float],
    ) -> TeachPoint: ...

    def set_checked(self, point_id: int, checked: bool) -> None: ...

    def delete(self, point_id: int) -> None: ...

    def as_json(self) -> list[dict]: ...


class ConfigurationRepository(Protocol):
    def names(self) -> list[str]: ...

    def get(self, name: str) -> dict[str, Any]: ...

    def save(self, name: str, values: dict[str, Any]) -> None: ...

    def delete(self, name: str) -> None: ...
