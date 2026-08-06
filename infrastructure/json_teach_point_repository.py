"""JSON-backed implementation of the teach-point repository."""

from __future__ import annotations

import json
from dataclasses import asdict
from pathlib import Path

from domain import TeachPoint


class JsonTeachPointRepository:
    def __init__(self, path: Path, joint_count: int = 7) -> None:
        self.path = path
        self.joint_count = joint_count
        self.points: list[TeachPoint] = []
        self._next_id = 1
        self.load()

    def load(self) -> None:
        if not self.path.exists():
            return
        payload = json.loads(self.path.read_text(encoding="utf-8"))
        self._replace_from_raw(payload.get("points", []))

    def _replace_from_raw(self, raw_points: object) -> None:
        if not isinstance(raw_points, list):
            raise ValueError("teach_points.json 中的 points 必须是数组")
        loaded: list[TeachPoint] = []
        for item in raw_points:
            motion_type = str(item["motion_type"]).upper()
            legacy_values = list(item.get("values", []))
            joints = item.get("joint_values")
            cartesian = item.get("cartesian_values")
            if joints is None or cartesian is None:
                # Keep legacy points executable in their original mode. New
                # points always persist both representations.
                joints = legacy_values if motion_type == "MOVJ" else [0.0] * self.joint_count
                cartesian = legacy_values if motion_type == "MOVL" else [0.0] * 6
            point = TeachPoint(
                point_id=int(item["point_id"]),
                name=str(item["name"]),
                motion_type=motion_type,
                joint_values=list(joints),
                cartesian_values=list(cartesian),
                speed_percent=float(item.get("speed_percent", 30.0)),
                checked=bool(item.get("checked", True)),
            )
            point.validate(self.joint_count)
            loaded.append(point)
        self.points = loaded
        self._next_id = max((point.point_id for point in loaded), default=0) + 1

    def replace(self, points: list[dict]) -> None:
        self._replace_from_raw(points)
        self.save()

    def save(self) -> None:
        payload = {"version": 3, "points": [asdict(point) for point in self.points]}
        temporary = self.path.with_suffix(".tmp")
        temporary.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.path)

    def add(
        self,
        motion_type: str,
        joint_values: list[float],
        cartesian_values: list[float],
        name: str = "",
        speed_percent: float = 30.0,
    ) -> TeachPoint:
        point = TeachPoint(
            point_id=self._next_id,
            name=name,
            motion_type=motion_type.upper(),
            joint_values=joint_values,
            cartesian_values=cartesian_values,
            speed_percent=speed_percent,
        )
        point.validate(self.joint_count)
        self._next_id += 1
        self.points.append(point)
        self.save()
        return point

    def get(self, point_id: int) -> TeachPoint:
        for point in self.points:
            if point.point_id == point_id:
                return point
        raise ValueError(f"示教点 P{point_id:03d} 不存在")

    def update(
        self,
        point_id: int,
        name: str,
        motion_type: str,
        joint_values: list[float],
        cartesian_values: list[float],
        speed_percent: float = 30.0,
    ) -> TeachPoint:
        point = self.get(point_id)
        updated = TeachPoint(
            point_id=point.point_id,
            name=name,
            motion_type=motion_type.upper(),
            joint_values=joint_values,
            cartesian_values=cartesian_values,
            speed_percent=speed_percent,
            checked=point.checked,
        )
        updated.validate(self.joint_count)
        self.points[self.points.index(point)] = updated
        self.save()
        return updated

    def set_checked(self, point_id: int, checked: bool) -> None:
        self.get(point_id).checked = checked
        self.save()

    def delete(self, point_id: int) -> None:
        point = self.get(point_id)
        self.points.remove(point)
        self.save()

    def as_json(self) -> list[dict]:
        return [
            {
                **asdict(point),
                "values": point.values,
            }
            for point in self.points
        ]
