from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from infrastructure import (
    JsonTeachPointProfileRepository,
    JsonTeachPointRepository,
)
from trajectory.joint import plan_joint_trajectory
from trajectory.models import TrajectoryError


class TeachPointTests(unittest.TestCase):
    def test_movl_movj_points_persist_and_can_be_modified(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "teach_points.json"
            store = JsonTeachPointRepository(path)
            joints = [1, 2, 3, 4, 5, 6, 7]
            cartesian = [0.1, 0.2, 0.3, 10, 20, 30]
            movl = store.add("MOVL", joints, cartesian)
            movj = store.add("MOVJ", joints, cartesian, "取料点", 45)
            self.assertEqual(movl.name, "P001")
            self.assertEqual(movj.name, "取料点")
            self.assertEqual(movj.speed_percent, 45)

            store.set_checked(movl.point_id, False)
            store.update(
                movj.point_id,
                "放料点",
                "MOVL",
                [7, 6, 5, 4, 3, 2, 1],
                cartesian,
            )
            reloaded = JsonTeachPointRepository(path)
            self.assertFalse(reloaded.get(movl.point_id).checked)
            self.assertEqual(reloaded.get(movj.point_id).name, "放料点")
            self.assertEqual(reloaded.get(movj.point_id).motion_type, "MOVL")
            self.assertEqual(
                json.loads(path.read_text(encoding="utf-8"))["version"], 3
            )

            reloaded.delete(movl.point_id)
            self.assertEqual(len(reloaded.points), 1)

    def test_named_teach_point_profile_restores_a_snapshot(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            store = JsonTeachPointRepository(root / "teach_points.json")
            store.add("MOVJ", [0] * 7, [0] * 6, "初始", 20)
            profiles = JsonTeachPointProfileRepository(
                root / "teach_point_profiles.json"
            )
            profiles.save("工位A", store.as_json())
            store.update(1, "已修改", "MOVJ", [1] * 7, [0] * 6, 80)
            store.replace(profiles.get("工位A"))
            self.assertEqual(store.get(1).name, "初始")
            self.assertEqual(store.get(1).speed_percent, 20)

    def test_movj_uses_smooth_joint_interpolation_and_limits(self) -> None:
        start = np.zeros(7)
        end = np.linspace(0.1, 0.7, 7)
        trajectory = plan_joint_trajectory(
            start,
            end,
            np.full(7, -1.0),
            np.full(7, 1.0),
            duration_s=1.0,
            frequency_hz=10.0,
        )
        np.testing.assert_allclose(trajectory.arms[0], start)
        np.testing.assert_allclose(trajectory.arms[-1], end)
        np.testing.assert_allclose(trajectory.times[[0, -1]], [0.0, 1.0])
        self.assertEqual(len(trajectory.arms), 11)

        with self.assertRaises(TrajectoryError):
            plan_joint_trajectory(
                start,
                np.full(7, 2.0),
                np.full(7, -1.0),
                np.full(7, 1.0),
                duration_s=1.0,
                frequency_hz=10.0,
            )


if __name__ == "__main__":
    unittest.main()
