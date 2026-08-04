from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from application.frames import CoordinateFrameService
from infrastructure import JsonCoordinateFrameRepository


class CoordinateFrameTests(unittest.TestCase):
    def test_selected_base_and_tcp_round_trip_and_persist(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            repository = JsonCoordinateFrameRepository(
                Path(directory) / "coordinate_frames.json"
            )
            arm_base = np.eye(4)
            arm_base[:3, 3] = [0.5, 0.0, 0.3]
            frames = CoordinateFrameService(
                np.eye(4), repository, lambda: arm_base
            )
            frames.create_base("工装", [1, 2, 3, 0, 0, 90])
            frames.create_tcp("吸盘", [0, 0, 0.1, 0, 0, 0])
            frames.select("arm_base_link", "吸盘")
            desired = [0.2, 0.0, 0.0, 0, 0, 0]
            default_pose = frames.default_from_display(desired)
            np.testing.assert_allclose(frames.pose_values(default_pose), desired)

            restored = CoordinateFrameService(np.eye(4), repository)
            self.assertEqual(restored.active_base, "arm_base_link")
            self.assertEqual(restored.active_tcp, "吸盘")


if __name__ == "__main__":
    unittest.main()
