from __future__ import annotations

import json
import tempfile
import unittest
from pathlib import Path

import numpy as np

from robot_framework.controller import Controller
from robots.e1pro.model import (
    RobotModel,
)
from robots.e1pro.smoke import resolve_urdf_path


ROOT = Path(__file__).resolve().parents[1]
URDF = resolve_urdf_path(ROOT)


class CoreTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.model = RobotModel.from_urdf(URDF)

    def test_tcp_is_fixed_extension_from_link7(self) -> None:
        arm = self.model.arm_vector(self.model.initial_configuration)
        aux = self.model.aux_configuration(self.model.initial_configuration)
        flange = self.model.flange_pose(arm, aux)
        tcp = self.model.tcp_pose(arm, aux)
        local_offset = flange[:3, :3].T @ (tcp[:3, 3] - flange[:3, 3])
        np.testing.assert_allclose(
            local_offset, self.model.tcp_transform[:3, 3], atol=1e-12
        )

    def test_ik_preserves_auxiliary_mechanisms_and_writes_result(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "last_solution.json"
            controller = Controller(self.model, output)
            before = controller.aux.copy()
            controller.target[:3, 3] += [0.01, -0.01, 0.005]
            solution = controller.solve(recovery_seeds=4)
            self.assertLess(solution.position_error_m, 0.002)
            self.assertLess(solution.orientation_error_rad, np.deg2rad(1.0))
            self.assertEqual(before, controller.aux)
            saved = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(
                set(saved["arm_joints_rad"]), set(self.model.arm_joint_names)
            )
            self.assertEqual(
                set(saved["auxiliary_joints"]), set(self.model.aux_joint_names)
            )


if __name__ == "__main__":
    unittest.main()
