from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from application import RobotApplicationService
from infrastructure import JsonConfigurationRepository, JsonTeachPointRepository
from robot_framework.controller import Controller
from robot_framework.pyroki_backend import create_solver
from robots.e1pro_dual.model import DualArmUrdfModel
from urdf import is_dual_arm_urdf

ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "e1_pro_full/eu_robot_describtion2/urdf/eu_robot_describtion2.urdf"
SINGLE_URDF = ROOT / "e1_pro_full/ur10/urdf/ur10_robot.urdf"


class DualArmModelTests(unittest.TestCase):
    def test_urdf_detection(self) -> None:
        self.assertTrue(is_dual_arm_urdf(URDF))
        self.assertFalse(is_dual_arm_urdf(SINGLE_URDF))

    def test_arm_labels_and_dynamic_protocol(self) -> None:
        model = DualArmUrdfModel.from_urdf(URDF)
        self.assertEqual(model.aux_joint_names, ("j1", "j2", "j3"))
        self.assertEqual(model.active_arm, "left")
        self.assertEqual(model.arm_joint_names, ("j4", "j5", "j6", "j7", "j8", "j9"))
        self.assertEqual(model.tcp_link_name, "Empty_Link9")
        self.assertEqual(len(model.lower), 6)
        model.set_active_arm("right")
        self.assertEqual(model.arm_joint_names, ("j10", "j11", "j12", "j13", "j14", "j15"))
        self.assertEqual(model.tcp_link_name, "Empty_Link15")

    def test_initial_pose_spreads_arms_for_visibility(self) -> None:
        model = DualArmUrdfModel.from_urdf(URDF)
        aux = model.aux_configuration(model.initial_configuration)
        model.set_active_arm("left")
        left = model.tcp_pose(model.stored_arm("left"), aux)
        model.set_active_arm("right")
        right = model.tcp_pose(model.stored_arm("right"), aux)
        span = abs(left[0, 3]) + abs(right[0, 3])
        self.assertGreaterEqual(span, 0.9)
        self.assertGreater(left[2, 3], 0.2)
        self.assertGreater(right[2, 3], 0.2)

    def test_forward_kinematics_matches_yourdfpy(self) -> None:
        # Zero configuration: both arms hang straight from the mast.
        from urdf import load_urdf_with_local_meshes

        model = DualArmUrdfModel.from_urdf(URDF)
        aux = {"j1": 0.0, "j2": 0.0, "j3": 0.0}
        zero = np.zeros(6)
        left = model.tcp_pose_of("left", zero, aux)
        self.assertTrue(np.allclose(left[:3, 3], [-0.005, -0.240, -0.155], atol=1e-3))

        yourdfpy_urdf = load_urdf_with_local_meshes(URDF)
        yourdfpy_urdf.update_cfg(yourdfpy_urdf.zero_cfg)
        reference = yourdfpy_urdf.get_transform("Empty_Link9", "base_link")
        np.testing.assert_allclose(left[:3, 3], reference[:3, 3], atol=1e-6)

    def test_arms_solve_independently(self) -> None:
        model = DualArmUrdfModel.from_urdf(URDF)
        aux = {"j1": 0.0, "j2": 0.0, "j3": 0.0}
        # A reachable left-arm pose: shoulder lifted, elbow bent.  Solving for
        # the active arm must leave the inactive right arm untouched.
        desired = np.array([np.deg2rad(100), np.deg2rad(-25), 0.0, 0.0, 0.0, 0.0])
        target = model.tcp_pose_of("left", desired, aux)
        with tempfile.TemporaryDirectory() as directory:
            controller = Controller(model, Path(directory) / "last_solution.json")
            model.set_active_arm("left")
            controller.arm = model.stored_arm("left").copy()
            right_before = model.stored_arm("right").copy()
            controller.target = target
            solution = controller.solve(recovery_seeds=4)
            self.assertLessEqual(solution.position_error_m, 0.002)
            np.testing.assert_allclose(
                solution.arm, desired, atol=np.deg2rad(0.1)
            )
            # The inactive arm must not move during the active-arm solve.
            np.testing.assert_allclose(model.stored_arm("right"), right_before, atol=1e-12)

    def test_dual_arm_uses_scipy_solver(self) -> None:
        model = DualArmUrdfModel.from_urdf(URDF)
        solver = create_solver(model)
        self.assertEqual(solver.backend_name, "SciPy fallback")

    def test_full_configuration_covers_all_joints(self) -> None:
        model = DualArmUrdfModel.from_urdf(URDF)
        aux = {"j1": 0.5, "j2": 0.0, "j3": 0.0}
        config = model.full_configuration(aux)
        self.assertEqual(set(config), {"j1", "j2", "j3", "j4", "j5", "j6",
                                       "j7", "j8", "j9", "j10", "j11", "j12",
                                       "j13", "j14", "j15"})
        self.assertEqual(config["j1"], 0.5)


class DualArmApplicationTests(unittest.TestCase):
    def test_switch_preserves_arm_states(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            model = DualArmUrdfModel.from_urdf(URDF)
            controller = Controller(model, Path(directory) / "last_solution.json")
            service = RobotApplicationService(
                model,
                controller,
                JsonTeachPointRepository(Path(directory) / "tp.json"),
                JsonConfigurationRepository(Path(directory) / "profiles.json"),
            )
            controller.arm[0] += np.deg2rad(10)
            left_before = controller.arm.copy()
            service.handle_command({"action": "set_active_arm", "side": "right"})
            self.assertEqual(model.active_arm, "right")
            np.testing.assert_allclose(model.stored_arm("left"), left_before, atol=1e-12)
            right_before = controller.arm.copy()
            service.handle_command({"action": "set_active_arm", "side": "left"})
            np.testing.assert_allclose(controller.arm, left_before, atol=1e-12)
            service.handle_command({"action": "set_active_arm", "side": "right"})
            np.testing.assert_allclose(controller.arm, right_before, atol=1e-12)

    def test_switch_rejects_unknown_side(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            model = DualArmUrdfModel.from_urdf(URDF)
            controller = Controller(model, Path(directory) / "last_solution.json")
            service = RobotApplicationService(
                model,
                controller,
                JsonTeachPointRepository(Path(directory) / "tp.json"),
                JsonConfigurationRepository(Path(directory) / "profiles.json"),
            )
            with self.assertRaises(ValueError):
                service.handle_command({"action": "set_active_arm", "side": "middle"})

    def test_state_reports_active_arm(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            model = DualArmUrdfModel.from_urdf(URDF)
            controller = Controller(model, Path(directory) / "last_solution.json")
            service = RobotApplicationService(
                model,
                controller,
                JsonTeachPointRepository(Path(directory) / "tp.json"),
                JsonConfigurationRepository(Path(directory) / "profiles.json"),
            )
            state = service.read_state()
            self.assertEqual(state["robot"]["active_arm"], "left")
            self.assertEqual(state["robot"]["arm_joint_names"], ["j4", "j5", "j6", "j7", "j8", "j9"])
            self.assertEqual(len(state["arm_degrees"]), 6)
            service.handle_command({"action": "set_active_arm", "side": "right"})
            state = service.read_state()
            self.assertEqual(state["robot"]["active_arm"], "right")
            self.assertEqual(state["robot"]["arm_joint_names"], ["j10", "j11", "j12", "j13", "j14", "j15"])


if __name__ == "__main__":
    unittest.main()
