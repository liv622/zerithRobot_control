from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from application import ApplicationEvents, RobotApplicationService
from infrastructure import JsonTeachPointRepository
from infrastructure import JsonConfigurationRepository
from robot_framework.controller import Controller
from robots.e1pro.model import RobotModel


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "e1_pro_full/urdf/E1-PRO_EVT2.0_V9_260714.urdf"


class ApplicationServiceTests(unittest.TestCase):
    def test_commands_are_independent_from_ui_and_transport(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            model = RobotModel.from_urdf(URDF)
            controller = Controller(
                model,
                Path(directory) / "last_solution.json",
            )
            service = RobotApplicationService(
                model,
                controller,
                JsonTeachPointRepository(
                    Path(directory) / "teach_points.json"
                ),
                JsonConfigurationRepository(
                    Path(directory) / "robot_profiles.json"
                ),
            )
            event_counts = {"target": 0, "settings": 0}
            service.events = ApplicationEvents(
                target_changed=lambda: event_counts.__setitem__(
                    "target",
                    event_counts["target"] + 1,
                ),
                settings_changed=lambda: event_counts.__setitem__(
                    "settings",
                    event_counts["settings"] + 1,
                ),
            )

            service.handle_command(
                {
                    "action": "settings",
                    "live": False,
                    "orientation_lock": True,
                    "auto_recovery": True,
                    "recovery_count": 8,
                    "guide_enabled": False,
                    "guide_strength": 0.05,
                }
            )
            xyz, rpy = controller.target_xyz_rpy()
            target = np.r_[xyz + [0.001, 0.0, 0.0], rpy]
            service.handle_command(
                {"action": "set_target", "values": target.tolist()}
            )
            service.handle_command(
                {
                    "action": "save_teach_point",
                    "motion_type": "MOVJ",
                    "name": "测试点",
                }
            )

            state = service.read_state()
            self.assertFalse(state["settings"]["live"])
            self.assertEqual(
                state["teach_program"]["points"][0]["motion_type"],
                "MOVJ",
            )
            self.assertEqual(
                len(state["teach_program"]["points"][0]["values"]),
                7,
            )
            self.assertEqual(
                len(state["teach_program"]["points"][0]["joint_values"]),
                7,
            )
            self.assertEqual(
                len(state["teach_program"]["points"][0]["cartesian_values"]),
                6,
            )
            self.assertGreaterEqual(event_counts["target"], 1)
            self.assertGreaterEqual(event_counts["settings"], 1)

            service.handle_command(
                {
                    "action": "motion_settings",
                    "speed_percent": 42,
                    "max_linear_speed_mm_s": 300,
                    "max_angular_speed_deg_s": 80,
                    "max_joint_speed_deg_s": 70,
                    "command_delay_s": 0.5,
                }
            )
            service.handle_command(
                {"action": "save_configuration", "name": "测试配置"}
            )
            service.handle_command(
                {
                    "action": "motion_settings",
                    "speed_percent": 10,
                    "max_linear_speed_mm_s": 100,
                    "max_angular_speed_deg_s": 50,
                    "max_joint_speed_deg_s": 40,
                    "command_delay_s": 0,
                }
            )
            service.handle_command(
                {"action": "load_configuration", "name": "测试配置"}
            )
            state = service.read_state()
            self.assertEqual(state["settings"]["speed_percent"], 42)
            self.assertEqual(state["configuration"]["active"], "测试配置")
            self.assertEqual(
                state["configuration"]["profiles"],
                ["测试配置"],
            )

            tcp_before = model.tcp_pose(
                controller.arm,
                controller.aux,
            )
            arm_before = controller.arm.copy()
            service.handle_command(
                {"action": "switch_arm_shape", "direction": 1}
            )
            tcp_after = model.tcp_pose(
                controller.arm,
                controller.aux,
            )
            self.assertGreater(
                float(np.linalg.norm(controller.arm - arm_before)),
                0.08,
            )
            self.assertLess(
                float(np.linalg.norm(tcp_after[:3, 3] - tcp_before[:3, 3])),
                0.002,
            )
            service.close()

    def test_null_space_motion_changes_joints_and_locks_tcp(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            model = RobotModel.from_urdf(URDF)
            controller = Controller(
                model,
                Path(directory) / "last_solution.json",
            )
            service = RobotApplicationService(
                model,
                controller,
                JsonTeachPointRepository(
                    Path(directory) / "teach_points.json"
                ),
            )
            arm_before = controller.arm.copy()
            tcp_before = model.tcp_pose(
                controller.arm,
                controller.aux,
            )
            service.null_space.begin()
            solution = service.null_space.step(10.0)
            service.null_space.end()
            tcp_after = model.tcp_pose(
                controller.arm,
                controller.aux,
            )

            self.assertGreater(
                float(np.linalg.norm(controller.arm - arm_before)),
                np.deg2rad(1.0),
            )
            self.assertGreaterEqual(
                int(
                    np.count_nonzero(
                        np.abs(controller.arm - arm_before) > 1e-5
                    )
                ),
                4,
            )
            self.assertLess(solution.position_error_m, 0.0005)
            self.assertLess(
                solution.orientation_error_rad,
                np.deg2rad(0.1),
            )
            self.assertLess(
                float(
                    np.linalg.norm(
                        tcp_after[:3, 3] - tcp_before[:3, 3]
                    )
                ),
                0.0005,
            )
            service.close()


if __name__ == "__main__":
    unittest.main()
