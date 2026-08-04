from __future__ import annotations

import ast
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAYER_PACKAGES = ("application", "trajectory", "robot_framework")


def imported_modules(path: Path) -> set[str]:
    tree = ast.parse(path.read_text(encoding="utf-8"))
    modules: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            modules.add(node.module)
    return modules


class ArchitectureTests(unittest.TestCase):
    def test_root_launchers_remain_thin(self) -> None:
        for name in ("app.py", "teach_pendant.py"):
            meaningful = [
                line
                for line in (ROOT / name).read_text(encoding="utf-8").splitlines()
                if line.strip() and not line.startswith("#!")
            ]
            self.assertLessEqual(len(meaningful), 6, name)

    def test_algorithm_and_application_layers_do_not_import_adapters(self) -> None:
        forbidden = ("viser", "http", "communication", "robots.e1pro")
        paths = [
            path
            for package in LAYER_PACKAGES
            for path in (ROOT / package).glob("*.py")
        ]
        for path in paths:
            modules = imported_modules(path)
            for module in modules:
                self.assertFalse(
                    module.startswith(forbidden),
                    f"{path} imports adapter {module}",
                )

    def test_e1pro_is_isolated_as_a_robot_plugin(self) -> None:
        from robot_framework import RobotModelProtocol, RobotPlugin
        from robots.e1pro import E1PRO_PLUGIN, RobotModel
        from robots.registry import get_robot_plugin

        self.assertTrue(hasattr(RobotModel, "arm_joint_names"))
        self.assertTrue(hasattr(RobotModelProtocol, "tcp_pose"))
        self.assertIsInstance(E1PRO_PLUGIN, RobotPlugin)
        self.assertIs(get_robot_plugin("e1pro"), E1PRO_PLUGIN)

    def test_generic_interface_does_not_import_e1pro(self) -> None:
        path = ROOT / "interfaces" / "simulator" / "viser_app.py"
        self.assertFalse(
            any(
                module.startswith("robots.e1pro")
                for module in imported_modules(path)
            )
        )

    def test_viser_adapter_has_no_custom_gui_controls(self) -> None:
        source = (ROOT / "interfaces/simulator/viser_app.py").read_text(
            encoding="utf-8"
        )
        self.assertNotIn("server.gui", source)
        self.assertNotIn("add_transform_controls", source)
        self.assertIn("Viser 仅用于三维实时显示", source)


if __name__ == "__main__":
    unittest.main()
