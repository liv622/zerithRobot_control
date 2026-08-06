from __future__ import annotations

import unittest
from pathlib import Path

from interfaces.simulator.urdf_loader import _package_mesh_path


class UrdfMeshResolutionTests(unittest.TestCase):
    def test_resolves_flattened_ros_package_mesh_layout(self) -> None:
        root = Path(__file__).resolve().parents[1]
        urdf = root / "e1_pro_full/ur10/urdf/ur10_robot.urdf"
        resolved = _package_mesh_path(
            "package://ur_description/meshes/ur10/visual/Wrist3.dae", urdf
        )
        self.assertEqual(
            resolved,
            (root / "e1_pro_full/ur10/visual/Wrist3.dae").resolve(),
        )


if __name__ == "__main__":
    unittest.main()
