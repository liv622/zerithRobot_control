from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import numpy as np

from robots.generic import GenericUrdfRobotModel


URDF = """<robot name="sample">
  <link name="base"/><link name="shoulder"/><link name="flange"/><link name="tool_tcp"/>
  <joint name="shoulder_axis" type="revolute"><parent link="base"/><child link="shoulder"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="wrist_axis" type="revolute"><parent link="shoulder"/><child link="flange"/>
    <origin xyz="1 0 0" rpy="0 0 0"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
  <joint name="custom_tool_mount" type="fixed"><parent link="flange"/><child link="tool_tcp"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/></joint>
</robot>"""


class GenericUrdfTests(unittest.TestCase):
    def test_discovers_joint_chain_and_uses_terminal_link_pose(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "any_name.urdf"
            path.write_text(URDF, encoding="utf-8")
            model = GenericUrdfRobotModel.from_urdf(path)

            self.assertEqual(model.arm_joint_names, ("shoulder_axis", "wrist_axis"))
            self.assertEqual(model.tcp_link_name, "tool_tcp")
            pose = model.tcp_pose(np.array([np.pi / 2, 0.0]))
            np.testing.assert_allclose(pose[:3, 3], [1.0, 1.0, 0.2], atol=1e-12)
            np.testing.assert_allclose(model.flange_pose(np.zeros(2))[:3, 3], [2, 0, 0])


if __name__ == "__main__":
    unittest.main()
