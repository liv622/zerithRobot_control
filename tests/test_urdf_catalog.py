from __future__ import annotations

import unittest
from pathlib import Path

from urdf import UrdfCatalog


class UrdfCatalogTests(unittest.TestCase):
    def test_current_robot_folders_appear_as_named_choices(self) -> None:
        root = Path(__file__).resolve().parents[1] / "e1_pro_full"
        entries = UrdfCatalog([root]).discover()
        labels = {entry.display_name for entry in entries}

        self.assertIn("marvin6 / E1-PRO_EVT2.0_V12_260730", labels)
        self.assertIn("ur10 / ur10_robot", labels)


if __name__ == "__main__":
    unittest.main()
