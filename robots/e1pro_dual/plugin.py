"""Dual-arm robot plugin registration."""

from robot_framework.plugin import RobotPlugin

from .model import DualArmUrdfModel
from .smoke import URDF_RELATIVE_PATH, resolve_urdf_path, run_smoke_test


DUAL_ARM_PLUGIN = RobotPlugin(
    key="e1pro_dual",
    display_name="E1-PRO 双臂",
    urdf_relative_path=URDF_RELATIVE_PATH,
    load_model=DualArmUrdfModel.from_urdf,
    run_smoke_test=run_smoke_test,
    urdf_path_resolver=resolve_urdf_path,
)
