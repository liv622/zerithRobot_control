"""E1-PRO plugin registration."""

from robot_framework.plugin import RobotPlugin

from .model import RobotModel
from .smoke import URDF_RELATIVE_PATH, resolve_urdf_path, run_smoke_test


E1PRO_PLUGIN = RobotPlugin(
    key="e1pro",
    display_name="E1-PRO",
    urdf_relative_path=URDF_RELATIVE_PATH,
    load_model=RobotModel.from_urdf,
    run_smoke_test=run_smoke_test,
    urdf_path_resolver=resolve_urdf_path,
)
