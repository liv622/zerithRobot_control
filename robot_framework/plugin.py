"""Robot plugin contract used by model-agnostic entrypoints and interfaces."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .model_protocol import RobotModelProtocol


ModelLoader = Callable[[Path], RobotModelProtocol]
SmokeTest = Callable[[Path], int]
UrdfResolver = Callable[[Path], Path]


@dataclass(frozen=True)
class RobotPlugin:
    """All model-specific wiring required to load one robot."""

    key: str
    display_name: str
    urdf_relative_path: Path
    load_model: ModelLoader
    run_smoke_test: SmokeTest
    urdf_path_resolver: UrdfResolver | None = None

    def resolve_urdf_path(self, project_root: Path) -> Path:
        """Resolve the active URDF when the application starts."""
        if self.urdf_path_resolver is not None:
            return self.urdf_path_resolver(project_root)
        return project_root / self.urdf_relative_path
