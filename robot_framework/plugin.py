"""Robot plugin contract used by model-agnostic entrypoints and interfaces."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Callable

from .model_protocol import RobotModelProtocol


ModelLoader = Callable[[Path], RobotModelProtocol]
SmokeTest = Callable[[Path], int]


@dataclass(frozen=True)
class RobotPlugin:
    """All model-specific wiring required to load one robot."""

    key: str
    display_name: str
    urdf_relative_path: Path
    load_model: ModelLoader
    run_smoke_test: SmokeTest

