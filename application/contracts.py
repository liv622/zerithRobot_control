"""Application settings and presentation output-port contracts."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

from robot_framework.solver import IKSolution


def _noop() -> None:
    return


def _noop_solution(_: IKSolution) -> None:
    return


def _noop_bool(_: bool) -> None:
    return


def _noop_status(_: str) -> None:
    return


@dataclass
class ApplicationEvents:
    scene_changed: Callable[[], None] = _noop
    target_changed: Callable[[], None] = _noop
    solution_changed: Callable[[IKSolution], None] = _noop_solution
    guide_changed: Callable[[], None] = _noop
    auxiliary_changed: Callable[[], None] = _noop
    settings_changed: Callable[[], None] = _noop
    drag_visibility_changed: Callable[[bool], None] = _noop_bool
    status_changed: Callable[[str], None] = _noop_status


@dataclass
class ApplicationSettings:
    live_solve: bool = True
    orientation_lock: bool = True
    auto_recovery: bool = True
    recovery_count: int = 10
    guide_enabled: bool = False
    guide_strength: float = 0.05
    point_duration_s: float = 5.0
    trajectory_frequency_hz: float = 30.0
    loop_teach_program: bool = False
    drag_unlocked: bool = False
    speed_percent: float = 30.0
    max_linear_speed_mm_s: float = 250.0
    max_angular_speed_deg_s: float = 60.0
    max_joint_speed_deg_s: float = 60.0
    command_delay_s: float = 0.0
