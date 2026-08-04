"""Generic robot state orchestration and application facade."""

from __future__ import annotations

import threading
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation

from domain import TeachPoint
from robot_framework.controller import Controller
from robot_framework.model_protocol import RobotModelProtocol
from robot_framework.solver import IKSolution
from .command_dispatcher import CommandDispatcher
from .configuration import ConfigurationService
from .continuous_jog import ContinuousJogService
from .contracts import ApplicationEvents, ApplicationSettings
from .hardware import NullRobotHardware
from .ports import ConfigurationRepository, RobotHardware, TeachPointRepository
from .null_space_motion import NullSpaceMotionService
from .teach_program import TeachProgramService


class RobotApplicationService:
    """Facade used by UIs and communication adapters."""

    def __init__(
        self,
        model: RobotModelProtocol,
        controller: Controller,
        teach_points: TeachPointRepository,
        configurations: ConfigurationRepository | None = None,
        settings: ApplicationSettings | None = None,
        events: ApplicationEvents | None = None,
        hardware: RobotHardware | None = None,
    ) -> None:
        self.model = model
        self.controller = controller
        self.teach_points = teach_points
        self.settings = settings or ApplicationSettings()
        self.configurations = ConfigurationService(
            configurations,
            self.settings,
        )
        self._events = events or ApplicationEvents()
        self.hardware = hardware or NullRobotHardware()
        self._solve_lock = threading.Lock()
        self._command_lock = threading.RLock()
        self.teach_program = TeachProgramService(
            model,
            controller,
            teach_points,
            self.settings,
            self._events,
        )
        self.null_space = NullSpaceMotionService(
            model,
            controller,
            self.settings,
            self._events,
        )
        self.continuous_jog = ContinuousJogService(
            model,
            controller,
            self._events,
            self.solve,
            lambda: self.teach_program.running,
            self.teach_program.set_status,
            lambda: self.teach_program.status,
            self.settings,
            self.null_space,
        )
        self.teach_program.motion_blocked = lambda: (
            self.continuous_jog.running or self._solve_lock.locked()
        )
        self.command_dispatcher = CommandDispatcher(self)

    @property
    def AUX_LIMITS(self) -> dict[str, tuple[float, float]]:
        return self.model.auxiliary_limits

    @property
    def AUX_LABELS(self) -> dict[str, str]:
        return self.model.auxiliary_labels

    @property
    def events(self) -> ApplicationEvents:
        return self._events

    @events.setter
    def events(self, value: ApplicationEvents) -> None:
        self._events = value
        self.teach_program.set_events(value)
        self.continuous_jog.set_events(value)
        self.null_space.set_events(value)

    @property
    def command_lock(self) -> threading.RLock:
        return self._command_lock

    @property
    def program_status(self) -> str:
        return self.teach_program.status

    def solve(
        self,
        *,
        force: bool = False,
        lock_orientation_override: bool | None = None,
        emit_motion: bool = False,
    ) -> IKSolution | None:
        if not self._solve_lock.acquire(blocking=False):
            return None
        try:
            solution = self.controller.solve(
                lock_orientation=(
                    self.settings.orientation_lock
                    if lock_orientation_override is None
                    else lock_orientation_override
                ),
                guide_enabled=self.settings.guide_enabled,
                guide_strength=self.settings.guide_strength,
                recovery_seeds=(
                    self.settings.recovery_count
                    if self.settings.auto_recovery or force
                    else 0
                ),
                force_recovery=force,
                multi_start=self.settings.auto_recovery or force,
            )
            self.events.scene_changed()
            self.events.solution_changed(solution)
            if emit_motion:
                self.events.motion_sample(self.controller.arm.copy())
            return solution
        finally:
            self._solve_lock.release()

    def set_target_values(
        self,
        values: np.ndarray,
        *,
        solve_live: bool = True,
    ) -> None:
        values = np.asarray(values, dtype=float)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise ValueError("TCP 目标必须包含 6 个有限数值")
        self.controller.set_target_xyz_rpy(values[:3], values[3:])
        self.events.target_changed()
        if solve_live and self.settings.live_solve:
            self.solve()

    def set_drag_pose(
        self,
        position: np.ndarray,
        wxyz: tuple[float, float, float, float],
    ) -> None:
        self.controller.target = self.model.pose(
            np.asarray(position, dtype=float),
            Rotation.from_quat([wxyz[1], wxyz[2], wxyz[3], wxyz[0]]),
        )
        self.events.target_changed()
        if self.settings.live_solve:
            self.solve()

    def set_drag_unlocked(self, unlocked: bool) -> None:
        self.settings.drag_unlocked = unlocked
        if unlocked:
            self.target_current()
        self.events.drag_visibility_changed(unlocked)
        self.events.settings_changed()

    def target_current(self) -> None:
        self.controller.target = self.model.tcp_pose(
            self.controller.arm,
            self.controller.aux,
        )
        self.events.target_changed()

    def reset(self) -> None:
        self.controller.reset()
        self.events.target_changed()
        self.events.guide_changed()
        self.events.auxiliary_changed()
        self.events.scene_changed()
        if self.controller.solution is not None:
            self.events.solution_changed(self.controller.solution)

    def set_auxiliary(
        self,
        name: str,
        value: float,
        *,
        solve_live: bool = True,
    ) -> None:
        if name not in self.model.aux_joint_names:
            raise ValueError("未知其他机构关节")
        lower, upper = self.AUX_LIMITS[name]
        self.controller.aux[name] = float(np.clip(value, lower, upper))
        self.events.auxiliary_changed()
        self.events.scene_changed()
        if solve_live and self.settings.live_solve:
            self.solve()

    def jog_auxiliary(self, name: str, delta: float) -> None:
        if not np.isfinite(delta):
            raise ValueError("机构点动增量无效")
        self.set_auxiliary(
            name,
            self.controller.aux.get(name, 0.0) + delta,
            solve_live=False,
        )

    def jog_joint(self, name: str, delta_degrees: float) -> None:
        if name not in self.model.arm_joint_names:
            raise ValueError("未知机械臂关节")
        index = self.model.arm_joint_names.index(name)
        delta = np.deg2rad(float(delta_degrees))
        if not np.isfinite(delta):
            raise ValueError("关节点动增量无效")
        self.controller.arm[index] = np.clip(
            self.controller.arm[index] + delta,
            self.model.lower[index],
            self.model.upper[index],
        )
        self.controller.guide = self.controller.arm.copy()
        self.controller.target = self.model.tcp_pose(
            self.controller.arm,
            self.controller.aux,
        )
        self.controller.solution = None
        self.events.guide_changed()
        self.events.target_changed()
        self.events.scene_changed()
        self.events.motion_sample(self.controller.arm.copy())

    def update_guide(
        self,
        values: np.ndarray,
        *,
        enabled: bool | None = None,
        strength: float | None = None,
    ) -> None:
        values = np.asarray(values, dtype=float)
        if values.shape != (len(self.model.arm_joint_names),):
            raise ValueError("臂形参考数量与机器人关节数不一致")
        self.controller.guide = values
        if enabled is not None:
            self.settings.guide_enabled = enabled
        if strength is not None:
            self.settings.guide_strength = strength
        if self.settings.live_solve and self.settings.guide_enabled:
            self.solve()

    def guide_current(self) -> None:
        self.controller.guide = self.controller.arm.copy()
        self.events.guide_changed()

    def switch_arm_shape(self, direction: int) -> float:
        if direction not in {-1, 1}:
            raise ValueError("臂型切换方向必须为 -1 或 1")
        if self.teach_program.running or self.continuous_jog.running:
            raise ValueError("机器人运动期间不能切换臂型")
        current = self.controller.arm.copy()
        target = self.model.tcp_pose(current, self.controller.aux)
        span = self.model.upper - self.model.lower
        candidates: list[IKSolution] = []
        for index in range(len(current)):
            guide = current.copy()
            guide[index] = np.clip(
                guide[index] + direction * span[index] * 0.35,
                self.model.lower[index],
                self.model.upper[index],
            )
            solution = self.controller.solver.solve(
                target,
                guide,
                self.controller.aux,
                lock_orientation=True,
                guide=guide,
                guide_strength=0.18,
                multi_start=False,
            )
            if (
                solution.position_error_m <= 0.002
                and solution.orientation_error_rad <= np.deg2rad(1.0)
            ):
                candidates.append(solution)
        if not candidates:
            raise ValueError("当前 TCP 未找到其他可用臂型")
        solution = max(
            candidates,
            key=lambda item: float(np.linalg.norm(item.arm - current)),
        )
        difference = float(np.linalg.norm(solution.arm - current))
        if difference < 0.08:
            raise ValueError("当前 TCP 未找到明显不同的臂型")
        self.controller.arm = solution.arm.copy()
        self.controller.guide = solution.arm.copy()
        self.controller.target = target
        self.controller.solution = solution
        self.controller.save()
        self.events.guide_changed()
        self.events.target_changed()
        self.events.solution_changed(solution)
        self.events.scene_changed()
        return difference

    def connect_hardware(self, ip: str) -> None:
        self.hardware.connect(ip)

    def disconnect_hardware(self) -> None:
        self.hardware.disconnect()

    def enable_hardware(self) -> None:
        frequency = self.settings.trajectory_frequency_hz
        if not 50.0 <= frequency <= 1000.0:
            raise ValueError("PD 前馈要求插补频率在 50 到 1000 Hz")
        self.hardware.enable(max(1, min(20, int(round(1000.0 / frequency)))))

    def disable_hardware(self) -> None:
        self.hardware.disable()

    def release_hardware_brake(self) -> None:
        self.hardware.release_brake()

    def apply_hardware_brake(self) -> None:
        self.hardware.apply_brake()

    def update_motion_settings(self, values: dict[str, Any]) -> None:
        validated = self.configurations.validate_motion_values(values)
        self.update_settings(**validated)

    def save_configuration(self, name: str) -> None:
        self.configurations.save(name, self.controller.guide)

    def load_configuration(self, name: str) -> None:
        guide = self.configurations.load(
            name,
            len(self.model.arm_joint_names),
        )
        if guide is not None:
            self.controller.guide = np.clip(
                guide,
                self.model.lower,
                self.model.upper,
            )
            self.events.guide_changed()
        self.events.settings_changed()

    def delete_configuration(self, name: str) -> None:
        self.configurations.delete(name)

    def save_current_teach_point(
        self,
        motion_type: str,
        name: str = "",
    ) -> TeachPoint:
        return self.teach_program.save_current(motion_type, name)

    def run_teach_points(self) -> None:
        self.teach_program.start()

    def stop_teach_points(self) -> None:
        self.teach_program.stop()

    def update_settings(self, **values: Any) -> None:
        for name, value in values.items():
            if not hasattr(self.settings, name):
                raise ValueError(f"未知设置：{name}")
            setattr(self.settings, name, value)
        self.events.settings_changed()

    @staticmethod
    def require_values(command: dict, count: int) -> np.ndarray:
        values = np.asarray(command["values"], dtype=float)
        if values.shape != (count,) or not np.all(np.isfinite(values)):
            raise ValueError(f"values 必须包含 {count} 个有限数值")
        return values

    def read_state(self) -> dict:
        xyz, rpy = self.controller.target_xyz_rpy()
        solution = self.controller.solution
        return {
            "backend": self.controller.solver.backend_name,
            "robot": {
                "arm_joint_names": list(self.model.arm_joint_names),
                "aux_joint_names": list(self.model.aux_joint_names),
                "auxiliary_labels": self.model.auxiliary_labels,
            },
            "target": {
                "position_m": [float(value) for value in xyz],
                "rpy_degrees": [float(value) for value in rpy],
            },
            "arm_degrees": [
                float(value) for value in np.rad2deg(self.controller.arm)
            ],
            "hardware": self.hardware.state(),
            "auxiliary": {
                name: float(self.controller.aux[name])
                for name in self.model.aux_joint_names
            },
            "reachable": None if solution is None else bool(solution.reachable),
            "position_error_mm": (
                None
                if solution is None
                else float(solution.position_error_m * 1000.0)
            ),
            "orientation_error_degrees": (
                None
                if solution is None
                else float(np.rad2deg(solution.orientation_error_rad))
            ),
            "attempts": None if solution is None else int(solution.attempts),
            "drag_unlocked": self.settings.drag_unlocked,
            "continuous_jog_running": self.continuous_jog.running,
            "null_space_active": self.null_space.active,
            "settings": {
                "live": self.settings.live_solve,
                "orientation_lock": self.settings.orientation_lock,
                "auto_recovery": self.settings.auto_recovery,
                "recovery_count": self.settings.recovery_count,
                "guide_enabled": self.settings.guide_enabled,
                "guide_strength": self.settings.guide_strength,
                "speed_percent": self.settings.speed_percent,
                "max_linear_speed_mm_s": (
                    self.settings.max_linear_speed_mm_s
                ),
                "max_angular_speed_deg_s": (
                    self.settings.max_angular_speed_deg_s
                ),
                "max_joint_speed_deg_s": (
                    self.settings.max_joint_speed_deg_s
                ),
                "command_delay_s": self.settings.command_delay_s,
            },
            "configuration": self.configurations.state(),
            "program_status": self.teach_program.status,
            "teach_program": {
                "duration": self.settings.point_duration_s,
                "frequency": self.settings.trajectory_frequency_hz,
                "loop": self.settings.loop_teach_program,
                "running": self.teach_program.running,
                "points": self.teach_points.as_json(),
            },
        }

    def handle_command(self, command: dict) -> dict:
        return self.command_dispatcher.dispatch(command)

    def close(self) -> None:
        self.continuous_jog.stop()
        self.teach_program.stop()
        self.hardware.disconnect()
