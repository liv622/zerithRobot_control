"""Generic robot state orchestration and application facade."""

from __future__ import annotations

import threading
from typing import Any, Protocol

import numpy as np
from scipy.spatial.transform import Rotation

from domain import TeachPoint
from robot_framework.controller import Controller
from robot_framework.model_protocol import RobotModelProtocol
from robot_framework.solver import IKSolution
from urdf import UrdfCatalog
from .command_dispatcher import CommandDispatcher
from .configuration import ConfigurationService
from .continuous_jog import ContinuousJogService
from .contracts import ApplicationEvents, ApplicationSettings
from .hardware import NullRobotHardware
from .frames import CoordinateFrameService
from .ports import (
    ConfigurationRepository,
    CoordinateFrameRepository,
    RobotHardware,
    TeachPointProfileRepository,
    TeachPointRepository,
    UrdfPreferenceRepository,
)
from .null_space_motion import NullSpaceMotionService
from .teach_program import TeachProgramService
from .urdf_library import UrdfLibraryService


class _OscilloscopeSource(Protocol):
    """Anything that can report statistics and be closed."""

    def statistics(self) -> dict[str, Any]: ...
    def close(self) -> None: ...


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
        frame_repository: CoordinateFrameRepository | None = None,
        teach_point_profiles: TeachPointProfileRepository | None = None,
        urdf_preferences: UrdfPreferenceRepository | None = None,
        urdf_search_roots: list | None = None,
        active_urdf_path=None,
        oscilloscope: _OscilloscopeSource | None = None,
    ) -> None:
        self.model = model
        self.controller = controller
        self.teach_points = teach_points
        self.teach_point_profiles = teach_point_profiles
        self.active_teach_point_profile = ""
        self.settings = settings or ApplicationSettings()
        self.configurations = ConfigurationService(
            configurations,
            self.settings,
        )
        self._events = events or ApplicationEvents()
        self.hardware = hardware or NullRobotHardware()
        def arm_base_transform() -> np.ndarray:
            platform = float(controller.aux.get("platform_joint", 0.0))
            platform_transform = np.eye(4)
            platform_transform[2, 3] = platform
            return platform_transform @ model.arm_base_origin

        self.frames = CoordinateFrameService(
            model.tcp_transform,
            frame_repository,
            arm_base_transform,
        )
        self._solve_lock = threading.Lock()
        self._command_lock = threading.RLock()
        self.teach_program = TeachProgramService(
            model,
            controller,
            teach_points,
            self.settings,
            self._events,
            self.frames.pose_values,
            self.frames.default_from_display,
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
            self.displayed_target_values,
            self.set_displayed_target_values,
            self.move_joint_input,
            self.move_cartesian_input,
        )
        self.teach_program.motion_blocked = lambda: (
            self.continuous_jog.running or self._solve_lock.locked()
        )
        # 实时下发链路由适配层通过 attach_motion_streamer 注入；
        # 未注入时应用层仍可在纯仿真下运行。
        self.motion_streamer = None
        self.joint_guard = None
        self.oscilloscope = oscilloscope
        # URDF 选择：目录白名单由基础设施层持久化，运动期间禁止切换。
        catalog = UrdfCatalog(list(urdf_search_roots or []))
        self.urdf_library = UrdfLibraryService(
            catalog,
            preferences=urdf_preferences,
            active_path=active_urdf_path,
            motion_blocked=lambda: (
                self.teach_program.running or self.continuous_jog.running
            ),
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
                smooth_strength=self.settings.ik_smooth_strength,
                velocity_limit_dt=self.settings.ik_velocity_limit_dt,
                manipulability_weight=self.settings.ik_manipulability_weight,
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
        self.controller.target = self.frames.default_from_display(values)
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

    def displayed_target_values(self) -> np.ndarray:
        return np.asarray(self.frames.pose_values(self.controller.target), dtype=float)

    def set_displayed_target_values(self, values: np.ndarray, *, solve_live: bool = True) -> None:
        self.set_target_values(values, solve_live=solve_live)

    def create_base_frame(self, name: str, values: np.ndarray) -> None:
        self.frames.create_base(name, [float(value) for value in self.require_values({"values": values}, 6)])
        self.events.settings_changed()

    def create_tcp_frame(self, name: str, values: np.ndarray) -> None:
        self.frames.create_tcp(name, [float(value) for value in self.require_values({"values": values}, 6)])
        self.events.settings_changed()

    def select_frames(self, base: str, tcp: str) -> None:
        self.frames.select(base, tcp)
        self.events.target_changed()
        self.events.settings_changed()

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

    def move_auxiliary_input(self, name: str, value: float) -> None:
        if not np.isfinite(value):
            raise ValueError("附加轴目标必须为有限数值")
        self.set_auxiliary(name, value, solve_live=False)

    def jog_joint(self, name: str, delta_degrees: float) -> None:
        if name not in self.model.arm_joint_names:
            raise ValueError("未知机械臂关节")
        index = self.model.arm_joint_names.index(name)
        delta = np.deg2rad(float(delta_degrees))
        if not np.isfinite(delta):
            raise ValueError("关节点动增量无效")
        target = self.controller.arm.copy()
        target[index] = np.clip(
                target[index] + delta,
                self.model.lower[index],
                self.model.upper[index],
            )
        self.move_joint_input(np.rad2deg(target))

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
        if self.teach_program.running or self.continuous_jog.running:
            raise ValueError("机器人运动期间不能调用配置文件")
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
        self._apply_sampling_frequency()
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

    def save_teach_point_profile(self, name: str) -> None:
        if self.teach_point_profiles is None:
            raise ValueError("当前应用未配置示教点位配置仓储")
        self.teach_point_profiles.save(name, self.teach_points.as_json())
        self.active_teach_point_profile = name.strip()

    def load_teach_point_profile(self, name: str) -> None:
        if self.teach_point_profiles is None:
            raise ValueError("当前应用未配置示教点位配置仓储")
        if self.teach_program.running or self.continuous_jog.running:
            raise ValueError("机器人运动期间不能调用示教点位配置")
        self.teach_points.replace(self.teach_point_profiles.get(name))
        self.active_teach_point_profile = name
        self.teach_program.set_status(f"已调用示教点位配置：{name}")

    def move_cartesian_input(self, values: np.ndarray) -> None:
        values = self.require_values({"values": values}, 6)
        # Publish the entered target before the asynchronous interpolator
        # starts, so polling clients never restore stale field values.
        self.controller.target = self.frames.default_from_display(values)
        self.events.target_changed()
        self.teach_program.move_values(
            "MOVL",
            joint_values=[float(value) for value in np.rad2deg(self.controller.arm)],
            cartesian_values=[float(value) for value in values],
            name="笛卡尔输入目标",
        )

    def move_joint_input(self, values: np.ndarray) -> None:
        values = self.require_values({"values": values}, len(self.model.arm_joint_names))
        self.controller.target = self.model.tcp_pose(
            np.deg2rad(values), self.controller.aux
        )
        self.events.target_changed()
        self.teach_program.move_values(
            "MOVJ",
            joint_values=[float(value) for value in values],
            cartesian_values=self.frames.pose_values(self.controller.target),
            name="关节输入目标",
        )

    def move_nullspace_input(self, delta_degrees: float) -> None:
        if not np.isfinite(delta_degrees):
            raise ValueError("零空间目标必须为有限数值")
        if self.teach_program.running or self.continuous_jog.running:
            raise ValueError("机器人运动期间不能执行零空间目标")
        self.continuous_jog.move_nullspace(float(delta_degrees))

    def update_settings(self, **values: Any) -> None:
        frequency = values.get("trajectory_frequency_hz")
        frequency_changed = False
        if frequency is not None:
            frequency = float(frequency)
            if not 50.0 <= frequency <= 1000.0:
                raise ValueError("示教控制器采样频率必须在 50 到 1000 Hz")
            if self.teach_program.running or self.continuous_jog.running:
                raise ValueError("机器人运动期间不能修改采样频率")
            frequency_changed = not np.isclose(
                frequency,
                float(self.settings.trajectory_frequency_hz),
            )
        for name, value in values.items():
            if not hasattr(self.settings, name):
                raise ValueError(f"未知设置：{name}")
            setattr(self.settings, name, value)
        if frequency_changed:
            self._apply_sampling_frequency()
        self.events.settings_changed()

    def _apply_sampling_frequency(self) -> None:
        frequency = float(self.settings.trajectory_frequency_hz)
        period_s = 1.0 / frequency
        # Cartesian and null-space IK use the same per-sample time base as
        # joint interpolation and hardware delivery.
        self.settings.ik_velocity_limit_dt = period_s
        if self.motion_streamer is not None:
            self.motion_streamer.set_minimum_send_period(period_s)
        self.hardware.set_control_period(
            max(1, min(20, int(round(period_s * 1000.0))))
        )

    @staticmethod
    def require_values(command: dict, count: int) -> np.ndarray:
        values = np.asarray(command["values"], dtype=float)
        if values.shape != (count,) or not np.all(np.isfinite(values)):
            raise ValueError(f"values 必须包含 {count} 个有限数值")
        return values

    def read_state(self) -> dict:
        target_values = self.displayed_target_values()
        solution = self.controller.solution
        return {
            "backend": self.controller.solver.backend_name,
            "robot": {
                "arm_joint_names": list(self.model.arm_joint_names),
                "aux_joint_names": list(self.model.aux_joint_names),
                "auxiliary_labels": self.model.auxiliary_labels,
            },
            "target": {
                "position_m": [float(value) for value in target_values[:3]],
                "rpy_degrees": [float(value) for value in target_values[3:]],
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
                "ik_smooth_strength": self.settings.ik_smooth_strength,
                "ik_velocity_limit_dt": self.settings.ik_velocity_limit_dt,
                "ik_manipulability_weight": (
                    self.settings.ik_manipulability_weight
                ),
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
            "coordinate_frames": self.frames.state(),
            "program_status": self.teach_program.status,
            "urdf": self.urdf_library.state(),
            "realtime": self.realtime_state(),
            "teach_program": {
                "duration": self.settings.point_duration_s,
                "frequency": self.settings.trajectory_frequency_hz,
                "loop": self.settings.loop_teach_program,
                "running": self.teach_program.running,
                "points": self.teach_points.as_json(),
                "profiles": (
                    [] if self.teach_point_profiles is None
                    else self.teach_point_profiles.names()
                ),
                "active_profile": self.active_teach_point_profile,
            },
        }

    def handle_command(self, command: dict) -> dict:
        return self.command_dispatcher.dispatch(command)

    def attach_motion_streamer(self, streamer, guard=None) -> None:
        """绑定异步关节下发器与安全门。

        由适配层在装配时调用。应用层只保存引用用于诊断上报和退出清理，
        真正的下发路径由 ``ApplicationEvents.motion_sample`` 决定，
        因此应用层不会因为持有下发器而绕过安全门。
        """
        self.motion_streamer = streamer
        self.joint_guard = guard
        self.motion_streamer.set_minimum_send_period(
            1.0 / float(self.settings.trajectory_frequency_hz)
        )

    def realtime_state(self) -> dict[str, Any]:
        """上报下发链路与安全门状态，供示教器诊断页面显示。"""
        return {
            "streaming": (
                self.motion_streamer.statistics.as_json()
                if self.motion_streamer is not None
                else {}
            ),
            "safety": (
                self.joint_guard.state() if self.joint_guard is not None else {}
            ),
            "oscilloscope": (
                self.oscilloscope.statistics()
                if self.oscilloscope is not None
                else {}
            ),
        }

    def close(self) -> None:
        """按依赖顺序释放资源，避免线程在对象销毁后继续运行。"""
        self.continuous_jog.stop()
        self.teach_program.stop()
        if self.motion_streamer is not None:
            # 先排空再关闭，保证已接受的最后一个采样点被送出。
            self.motion_streamer.drain(timeout_s=0.5)
            self.motion_streamer.close()
        if self.oscilloscope is not None:
            self.oscilloscope.close()
        self.hardware.disconnect()
