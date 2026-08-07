"""MOVL and MOVJ teach-program execution use case."""

from __future__ import annotations

import threading
from collections.abc import Callable
from dataclasses import replace

import numpy as np
from scipy.spatial.transform import Rotation

from domain import TeachPoint
from realtime import PacedLoop
from robot_framework.controller import Controller
from robot_framework.model_protocol import RobotModelProtocol
from robot_logging import get_logger
from trajectory import (
    CartesianLimits,
    TrajectoryError,
    interpolate_cartesian_segment,
    joint_limits_from_speed,
    plan_cartesian_path_profile,
    plan_cartesian_trajectory,
    plan_joint_trajectory,
    sample_double_s,
)
from .contracts import ApplicationEvents, ApplicationSettings
from .ports import TeachPointRepository

_logger = get_logger("application.teach_program")


class TeachProgramService:
    def __init__(
        self,
        model: RobotModelProtocol,
        controller: Controller,
        repository: TeachPointRepository,
        settings: ApplicationSettings,
        events: ApplicationEvents,
        display_pose: Callable[[np.ndarray], list[float]] | None = None,
        default_target: Callable[[list[float] | np.ndarray], np.ndarray] | None = None,
    ) -> None:
        self.model = model
        self.controller = controller
        self.repository = repository
        self.settings = settings
        self.events = events
        self.motion_blocked: Callable[[], bool] = lambda: False
        self._cancel = threading.Event()
        self._running = threading.Event()
        self.status = "待机"
        self.display_pose = display_pose or self._default_display_pose
        self.default_target = default_target or self._default_target
        # Set by the application so a stopped or failed move releases any
        # real-time resources it acquired.
        self.on_motion_finished: Callable[[], None] = lambda: None

    def _speed_scale(self, point: TeachPoint) -> float:
        """Operator speed override for one point, as a fraction in (0, 1]."""
        percent = float(point.speed_percent)
        if not np.isfinite(percent) or not 0.0 < percent <= 100.0:
            raise ValueError("示教点速度百分比必须在 0 到 100 之间")
        return percent / 100.0

    def _cartesian_limits(self, speed_scale: float) -> CartesianLimits:
        """Task-space double-S limits for the configured speeds."""
        return CartesianLimits.from_pendant_units(
            max_linear_speed_mm_s=(
                self.settings.max_linear_speed_mm_s * speed_scale
            ),
            max_angular_speed_deg_s=(
                self.settings.max_angular_speed_deg_s * speed_scale
            ),
        )

    def _joint_limits(self, speed_scale: float):
        """Joint-space double-S limits for the configured joint speed."""
        return joint_limits_from_speed(
            len(self.model.arm_joint_names),
            np.deg2rad(self.settings.max_joint_speed_deg_s * speed_scale),
        )

    def _publish_sample(
        self,
        arm: np.ndarray,
        *,
        solution=None,
        velocity: np.ndarray | None = None,
        acceleration: np.ndarray | None = None,
    ) -> None:
        """Publish one accepted motion sample to the scene and to hardware.

        ``save_if_due`` throttles the diagnostic snapshot so the interpolation
        rate is not also a disk-write rate; the motion event itself is emitted
        every sample because that is what drives the robot.
        """
        with self.controller._arm_lock:
            self.controller.arm = arm
        if solution is not None:
            self.controller.solution = solution
            self.controller.save_if_due()
        self.events.scene_changed()
        self.events.motion_sample(arm.copy())
        if velocity is not None and acceleration is not None:
            self.events.motion_state(
                arm.copy(), velocity.copy(), acceleration.copy()
            )

    def _default_display_pose(self, pose: np.ndarray) -> list[float]:
        xyz = pose[:3, 3]
        rpy = Rotation.from_matrix(pose[:3, :3]).as_euler("xyz", degrees=True)
        return [float(value) for value in np.r_[xyz, rpy]]

    def _default_target(self, values: list[float] | np.ndarray) -> np.ndarray:
        data = np.asarray(values, dtype=float)
        return self.model.pose(data[:3], Rotation.from_euler("xyz", data[3:], degrees=True))

    @property
    def running(self) -> bool:
        return self._running.is_set()

    def set_events(self, events: ApplicationEvents) -> None:
        self.events = events

    def set_status(self, value: str) -> None:
        self.status = value
        self.events.status_changed(value)

    def current_joint_values(self) -> list[float]:
        return [float(value) for value in np.rad2deg(self.controller.arm)]

    def current_cartesian_values(self) -> list[float]:
        actual = self.model.tcp_pose(self.controller.arm, self.controller.aux)
        return self.display_pose(actual)

    def save_current(self, motion_type: str, name: str = "") -> TeachPoint:
        point = self.repository.add(
            motion_type,
            self.current_joint_values(),
            self.current_cartesian_values(),
            name,
            self.settings.speed_percent,
        )
        self.set_status(f"已保存 {point.name} ({point.motion_type})")
        return point

    def _play_movl(self, point: TeachPoint, *, immediate: bool = False) -> bool:
        """Execute a MOVL with a double-S path profile.

        The profile bounds path velocity, acceleration and jerk, so
        ``point_duration_s`` acts as a minimum cycle time rather than as the
        thing that sets the speed.  A move that cannot be completed that quickly
        within the configured limits is stretched instead of over-speeding.
        """
        values = np.asarray(point.values, dtype=float)
        target = self.default_target(values)
        self.set_status(f"{point.name} MOVL 预检中…")
        current = self.model.tcp_pose(
            self.controller.arm,
            self.controller.aux,
        )
        speed_scale = self._speed_scale(point)
        limits = self._cartesian_limits(speed_scale)
        minimum_duration = (
            0.2 if immediate else self.settings.point_duration_s / speed_scale
        )
        frequency = self.settings.trajectory_frequency_hz
        loop = PacedLoop(frequency, self._cancel, name=f"MOVL {point.name}")

        self.set_status(f"执行 {point.name} MOVL")
        if immediate:
            # For an operator-initiated single point, solve each sample against
            # the preceding arm state instead of preflighting the whole path, so
            # the robot starts moving without a long up-front IK pause.
            profile, _, _ = plan_cartesian_path_profile(
                [current, target],
                limits,
                minimum_duration_s=minimum_duration,
            )
            samples = sample_double_s(profile, frequency)
            poses = interpolate_cartesian_segment(
                current,
                target,
                len(samples),
                progress=np.asarray(
                    [float(np.clip(item.position[0], 0.0, 1.0)) for item in samples]
                ),
            )
            arm = self.controller.arm.copy()
            loop.reset()
            for sample, pose in zip(samples[1:], poses[1:]):
                if self._cancel.is_set():
                    return False
                solution = self.controller.solver.solve(
                    pose,
                    arm,
                    self.controller.aux.copy(),
                    lock_orientation=True,
                    multi_start=False,
                    recovery_seeds=0,
                )
                if (
                    solution.position_error_m > 0.002
                    or solution.orientation_error_rad > np.deg2rad(1.0)
                ):
                    raise TrajectoryError("连续 IK 无法保持 MOVL 轨迹")
                if loop.wait_until(sample.time_s):
                    return False
                self._publish_sample(solution.arm.copy(), solution=solution)
                arm = solution.arm
        else:
            trajectory = plan_cartesian_trajectory(
                self.controller.solver,
                [current, target],
                self.controller.arm,
                self.controller.aux.copy(),
                minimum_duration,
                frequency,
                limits=limits,
            )
            loop.reset()
            for timestamp, solution in zip(trajectory.times, trajectory.solutions):
                if self._cancel.is_set() or loop.wait_until(float(timestamp)):
                    return False
                self._publish_sample(solution.arm.copy(), solution=solution)
        # Flush the final snapshot that the throttle may have skipped.
        self.controller.save_if_due(force=True)
        self.controller.target = target
        self.events.target_changed()
        self._log_loop(loop, point, "MOVL")
        return True

    def _log_loop(self, loop: PacedLoop, point: TeachPoint, motion: str) -> None:
        statistics = loop.statistics
        if statistics.overruns:
            _logger.warning(
                "%s %s 有 %d/%d 个插补点滞后，最大 %.1f ms",
                point.name,
                motion,
                statistics.overruns,
                statistics.samples,
                statistics.max_lateness_s * 1000.0,
            )
        else:
            _logger.debug(
                "%s %s 完成：%d 点，最大抖动 %.2f ms",
                point.name,
                motion,
                statistics.samples,
                statistics.max_jitter_s * 1000.0,
            )

    def _play_movj(self, point: TeachPoint, *, immediate: bool = False) -> bool:
        """Execute a MOVJ with a time-synchronised double-S joint profile.

        All joints share one duration, so the arm follows a straight line in
        joint space and every axis stops at the same instant.  The planner
        stretches the move when the configured joint speed cannot cover the
        distance in the requested cycle time.
        """
        target_arm = np.deg2rad(np.asarray(point.values, dtype=float))
        speed_scale = self._speed_scale(point)
        frequency = self.settings.trajectory_frequency_hz
        trajectory = plan_joint_trajectory(
            self.controller.arm,
            target_arm,
            self.model.lower,
            self.model.upper,
            0.2 if immediate else self.settings.point_duration_s / speed_scale,
            frequency,
            limits=self._joint_limits(speed_scale),
        )
        self.set_status(f"执行 {point.name} MOVJ")
        loop = PacedLoop(frequency, self._cancel, name=f"MOVJ {point.name}")
        loop.reset()
        for timestamp, arm, velocity, acceleration in zip(
            trajectory.times,
            trajectory.arms,
            trajectory.velocities,
            trajectory.accelerations,
        ):
            if self._cancel.is_set() or loop.wait_until(float(timestamp)):
                return False
            # MOVJ commands joints directly, so there is no IK solution to
            # publish; clearing it keeps the diagnostics from showing a stale
            # Cartesian error for a joint-space move.
            self.controller.solution = None
            self._publish_sample(
                arm.copy(),
                velocity=velocity,
                acceleration=acceleration,
            )
        self.controller.guide = self.controller.arm.copy()
        self.controller.target = self.model.tcp_pose(
            self.controller.arm,
            self.controller.aux,
        )
        self.events.guide_changed()
        self.events.target_changed()
        self._log_loop(loop, point, "MOVJ")
        return True

    def _run(self, loop: bool) -> None:
        if self.running or self.motion_blocked():
            self.set_status("当前有运动或 IK 正在执行")
            return
        selected = [
            replace(
                point,
                joint_values=point.joint_values.copy(),
                cartesian_values=point.cartesian_values.copy(),
                speed_percent=point.speed_percent,
            )
            for point in self.repository.points
            if point.checked
        ]
        if not selected:
            self.set_status("请先勾选至少一个示教点")
            return
        self._running.set()
        self._cancel.clear()
        try:
            while True:
                for point in selected:
                    if self._cancel.is_set():
                        self.set_status("示教程序已停止")
                        return
                    try:
                        completed = (
                            self._play_movl(point)
                            if point.motion_type == "MOVL"
                            else self._play_movj(point)
                        )
                    except (TrajectoryError, ValueError) as exc:
                        self.set_status(
                            f"{point.name} {point.motion_type} 拒绝执行：{exc}"
                        )
                        return
                    if not completed:
                        self.set_status("示教程序已停止")
                        return
                    if self.settings.command_delay_s > 0:
                        self.set_status(
                            f"{point.name} 后延时 "
                            f"{self.settings.command_delay_s:.2f} s"
                        )
                        if self._cancel.wait(self.settings.command_delay_s):
                            self.set_status("示教程序已停止")
                            return
                if not loop:
                    self.set_status("勾选示教点执行完成")
                    return
                self.set_status("循环：准备下一轮")
        finally:
            self._running.clear()

    def start(self, loop: bool | None = None) -> None:
        loop_value = (
            self.settings.loop_teach_program if loop is None else bool(loop)
        )
        threading.Thread(
            target=self._run,
            args=(loop_value,),
            name="e1pro-teach-program",
            daemon=True,
        ).start()

    def stop(self) -> None:
        self._cancel.set()

    def move_point(self, point_id: int) -> None:
        if self.running or self.motion_blocked():
            raise ValueError("当前有运动或 IK 正在执行")
        point = self.repository.get(point_id)

        def run_one() -> None:
            self._running.set()
            self._cancel.clear()
            try:
                completed = (
                    self._play_movl(point, immediate=True)
                    if point.motion_type == "MOVL"
                    else self._play_movj(point, immediate=True)
                )
                self.set_status(
                    f"已到达 {point.name}" if completed else "示教点运动已停止"
                )
            except (TrajectoryError, ValueError) as exc:
                self.set_status(f"{point.name} 运动失败：{exc}")
            finally:
                self._running.clear()

        threading.Thread(target=run_one, name="e1pro-move-teach-point", daemon=True).start()

    def move_values(
        self,
        motion_type: str,
        *,
        joint_values: list[float],
        cartesian_values: list[float],
        name: str,
    ) -> None:
        """Execute an unsaved input target through the same interpolators."""
        if motion_type not in {"MOVL", "MOVJ"}:
            raise ValueError("运动类型必须为 MOVL 或 MOVJ")
        point = TeachPoint(
            point_id=0,
            name=name,
            motion_type=motion_type,
            joint_values=np.asarray(joint_values, dtype=float),
            cartesian_values=np.asarray(cartesian_values, dtype=float),
            speed_percent=self.settings.speed_percent,
        )
        if point.motion_type == "MOVL":
            point.cartesian_values = np.asarray(cartesian_values, dtype=float)
        self._move_unsaved(point)

    def _move_unsaved(self, point: TeachPoint) -> None:
        if self.running or self.motion_blocked():
            raise ValueError("当前有运动或 IK 正在执行")

        def run_one() -> None:
            self._running.set()
            self._cancel.clear()
            try:
                completed = (
                    self._play_movl(point, immediate=True)
                    if point.motion_type == "MOVL"
                    else self._play_movj(point, immediate=True)
                )
                self.set_status(
                    f"已到达 {point.name}" if completed else "输入目标运动已停止"
                )
            except (TrajectoryError, ValueError) as exc:
                self.set_status(f"{point.name} 运动失败：{exc}")
            finally:
                self._running.clear()

        threading.Thread(target=run_one, name="e1pro-move-input-target", daemon=True).start()
