"""MOVL and MOVJ teach-program execution use case."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import replace

import numpy as np
from scipy.spatial.transform import Rotation

from domain import TeachPoint
from robot_framework.controller import Controller
from robot_framework.model_protocol import RobotModelProtocol
from trajectory import (
    TrajectoryError,
    interpolate_cartesian_segment,
    plan_cartesian_trajectory,
    plan_joint_trajectory,
)
from .contracts import ApplicationEvents, ApplicationSettings
from .ports import TeachPointRepository


class TeachProgramService:
    def __init__(
        self,
        model: RobotModelProtocol,
        controller: Controller,
        repository: TeachPointRepository,
        settings: ApplicationSettings,
        events: ApplicationEvents,
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
        xyz = actual[:3, 3]
        rpy = Rotation.from_matrix(actual[:3, :3]).as_euler(
            "xyz",
            degrees=True,
        )
        return [float(value) for value in np.r_[xyz, rpy]]

    def save_current(self, motion_type: str, name: str = "") -> TeachPoint:
        point = self.repository.add(
            motion_type,
            self.current_joint_values(),
            self.current_cartesian_values(),
            name,
        )
        self.set_status(f"已保存 {point.name} ({point.motion_type})")
        return point

    def _wait_for_sample(self, started: float, timestamp: float) -> bool:
        remaining = started + timestamp - time.monotonic()
        return remaining > 0 and self._cancel.wait(remaining)

    def _play_movl(self, point: TeachPoint, *, immediate: bool = False) -> bool:
        values = np.asarray(point.values, dtype=float)
        target = self.model.pose(
            values[:3],
            Rotation.from_euler("xyz", values[3:], degrees=True),
        )
        self.set_status(f"{point.name} MOVL 预检中…")
        current = self.model.tcp_pose(
            self.controller.arm,
            self.controller.aux,
        )
        speed_scale = self.settings.speed_percent / 100.0
        distance_mm = float(
            np.linalg.norm(target[:3, 3] - current[:3, 3]) * 1000.0
        )
        angle_deg = float(
            np.rad2deg(
                np.linalg.norm(
                    Rotation.from_matrix(
                        target[:3, :3] @ current[:3, :3].T
                    ).as_rotvec()
                )
            )
        )
        duration = max(
            0.2 if immediate else self.settings.point_duration_s / speed_scale,
            distance_mm
            / (self.settings.max_linear_speed_mm_s * speed_scale),
            angle_deg
            / (self.settings.max_angular_speed_deg_s * speed_scale),
        )
        self.set_status(f"执行 {point.name} MOVL")
        started = time.monotonic()
        if immediate:
            # For an operator-initiated single-point move, solve and execute
            # each sample in sequence. This uses the preceding arm state as
            # the seed and avoids blocking on a full-trajectory IK preflight.
            count = max(2, int(np.ceil(duration * self.settings.trajectory_frequency_hz)) + 1)
            poses = interpolate_cartesian_segment(current, target, count)
            times = np.linspace(0.0, duration, len(poses))
            arm = self.controller.arm.copy()
            for timestamp, pose in zip(times[1:], poses[1:]):
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
                if self._wait_for_sample(started, float(timestamp)):
                    return False
                self.controller.arm = solution.arm.copy()
                self.controller.solution = solution
                self.controller.save()
                self.events.scene_changed()
                arm = solution.arm
        else:
            trajectory = plan_cartesian_trajectory(
                self.controller.solver,
                [current, target],
                self.controller.arm,
                self.controller.aux.copy(),
                duration,
                self.settings.trajectory_frequency_hz,
            )
            for timestamp, solution in zip(trajectory.times, trajectory.solutions):
                if self._cancel.is_set() or self._wait_for_sample(
                    started,
                    float(timestamp),
                ):
                    return False
                self.controller.arm = solution.arm.copy()
                self.controller.solution = solution
                self.controller.save()
                self.events.scene_changed()
        self.controller.target = target
        self.events.target_changed()
        return True

    def _play_movj(self, point: TeachPoint, *, immediate: bool = False) -> bool:
        target_arm = np.deg2rad(np.asarray(point.values, dtype=float))
        speed_scale = self.settings.speed_percent / 100.0
        required_duration = float(
            np.max(np.abs(target_arm - self.controller.arm))
            / np.deg2rad(
                self.settings.max_joint_speed_deg_s * speed_scale
            )
        )
        trajectory = plan_joint_trajectory(
            self.controller.arm,
            target_arm,
            self.model.lower,
            self.model.upper,
            max(
                0.2 if immediate else self.settings.point_duration_s / speed_scale,
                required_duration,
            ),
            self.settings.trajectory_frequency_hz,
        )
        self.set_status(f"执行 {point.name} MOVJ")
        started = time.monotonic()
        for timestamp, arm in zip(trajectory.times, trajectory.arms):
            if self._cancel.is_set() or self._wait_for_sample(
                started,
                float(timestamp),
            ):
                return False
            self.controller.arm = arm.copy()
            self.controller.solution = None
            self.events.scene_changed()
        self.controller.guide = self.controller.arm.copy()
        self.controller.target = self.model.tcp_pose(
            self.controller.arm,
            self.controller.aux,
        )
        self.events.guide_changed()
        self.events.target_changed()
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
