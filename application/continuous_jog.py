"""Press-and-hold Cartesian and joint jog use case."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable

import numpy as np

from robot_framework.controller import Controller
from robot_framework.model_protocol import RobotModelProtocol
from robot_framework.solver import IKSolution
from .contracts import ApplicationEvents, ApplicationSettings
from .null_space_motion import NullSpaceMotionService


class ContinuousJogService:
    def __init__(
        self,
        model: RobotModelProtocol,
        controller: Controller,
        events: ApplicationEvents,
        solve: Callable[..., IKSolution | None],
        program_running: Callable[[], bool],
        set_status: Callable[[str], None],
        get_status: Callable[[], str],
        settings: ApplicationSettings,
        null_space: NullSpaceMotionService,
        displayed_target_values: Callable[[], np.ndarray],
        set_displayed_target_values: Callable[..., None],
    ) -> None:
        self.model = model
        self.controller = controller
        self.events = events
        self.solve = solve
        self.program_running = program_running
        self.set_status = set_status
        self.get_status = get_status
        self.settings = settings
        self.null_space = null_space
        self.displayed_target_values = displayed_target_values
        self.set_displayed_target_values = set_displayed_target_values
        self._cancel = threading.Event()
        self._running = threading.Event()

    @property
    def running(self) -> bool:
        return self._running.is_set()

    def set_events(self, events: ApplicationEvents) -> None:
        self.events = events

    def _run(
        self,
        cancel: threading.Event,
        mode: str,
        axis: int,
        direction: int,
        step_value: float,
    ) -> None:
        period_s = 1.0 / min(
            1000.0,
            max(50.0, self.settings.trajectory_frequency_hz),
        )
        first_sample = True
        try:
            while first_sample or not cancel.is_set():
                first_sample = False
                tick_started = time.monotonic()
                speed_scale = self.settings.speed_percent / 100.0
                if mode == "cartesian":
                    maximum = (
                        self.settings.max_linear_speed_mm_s
                        if axis < 3
                        else self.settings.max_angular_speed_deg_s
                    )
                    speed = min(step_value * 10.0, maximum) * speed_scale
                    increment = speed * period_s
                    values = self.displayed_target_values()
                    values[axis] += direction * increment * (
                        0.001 if axis < 3 else 1.0
                    )
                    self.set_displayed_target_values(values, solve_live=False)
                    solution = self.solve(
                        lock_orientation_override=True,
                        emit_motion=True,
                    )
                    if solution is not None and not solution.reachable:
                        self.set_status("连续点动停止：目标不可达")
                        return
                elif mode == "joint":
                    speed = min(
                        step_value * 10.0,
                        self.settings.max_joint_speed_deg_s,
                    ) * speed_scale
                    increment = speed * period_s
                    previous = float(self.controller.arm[axis])
                    self.controller.arm[axis] = np.clip(
                        previous + direction * np.deg2rad(increment),
                        self.model.lower[axis],
                        self.model.upper[axis],
                    )
                    if self.controller.arm[axis] == previous:
                        self.set_status("连续点动停止：已到关节限位")
                        return
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
                elif mode == "nullspace":
                    speed = min(
                        step_value * 10.0,
                        self.settings.max_joint_speed_deg_s,
                    ) * speed_scale
                    increment = speed * period_s
                    try:
                        self.null_space.step(direction * increment)
                    except ValueError as exc:
                        self.set_status(str(exc))
                        return
                else:
                    name = self.model.aux_joint_names[axis]
                    lower, upper = self.model.auxiliary_limits[name]
                    speed = min(
                        step_value * 10.0,
                        self.settings.max_joint_speed_deg_s,
                    ) * speed_scale * 0.001
                    previous = float(self.controller.aux.get(name, 0.0))
                    self.controller.aux[name] = float(np.clip(
                        previous + direction * speed * period_s,
                        lower,
                        upper,
                    ))
                    if self.controller.aux[name] == previous:
                        self.set_status("连续点动停止：已到附加轴限位")
                        return
                    self.events.auxiliary_changed()
                    self.events.scene_changed()
                elapsed = time.monotonic() - tick_started
                cancel.wait(max(0.0, period_s - elapsed))
        finally:
            if self._cancel is cancel:
                if mode == "nullspace":
                    self.null_space.end()
                self._running.clear()
                if not any(
                    marker in self.get_status()
                    for marker in ("连续点动停止：", "零空间运动停止：")
                ):
                    self.set_status("连续点动已停止")

    def start(
        self,
        *,
        mode: str,
        direction: int,
        step: float,
        axis: int | None = None,
        joint: str | None = None,
    ) -> None:
        if self.program_running():
            raise ValueError("示教程序运行期间不能连续点动")
        if mode not in {"cartesian", "joint", "nullspace", "auxiliary"}:
            raise ValueError(
                "连续点动模式必须是 cartesian、joint、nullspace 或 auxiliary"
            )
        if direction not in {-1, 1}:
            raise ValueError("连续点动方向必须是 -1 或 1")
        if not np.isfinite(step) or not 0.1 <= step <= 30.0:
            raise ValueError("连续点动步长必须在 0.1 到 30 之间")
        if mode == "cartesian":
            if axis not in range(6):
                raise ValueError("未知笛卡尔轴")
            axis_index = int(axis)
            label = ("X", "Y", "Z", "Rx", "Ry", "Rz")[axis_index]
            self.controller.target = self.model.tcp_pose(
                self.controller.arm,
                self.controller.aux,
            )
            self.events.target_changed()
        elif mode == "joint":
            if joint not in self.model.arm_joint_names:
                raise ValueError("未知机械臂关节")
            axis_index = self.model.arm_joint_names.index(str(joint))
            label = f"J{axis_index + 1}"
        elif mode == "nullspace":
            axis_index = 0
            self.null_space.begin()
            label = "零空间"
        else:
            if joint not in self.model.aux_joint_names:
                raise ValueError("未知附加轴")
            axis_index = self.model.aux_joint_names.index(str(joint))
            label = self.model.auxiliary_labels[str(joint)]

        self._cancel.set()
        cancel = threading.Event()
        self._cancel = cancel
        self._running.set()
        self.set_status(
            f"连续点动 {label}{'+' if direction > 0 else '-'}"
        )
        threading.Thread(
            target=self._run,
            args=(cancel, mode, axis_index, direction, step),
            name="robot-continuous-jog",
            daemon=True,
        ).start()

    def step_once(
        self,
        *,
        mode: str,
        direction: int,
        step: float,
        axis: int | None = None,
        joint: str | None = None,
    ) -> None:
        """Move exactly one user-selected Cartesian or joint-space step."""
        if self.program_running() or self.running:
            raise ValueError("机器人运动期间不能执行步进点动")
        if mode not in {"cartesian", "joint", "nullspace"}:
            raise ValueError("步进点动模式无效")
        if direction not in {-1, 1}:
            raise ValueError("步进方向必须是 -1 或 1")
        if not np.isfinite(step) or not 0.1 <= step <= 30.0:
            raise ValueError("点动步长必须在 0.1 到 30 之间")

        if mode == "cartesian":
            if axis not in range(6):
                raise ValueError("未知笛卡尔轴")
            self.controller.target = self.model.tcp_pose(
                self.controller.arm,
                self.controller.aux,
            )
            values = self.displayed_target_values()
            values[int(axis)] += direction * step * (
                0.001 if axis < 3 else 1.0
            )
            self.set_displayed_target_values(values, solve_live=False)
            solution = self.solve(
                lock_orientation_override=True,
                emit_motion=True,
            )
            if solution is not None and not solution.reachable:
                raise ValueError("步进点动停止：目标不可达")
            return

        if mode == "joint":
            if joint not in self.model.arm_joint_names:
                raise ValueError("未知机械臂关节")
            index = self.model.arm_joint_names.index(str(joint))
            previous = float(self.controller.arm[index])
            self.controller.arm[index] = np.clip(
                previous + direction * np.deg2rad(step),
                self.model.lower[index],
                self.model.upper[index],
            )
            if self.controller.arm[index] == previous:
                raise ValueError("步进点动停止：已到关节限位")
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
            return

        self.null_space.begin()
        try:
            self.null_space.step(direction * step)
        finally:
            self.null_space.end()

    def stop(self) -> None:
        self._cancel.set()
