"""Press-and-hold Cartesian and joint jog use case."""

from __future__ import annotations

import threading
from collections.abc import Callable

import numpy as np

from realtime import PacedLoop
from robot_framework.controller import Controller
from robot_framework.model_protocol import RobotModelProtocol
from robot_framework.solver import IKSolution
from robot_logging import get_logger
from trajectory import joint_limits_from_speed, plan_double_s, sample_double_s
from .contracts import ApplicationEvents, ApplicationSettings
from .null_space_motion import NullSpaceMotionService

_logger = get_logger("application.continuous_jog")


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
        move_joint_values: Callable[[np.ndarray], None] | None = None,
        move_cartesian_values: Callable[[np.ndarray], None] | None = None,
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
        self.move_joint_values = move_joint_values
        self.move_cartesian_values = move_cartesian_values
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
        if mode == "joint":
            self._run_joint(cancel, axis, direction, step_value)
            return
        # 点动节拍用绝对时基推进：每个采样点的截止时间由起点加序号推出，
        # 不使用「周期减去本次耗时」的相对休眠，避免抖动逐拍累积。
        frequency_hz = min(
            1000.0,
            max(50.0, self.settings.trajectory_frequency_hz),
        )
        loop = PacedLoop(frequency_hz, cancel, name=f"jog-{mode}")
        loop.reset()
        period_s = loop.period_s
        sample_index = 0
        first_sample = True
        try:
            while first_sample or not cancel.is_set():
                first_sample = False
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
                    raise AssertionError("joint mode is handled by _run_joint")
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
                sample_index += 1
                if loop.wait_until(sample_index * period_s):
                    return
        finally:
            if loop.statistics.overruns:
                _logger.debug(
                    "%s 点动有 %d/%d 拍滞后，最大 %.1f ms",
                    mode,
                    loop.statistics.overruns,
                    loop.statistics.samples,
                    loop.statistics.max_lateness_s * 1000.0,
                )
            if self._cancel is cancel:
                if mode == "nullspace":
                    self.null_space.end()
                self._running.clear()
                if not any(
                    marker in self.get_status()
                    for marker in ("连续点动停止：", "零空间运动停止：")
                ):
                    self.set_status("连续点动已停止")

    @staticmethod
    def _smoothstep(value: float) -> float:
        value = float(np.clip(value, 0.0, 1.0))
        return 10.0 * value**3 - 15.0 * value**4 + 6.0 * value**5

    def _run_joint(
        self,
        cancel: threading.Event,
        axis: int,
        direction: int,
        step_value: float,
    ) -> None:
        """Jerk-smooth hold-to-run joint jog on the controller sample grid."""
        frequency = float(self.settings.trajectory_frequency_hz)
        period_s = 1.0 / frequency
        ramp_duration_s = 0.30
        maximum_deg_s = min(
            step_value * 10.0,
            self.settings.max_joint_speed_deg_s,
        ) * self.settings.speed_percent / 100.0
        maximum = direction * np.deg2rad(maximum_deg_s)
        # The pacing event is deliberately separate from the operator cancel:
        # releasing the key starts a smooth deceleration instead of aborting
        # the deadline wait and commanding an instantaneous zero velocity.
        loop = PacedLoop(frequency, threading.Event(), name="jog-joint")
        loop.reset()
        sample_index = 0
        release_index: int | None = None
        release_velocity = 0.0
        previous_velocity = 0.0
        try:
            while True:
                elapsed = sample_index * period_s
                if cancel.is_set() and release_index is None:
                    release_index = sample_index
                    release_velocity = previous_velocity
                if release_index is None:
                    velocity = maximum * self._smoothstep(
                        elapsed / ramp_duration_s
                    )
                else:
                    release_elapsed = (
                        sample_index - release_index
                    ) * period_s
                    velocity = release_velocity * (
                        1.0
                        - self._smoothstep(
                            release_elapsed / ramp_duration_s
                        )
                    )
                acceleration = (velocity - previous_velocity) / period_s
                arm = self.controller.arm.copy()
                candidate = float(np.clip(
                    arm[axis] + velocity * period_s,
                    self.model.lower[axis],
                    self.model.upper[axis],
                ))
                at_limit = candidate == arm[axis] and abs(velocity) > 1e-12
                arm[axis] = candidate
                with self.controller._arm_lock:
                    self.controller.arm = arm.copy()
                self.controller.guide = arm.copy()
                self.controller.target = self.model.tcp_pose(
                    arm, self.controller.aux
                )
                self.controller.solution = None
                velocity_vector = np.zeros_like(arm)
                acceleration_vector = np.zeros_like(arm)
                velocity_vector[axis] = velocity
                acceleration_vector[axis] = acceleration
                self.events.guide_changed()
                self.events.target_changed()
                self.events.scene_changed()
                self.events.motion_sample(arm.copy())
                self.events.motion_state(
                    arm.copy(), velocity_vector, acceleration_vector
                )
                previous_velocity = velocity
                if at_limit:
                    self.set_status("连续点动停止：已到关节限位")
                    return
                if release_index is not None and (
                    sample_index - release_index
                ) * period_s >= ramp_duration_s:
                    return
                sample_index += 1
                loop.wait_until(sample_index * period_s)
        finally:
            if self._cancel is cancel:
                self._running.clear()
                if "连续点动停止：" not in self.get_status():
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
            if self.move_cartesian_values is None:
                raise ValueError("未配置笛卡尔轨迹执行器")
            self.move_cartesian_values(values)
            return

        if mode == "joint":
            if joint not in self.model.arm_joint_names:
                raise ValueError("未知机械臂关节")
            index = self.model.arm_joint_names.index(str(joint))
            target = self.controller.arm.copy()
            previous = float(target[index])
            target[index] = np.clip(
                    previous + direction * np.deg2rad(step),
                    self.model.lower[index],
                    self.model.upper[index],
                )
            at_limit = target[index] == previous
            if at_limit:
                raise ValueError("步进点动停止：已到关节限位")
            if self.move_joint_values is None:
                raise ValueError("未配置关节轨迹执行器")
            self.move_joint_values(np.rad2deg(target))
            return

        self.move_nullspace(direction * step)

    def move_nullspace(self, delta_degrees: float) -> None:
        """Execute a finite null-space move on the configured sample grid."""
        if self.program_running() or self.running:
            raise ValueError("机器人运动期间不能执行零空间目标")
        if not np.isfinite(delta_degrees):
            raise ValueError("零空间目标必须为有限数值")
        cancel = threading.Event()
        self._cancel = cancel
        self._running.set()

        def run() -> None:
            frequency = self.settings.trajectory_frequency_hz
            limits = joint_limits_from_speed(
                1,
                np.deg2rad(
                    self.settings.max_joint_speed_deg_s
                    * self.settings.speed_percent
                    / 100.0
                ),
            )
            profile = plan_double_s(
                np.zeros(1),
                np.array([np.deg2rad(delta_degrees)]),
                limits,
                minimum_duration_s=0.2,
            )
            samples = sample_double_s(profile, frequency)
            loop = PacedLoop(frequency, cancel, name="nullspace-target")
            previous = 0.0
            self.null_space.begin()
            try:
                loop.reset()
                for sample in samples[1:]:
                    if loop.wait_until(sample.time_s):
                        return
                    current = float(np.rad2deg(sample.position[0]))
                    self.null_space.step(current - previous)
                    previous = current
            finally:
                self.null_space.end()
                self._running.clear()

        threading.Thread(
            target=run,
            name="robot-nullspace-target",
            daemon=True,
        ).start()

    def stop(self) -> None:
        self._cancel.set()
