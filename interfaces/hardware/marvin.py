"""Marvin real-robot adapter using the bundled vendor Python SDK.

The adapter intentionally exposes no vendor planning or kinematics calls.
Our application supplies each interpolated joint sample; this adapter only
converts radians to degrees and sends it through the SDK's 1 kHz command
buffer.
"""

from __future__ import annotations

import importlib
import sys
import threading
import time
from pathlib import Path
from typing import Any, Callable

import numpy as np


class MarvinRobotHardware:
    # Values and 5 ms period are the vendor README's documented PD feedforward
    # configuration.  K is N*m/deg and D is the documented damping factor.
    _PD_K = (14.0, 14.0, 14.0, 10.5, 5.6, 5.6, 5.6)
    _PD_D = (0.3, 0.3, 0.3, 0.3, 0.3, 0.3, 0.3)
    _PD_PERIOD_MS = 5
    def __init__(
        self,
        project_root: Path,
        *,
        robot_factory: Callable[[], Any] | None = None,
        dcss_factory: Callable[[], Any] | None = None,
    ) -> None:
        self._project_root = project_root
        self._robot_factory = robot_factory
        self._dcss_factory = dcss_factory
        self._robot: Any | None = None
        self._dcss: Any | None = None
        self._lock = threading.RLock()
        self._connected = False
        self._enabled = False
        self._brake_released = False
        self._ip = ""
        self._arm = "B"
        self._pd_period_ms = self._PD_PERIOD_MS

    def _load_sdk(self) -> tuple[Callable[[], Any], Callable[[], Any]]:
        if self._robot_factory is not None and self._dcss_factory is not None:
            return self._robot_factory, self._dcss_factory
        sdk_root = self._project_root / "TJ_FX_ROBOT_CONTRL_SDK"
        if not sdk_root.is_dir():
            raise ValueError("未找到 TJ_FX_ROBOT_CONTRL_SDK 真机 SDK")
        sdk_parent = str(sdk_root)
        if sdk_parent not in sys.path:
            sys.path.insert(0, sdk_parent)
        try:
            module = importlib.import_module("SDK_PYTHON.fx_robot")
        except Exception as exc:  # vendor shared library import errors included
            raise ValueError(f"Marvin SDK 加载失败：{exc}") from exc
        return module.Marvin_Robot, module.DCSS

    @staticmethod
    def _validate_ip(ip: str) -> str:
        pieces = ip.strip().split(".")
        if len(pieces) != 4 or any(not piece.isdigit() or not 0 <= int(piece) <= 255 for piece in pieces):
            raise ValueError("机器人 IP 必须为 IPv4 地址")
        return ".".join(str(int(piece)) for piece in pieces)

    def connect(self, ip: str) -> None:
        ip = self._validate_ip(ip)
        with self._lock:
            if self._connected:
                self.disconnect()
            robot_factory, dcss_factory = self._load_sdk()
            robot, dcss = robot_factory(), dcss_factory()
            if not robot.connect(ip):
                raise ValueError("连接机器人失败：控制柜已被占用或 IP 不可达")
            # The vendor UI verifies the feedback channel by watching the
            # frame serial.  Do the same before exposing this as connected.
            serials: set[int] = set()
            try:
                for _ in range(5):
                    data = robot.subscribe(dcss)
                    serial = int(data["outputs"][0]["frame_serial"])
                    if serial:
                        serials.add(serial)
                    time.sleep(0.01)
            except Exception:
                robot.release_robot()
                raise ValueError("机器人反馈通道不可用")
            if len(serials) < 2:
                robot.release_robot()
                raise ValueError("机器人反馈未刷新，拒绝进入真机模式")
            self._robot, self._dcss = robot, dcss
            self._connected, self._enabled, self._brake_released = True, False, False
            self._ip = ip

    def _require_connected(self) -> Any:
        if not self._connected or self._robot is None:
            raise ValueError("请先连接机器人")
        return self._robot

    def disconnect(self) -> None:
        with self._lock:
            if self._robot is not None:
                try:
                    if self._enabled:
                        self._robot.clear_set()
                        self._robot.set_state(self._arm, 0)
                        self._robot.send_cmd()
                    self._robot.release_robot()
                finally:
                    self._robot = self._dcss = None
            self._connected = self._enabled = self._brake_released = False

    def enable(self, control_period_ms: int = _PD_PERIOD_MS) -> None:
        if not 1 <= int(control_period_ms) <= 20:
            raise ValueError("PD 前馈周期必须在 1 到 20 ms")
        with self._lock:
            robot = self._require_connected()
            # PD feedforward prerequisite 1: install the vendor's recommended
            # joint impedance gains and remove internal velocity limits.
            robot.clear_set()
            if not robot.set_joint_kd_params(
                self._arm, list(self._PD_K), list(self._PD_D)
            ):
                raise ValueError("PD 关节阻抗参数写入失败")
            if not robot.set_vel_acc(self._arm, 100, 100):
                raise ValueError("PD 速度/加速度参数写入失败")
            if not robot.send_cmd():
                raise ValueError("PD 参数命令发送失败")
            time.sleep(0.2)

            # Prerequisite 2: torque state + joint impedance mode.
            robot.clear_set()
            if not robot.set_state(self._arm, 3):  # ARM_STATE_TORQ
                raise ValueError("PD 扭矩状态切换失败")
            if not robot.set_impedance_type(self._arm, 1):
                raise ValueError("PD 关节阻抗模式切换失败")
            if not robot.send_cmd():
                raise ValueError("PD 模式命令发送失败")
            time.sleep(1.0)

            # Prerequisite 3: enable the SDK velocity feedforward estimator.
            robot.clear_set()
            self._pd_period_ms = int(control_period_ms)
            if not robot.set_PD_vel_est_step(self._arm, self._pd_period_ms):
                raise ValueError("PD 前馈周期设置失败")
            if not robot.send_cmd():
                raise ValueError("PD 前馈使能命令发送失败")
            self._enabled = True

    def disable(self) -> None:
        with self._lock:
            robot = self._require_connected()
            robot.clear_set()
            robot.set_PD_vel_est_step(self._arm, 0)
            robot.set_state(self._arm, 0)  # ARM_STATE_IDLE / 下伺服
            if not robot.send_cmd():
                raise ValueError("机器人下使能命令发送失败")
            self._enabled = False

    def _set_brake(self, value: int) -> None:
        robot = self._require_connected()
        if not robot.set_param("int", "BRAK1", value):
            raise ValueError("抱闸命令发送失败")
        self._brake_released = value == 2

    def release_brake(self) -> None:
        with self._lock:
            self._set_brake(2)

    def apply_brake(self) -> None:
        with self._lock:
            self._set_brake(1)

    def send_joint_radians(self, joints: np.ndarray) -> None:
        values = np.asarray(joints, dtype=float)
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise ValueError("真机关节指令必须包含 7 个有限弧度值")
        with self._lock:
            # Simulation remains fully usable while the right arm is not
            # explicitly connected and enabled.  Only an opt-in live session
            # receives the samples below.
            if not self._connected or not self._enabled:
                return
            robot = self._require_connected()
            degrees = [float(value) for value in np.rad2deg(values)]
            # set_joint_cmd_pose is the SDK/UI real-time tracking API.  The
            # vendor documentation requires this high-rate write to be inside
            # clear_set/send_cmd for every buffer cycle.
            robot.clear_set()
            if not robot.set_joint_cmd_pose(self._arm, degrees):
                raise ValueError("实时关节插补写入失败")
            if not robot.send_cmd():
                raise ValueError("实时关节插补发送失败")

    def state(self) -> dict[str, Any]:
        with self._lock:
            return {
                "available": True,
                "connected": self._connected,
                "enabled": self._enabled,
                "brake_released": self._brake_released,
                "control_mode": f"PD 前馈 · {self._pd_period_ms} ms",
                "arm": self._arm,
                "ip": self._ip,
            }
