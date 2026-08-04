from __future__ import annotations

import unittest
from pathlib import Path

import numpy as np

from interfaces.hardware.marvin import MarvinRobotHardware


class FakeDCSS:
    pass


class FakeRobot:
    def __init__(self) -> None:
        self.calls: list[tuple] = []
        self.serial = 0

    def connect(self, ip: str) -> bool:
        self.calls.append(("connect", ip))
        return True

    def subscribe(self, _dcss: FakeDCSS) -> dict:
        self.serial += 1
        return {"outputs": [{"frame_serial": self.serial}]}

    def clear_set(self) -> bool:
        self.calls.append(("clear_set",))
        return True

    def set_state(self, arm: str, state: int) -> bool:
        self.calls.append(("set_state", arm, state))
        return True

    def set_joint_kd_params(self, arm: str, k: list[float], d: list[float]) -> bool:
        self.calls.append(("joint_kd", arm, k, d))
        return True

    def set_vel_acc(self, arm: str, velocity: int, acceleration: int) -> bool:
        self.calls.append(("vel_acc", arm, velocity, acceleration))
        return True

    def set_impedance_type(self, arm: str, mode: int) -> bool:
        self.calls.append(("impedance", arm, mode))
        return True

    def set_PD_vel_est_step(self, arm: str, period_ms: int) -> bool:
        self.calls.append(("pd_period", arm, period_ms))
        return True

    def set_joint_cmd_pose(self, arm: str, joints: list[float]) -> bool:
        self.calls.append(("joint", arm, joints))
        return True

    def send_cmd(self) -> bool:
        self.calls.append(("send_cmd",))
        return True

    def set_param(self, kind: str, name: str, value: int) -> bool:
        self.calls.append(("param", kind, name, value))
        return True

    def release_robot(self) -> bool:
        self.calls.append(("release",))
        return True


class MarvinHardwareTests(unittest.TestCase):
    def test_right_arm_uses_realtime_joint_tracking_buffer(self) -> None:
        robot = FakeRobot()
        hardware = MarvinRobotHardware(
            Path(__file__).resolve().parents[1],
            robot_factory=lambda: robot,
            dcss_factory=FakeDCSS,
        )
        hardware.connect("192.168.001.190")
        hardware.release_brake()
        hardware.enable(5)
        hardware.send_joint_radians(np.deg2rad(np.arange(7, dtype=float)))
        hardware.apply_brake()
        hardware.disable()

        self.assertEqual(hardware.state()["arm"], "B")
        self.assertEqual(hardware.state()["control_mode"], "PD 前馈 · 5 ms")
        self.assertIn(("param", "int", "BRAK1", 2), robot.calls)
        self.assertIn(("param", "int", "BRAK1", 1), robot.calls)
        self.assertIn(("joint_kd", "B", [14.0, 14.0, 14.0, 10.5, 5.6, 5.6, 5.6], [0.3] * 7), robot.calls)
        self.assertIn(("vel_acc", "B", 100, 100), robot.calls)
        self.assertIn(("set_state", "B", 3), robot.calls)
        self.assertIn(("impedance", "B", 1), robot.calls)
        self.assertIn(("pd_period", "B", 5), robot.calls)
        self.assertIn(("pd_period", "B", 0), robot.calls)
        self.assertIn(("set_state", "B", 0), robot.calls)
        sample = next(call for call in robot.calls if call[0] == "joint")
        self.assertEqual(sample[1], "B")
        np.testing.assert_allclose(
            sample[2],
            [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            atol=1e-12,
        )
        index = robot.calls.index(sample)
        self.assertEqual(robot.calls[index - 1], ("clear_set",))
        self.assertEqual(robot.calls[index + 1], ("send_cmd",))

    def test_disconnected_simulation_samples_are_noops(self) -> None:
        robot = FakeRobot()
        hardware = MarvinRobotHardware(
            Path(__file__).resolve().parents[1],
            robot_factory=lambda: robot,
            dcss_factory=FakeDCSS,
        )
        hardware.send_joint_radians(np.zeros(7))
        self.assertEqual(robot.calls, [])


if __name__ == "__main__":
    unittest.main()
