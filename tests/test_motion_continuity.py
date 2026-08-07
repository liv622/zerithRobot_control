from __future__ import annotations

import threading
import time
import tempfile
import unittest
from pathlib import Path

import numpy as np

from realtime import MotionStreamer
from application import ApplicationEvents, RobotApplicationService
from infrastructure import JsonTeachPointRepository
from robot_framework.controller import Controller
from robots.e1pro.model import RobotModel
from robots.e1pro.smoke import resolve_urdf_path
from interfaces.oscilloscope import OscilloscopeService
from trajectory import joint_limits_from_speed, plan_joint_trajectory


class _RecordingSink:
    def __init__(self) -> None:
        self.values: list[np.ndarray] = []
        self.times: list[float] = []
        self.done = threading.Event()

    def send_joint_radians(self, joints: np.ndarray) -> None:
        self.values.append(joints.copy())
        self.times.append(time.monotonic())
        if len(self.values) == 5:
            self.done.set()


class MotionContinuityTests(unittest.TestCase):
    def test_joint_one_and_two_continuous_jog_ramp_without_velocity_jump(self) -> None:
        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            model = RobotModel.from_urdf(resolve_urdf_path(root))
            controller = Controller(model, Path(directory) / "solution.json")
            service = RobotApplicationService(
                model,
                controller,
                JsonTeachPointRepository(Path(directory) / "points.json"),
            )
            scope = OscilloscopeService(7, sample_hz=50.0)
            service.events = ApplicationEvents(
                motion_sample=lambda positions: scope.submit_recorded_positions(
                    positions,
                    1.0 / service.settings.trajectory_frequency_hz,
                )
            )
            try:
                for axis in (0, 1):
                    start = len(scope.monitor.snapshot())
                    service.continuous_jog.start(
                        mode="joint",
                        direction=1,
                        step=5.0,
                        joint=model.arm_joint_names[axis],
                    )
                    time.sleep(0.10)
                    service.continuous_jog.stop()
                    deadline = time.monotonic() + 1.0
                    while service.continuous_jog.running and time.monotonic() < deadline:
                        time.sleep(0.005)
                    time.sleep(0.03)
                    recorded = scope.monitor.snapshot()[start:]
                    times = np.asarray([sample.t for sample in recorded])
                    positions = np.asarray(
                        [sample.positions for sample in recorded]
                    )
                    intervals = np.diff(times)
                    velocities = (
                        np.diff(positions[:, axis]) / intervals
                    )
                    self.assertGreater(len(velocities), 10)
                    np.testing.assert_allclose(
                        intervals,
                        1.0 / service.settings.trajectory_frequency_hz,
                        atol=1e-9,
                    )
                    # Velocity is derived only from recorded positions; its
                    # adjacent changes remain bounded by the jog ramp.
                    self.assertLess(
                        float(np.max(np.abs(np.diff(velocities)))),
                        np.deg2rad(1.0),
                    )
            finally:
                scope.stop()
                service.close()

    def test_all_seven_joints_have_bounded_differenced_velocity(self) -> None:
        frequency = 200.0
        limits = joint_limits_from_speed(7, np.deg2rad(60.0))
        trajectory = plan_joint_trajectory(
            np.zeros(7),
            np.deg2rad(np.array([40, -35, 55, -45, 30, 50, -25])),
            np.full(7, -3.0),
            np.full(7, 3.0),
            0.2,
            frequency,
            limits=limits,
        )
        positions = np.asarray(trajectory.arms)
        intervals = np.diff(trajectory.times)
        np.testing.assert_allclose(intervals, 1.0 / frequency, atol=1e-12)
        differenced_velocity = np.diff(positions, axis=0) / intervals[:, None]
        adjacent_change = np.abs(np.diff(differenced_velocity, axis=0))
        self.assertTrue(
            np.all(
                adjacent_change
                <= limits.max_acceleration[None, :] / frequency + 1e-7
            )
        )

    def test_runtime_frequency_updates_planning_ik_and_delivery_period(self) -> None:
        class StreamerProbe:
            statistics = None

            def __init__(self) -> None:
                self.period = None

            def set_minimum_send_period(self, period: float) -> None:
                self.period = period

            def drain(self, timeout_s: float = 1.0) -> bool:
                return True

            def close(self) -> None:
                return

        root = Path(__file__).resolve().parents[1]
        with tempfile.TemporaryDirectory() as directory:
            model = RobotModel.from_urdf(resolve_urdf_path(root))
            controller = Controller(model, Path(directory) / "solution.json")
            service = RobotApplicationService(
                model,
                controller,
                JsonTeachPointRepository(Path(directory) / "points.json"),
            )
            probe = StreamerProbe()
            service.attach_motion_streamer(probe)
            service.update_settings(trajectory_frequency_hz=125.0)
            self.assertAlmostEqual(probe.period, 0.008)
            self.assertAlmostEqual(service.settings.ik_velocity_limit_dt, 0.008)
            service.close()

    def test_user_frequency_defines_double_s_interpolation_grid(self) -> None:
        arguments = (
            np.zeros(1),
            np.array([0.5]),
            np.array([-2.0]),
            np.array([2.0]),
            0.5,
        )
        limits = joint_limits_from_speed(1, 1.0)
        low = plan_joint_trajectory(*arguments, 100.0, limits=limits)
        high = plan_joint_trajectory(*arguments, 400.0, limits=limits)
        np.testing.assert_allclose(np.diff(low.times), 0.01, atol=1e-12)
        np.testing.assert_allclose(np.diff(high.times), 0.0025, atol=1e-12)
        self.assertEqual(len(high.times) - 1, 4 * (len(low.times) - 1))

    def test_double_s_velocity_is_continuous_and_starts_and_ends_at_zero(self) -> None:
        trajectory = plan_joint_trajectory(
            np.zeros(2),
            np.array([1.0, -0.6]),
            np.full(2, -2.0),
            np.full(2, 2.0),
            0.2,
            200.0,
            limits=joint_limits_from_speed(2, 1.0),
        )
        velocities = np.asarray(trajectory.velocities)
        np.testing.assert_allclose(velocities[[0, -1]], 0.0, atol=1e-10)
        # Bounded acceleration implies no discrete velocity jump between
        # adjacent 5 ms control samples.
        max_step = np.max(np.abs(np.diff(velocities, axis=0)), axis=0)
        acceleration_limit = joint_limits_from_speed(2, 1.0).max_acceleration
        self.assertTrue(np.all(max_step <= acceleration_limit / 200.0 + 1e-9))

    def test_streamer_preserves_every_joint_sample_and_output_period(self) -> None:
        sink = _RecordingSink()
        streamer = MotionStreamer(
            sink,
            queue_depth=16,
            minimum_send_period_s=0.003,
        )
        try:
            expected = [np.array([float(index)]) for index in range(5)]
            for value in expected:
                streamer.submit(value)
            self.assertTrue(sink.done.wait(1.0))
            self.assertEqual(len(sink.values), len(expected))
            for actual, wanted in zip(sink.values, expected):
                np.testing.assert_array_equal(actual, wanted)
            self.assertTrue(np.all(np.diff(sink.times) >= 0.0025))
            self.assertEqual(streamer.statistics.dropped, 0)
        finally:
            streamer.close()


if __name__ == "__main__":
    unittest.main()
