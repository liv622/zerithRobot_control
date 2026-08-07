"""Viser scene-only adapter driven by the Robot teach pendant.

The Viser page intentionally contains no application controls.  Motion and
configuration commands belong to the teach pendant; this adapter only mirrors
the application state into the 3-D scene and exposes the transport endpoint
used by the pendant.
"""

from __future__ import annotations

import time
import re
import threading
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import viser
from viser.extras import ViserUrdf

from application import ApplicationEvents, RobotApplicationService
from application.hardware import NullRobotHardware
from communication import SimulationCommandServer
from infrastructure import (
    JsonConfigurationRepository,
    JsonCoordinateFrameRepository,
    JsonTeachPointRepository,
    JsonTeachPointProfileRepository,
    JsonUrdfPreferenceRepository,
)
from interfaces.hardware import MarvinRobotHardware
from interfaces.oscilloscope import OscilloscopeService
from realtime import (
    GuardedJointSink,
    JointCommandGuard,
    JointSafetyLimits,
    MotionStreamer,
)
from robot_framework.controller import Controller
from robot_framework.plugin import RobotPlugin
from robot_logging import configure_logging, get_logger
from trajectory import double_s_backend_name
from urdf import load_urdf_with_local_meshes

_logger = get_logger("interfaces.simulator")
VISER_REFRESH_HZ = 30.0


def _wxyz(rotation_matrix: np.ndarray) -> tuple[float, float, float, float]:
    xyzw = Rotation.from_matrix(rotation_matrix).as_quat()
    return (float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2]))


def run_ui(
    project_root: Path,
    host: str,
    port: int,
    *,
    plugin: RobotPlugin,
    urdf_path: Path | None = None,
    control_host: str = "127.0.0.1",
    control_port: int = 8765,
) -> None:
    """Start a Viser scene with no motion controls of its own."""
    # 日志目录随项目根目录，控制台保留 INFO 以上，文件保留完整上下文。
    configure_logging(log_directory=project_root / "logs")
    urdf_path = urdf_path or plugin.resolve_urdf_path(project_root)
    model = plugin.load_model(urdf_path)
    # Keep pendant data isolated by robot. A six-axis URDF must not try to
    # deserialize the seven-axis E1-PRO teach points/configuration profiles.
    state_suffix = "" if plugin.key == "e1pro" else "_" + re.sub(
        r"[^A-Za-z0-9_-]+", "_", urdf_path.stem
    )
    def state_file(name: str) -> Path:
        path = project_root / name
        return path if not state_suffix else path.with_name(
            f"{path.stem}{state_suffix}{path.suffix}"
        )
    controller = Controller(model, project_root / "last_solution.json")
    # 示波器 —— 独立后台采样线程，持续读取关节状态用于实时波形显示。
    oscilloscope = OscilloscopeService(
        joint_count=len(model.arm_joint_names),
        capacity=2000,
        sample_hz=50.0,
    )
    oscilloscope.start(get_joints=controller.arm_snapshot)

    service = RobotApplicationService(
        model,
        controller,
        JsonTeachPointRepository(
            state_file("teach_points.json"),
            joint_count=len(model.arm_joint_names),
        ),
        JsonConfigurationRepository(state_file("robot_profiles.json")),
        # The Marvin SDK accepts only its own seven-axis E1-PRO joint vector.
        # A user supplied URDF remains fully teachable in simulation, without
        # accidentally sending incompatible joint commands to real hardware.
        hardware=(MarvinRobotHardware(project_root) if plugin.key == "e1pro" else NullRobotHardware()),
        frame_repository=JsonCoordinateFrameRepository(state_file("coordinate_frames.json")),
        teach_point_profiles=JsonTeachPointProfileRepository(state_file("teach_point_profiles.json")),
        urdf_preferences=JsonUrdfPreferenceRepository(
            state_file("urdf_preferences.json")
        ),
        # 默认授权随机器人交付的资源目录，操作员可再追加其他文件夹。
        urdf_search_roots=[project_root / "e1_pro_full"],
        active_urdf_path=urdf_path,
        oscilloscope=oscilloscope,
    )

    # 关节下发链路：安全门 -> 异步下发线程。
    # 安全门独立于规划器再次校验限位与速度，下发线程保证 200 Hz 插补循环
    # 不会被 SDK 调用阻塞，且下发异常不会杀死运动线程。
    safety_limits = JointSafetyLimits.build(
        model.lower,
        model.upper,
        np.deg2rad(service.settings.max_joint_speed_deg_s) * 3.0,
        np.deg2rad(service.settings.max_joint_speed_deg_s) * 12.0,
    )
    joint_guard = JointCommandGuard(safety_limits)
    streamer = MotionStreamer(
        GuardedJointSink(service.hardware, joint_guard),
        queue_depth=max(
            512, int(service.settings.trajectory_frequency_hz * 2.0)
        ),
        minimum_send_period_s=(
            1.0 / service.settings.trajectory_frequency_hz
        ),
    )
    service.attach_motion_streamer(streamer, joint_guard)
    urdf = load_urdf_with_local_meshes(urdf_path)
    actuated_names = tuple(urdf.actuated_joint_names)

    server = viser.ViserServer(host=host, port=port)
    server.scene.add_grid(
        "/ground",
        width=2.5,
        height=2.5,
        cell_size=0.1,
        cell_thickness=1.0,
    )
    visual = ViserUrdf(server, urdf, root_node_name="/machine")

    def full_cfg() -> np.ndarray:
        values = dict(
            model.full_configuration(controller.aux)
            if hasattr(model, "full_configuration")
            else model.initial_configuration
        )
        values.update(controller.aux)
        values.update(
            {
                name: float(value)
                for name, value in zip(model.arm_joint_names, controller.arm)
            }
        )
        return np.array([values.get(name, 0.0) for name in actuated_names])

    # Frame the initial camera on the robot's actual bounding box.  Viser's
    # default camera keeps a 5 m standoff, which makes a slender or dual-arm
    # robot (about 1 m across) look like a speck in the viewport.
    urdf.update_cfg(full_cfg())
    scene_bounds = urdf.scene.bounds
    bounds_center = (scene_bounds[0] + scene_bounds[1]) / 2.0
    bounds_size = float(np.linalg.norm(scene_bounds[1] - scene_bounds[0]))
    if bounds_size > 1e-6:
        direction = np.array([1.0, 1.0, 0.6])
        direction /= float(np.linalg.norm(direction))
        server.initial_camera.position = tuple(
            bounds_center + direction * bounds_size * 1.4
        )
        server.initial_camera.look_at = tuple(bounds_center)
        server.initial_camera.up = (0.0, 0.0, 1.0)

    visual.update_cfg(full_cfg())
    target_frame = server.scene.add_frame(
        "/tcp_target",
        axes_length=0.09,
        axes_radius=0.003,
        position=tuple(controller.target[:3, 3]),
        wxyz=_wxyz(controller.target[:3, :3]),
    )
    actual_pose = model.tcp_pose(controller.arm, controller.aux)
    actual_frame = server.scene.add_frame(
        "/actual_tcp",
        axes_length=0.07,
        axes_radius=0.0025,
        position=tuple(actual_pose[:3, 3]),
        wxyz=_wxyz(actual_pose[:3, :3]),
    )
    reachable_ball = server.scene.add_icosphere(
        "/actual_tcp/reachable",
        radius=0.012,
        color=(40, 210, 90),
    )
    unreachable_ball = server.scene.add_icosphere(
        "/actual_tcp/unreachable",
        radius=0.014,
        color=(240, 55, 55),
        visible=False,
    )

    def refresh_scene() -> None:
        visual.update_cfg(full_cfg())
        target_frame.position = tuple(controller.target[:3, 3])
        target_frame.wxyz = _wxyz(controller.target[:3, :3])
        actual = model.tcp_pose(controller.arm, controller.aux)
        actual_frame.position = tuple(actual[:3, 3])
        actual_frame.wxyz = _wxyz(actual[:3, :3])
        solution = controller.solution
        if solution is not None:
            reachable = bool(solution.reachable)
            reachable_ball.visible = reachable
            unreachable_ball.visible = not reachable

    scene_stop = threading.Event()

    def scene_render_loop() -> None:
        """Render at a UI rate independent from the control sample rate."""
        period = 1.0 / VISER_REFRESH_HZ
        while not scene_stop.is_set():
            started = time.monotonic()
            refresh_scene()
            scene_stop.wait(max(0.0, period - (time.monotonic() - started)))

    scene_thread = threading.Thread(
        target=scene_render_loop,
        name="viser-low-rate-render",
        daemon=True,
    )
    scene_thread.start()

    def publish_motion_sample(arm: np.ndarray) -> None:
        # Record the same position sample that is handed to the hardware.
        # The oscilloscope performs its own timestamped finite differences.
        oscilloscope.submit_recorded_positions(
            arm,
            1.0 / float(service.settings.trajectory_frequency_hz),
        )
        streamer.submit(arm)

    service.events = ApplicationEvents(
        # Rendering is driven by the independent low-rate loop above. Motion
        # events therefore never perform Viser work in the control loop.
        scene_changed=lambda: None,
        target_changed=lambda: None,
        solution_changed=lambda _: None,
        # 运动采样点交给异步下发器，绝不在插补循环内直接调用 SDK。
        motion_sample=publish_motion_sample,
    )
    service.solve()
    command_server = SimulationCommandServer(
        control_host,
        control_port,
        service.read_state,
        service.handle_command,
        oscilloscope=oscilloscope,
    )
    command_server.start()
    api_host, api_port = command_server.address
    _logger.info("已加载机器人 %s：%s", plugin.display_name, urdf_path)
    _logger.info("轨迹速度规划后端：%s", double_s_backend_name())
    _logger.info("示教器通信接口：http://%s:%s", api_host, api_port)
    print(f"已加载机器人 {plugin.display_name}：{urdf_path}")
    print("Viser 仅用于三维实时显示，所有操作请在 Robot 示教器中执行。")
    print(f"示教器通信接口：http://{api_host}:{api_port}")
    print("请打开 Robot 示教器中的“仿真”页面。按 Ctrl+C 退出。")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("正在退出。")
    finally:
        # 退出顺序：先停命令入口，再停运动下发，最后释放应用资源，
        # 确保没有线程在对象销毁后继续访问硬件。
        command_server.close()
        scene_stop.set()
        scene_thread.join(timeout=1.0)
        service.close()
        streamer.close()
        oscilloscope.stop()
