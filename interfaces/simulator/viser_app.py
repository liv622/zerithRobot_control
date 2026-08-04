"""Viser scene-only adapter driven by the zerithRobot teach pendant.

The Viser page intentionally contains no application controls.  Motion and
configuration commands belong to the teach pendant; this adapter only mirrors
the application state into the 3-D scene and exposes the transport endpoint
used by the pendant.
"""

from __future__ import annotations

import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation
import viser
import yourdfpy
from viser.extras import ViserUrdf

from application import ApplicationEvents, RobotApplicationService
from communication import SimulationCommandServer
from infrastructure import (
    JsonConfigurationRepository,
    JsonTeachPointRepository,
)
from interfaces.hardware import MarvinRobotHardware
from robot_framework.controller import Controller
from robot_framework.plugin import RobotPlugin


def _wxyz(rotation_matrix: np.ndarray) -> tuple[float, float, float, float]:
    xyzw = Rotation.from_matrix(rotation_matrix).as_quat()
    return (float(xyzw[3]), float(xyzw[0]), float(xyzw[1]), float(xyzw[2]))


def run_ui(
    project_root: Path,
    host: str,
    port: int,
    *,
    plugin: RobotPlugin,
    control_host: str = "127.0.0.1",
    control_port: int = 8765,
) -> None:
    """Start a Viser scene with no motion controls of its own."""
    urdf_path = project_root / plugin.urdf_relative_path
    model = plugin.load_model(urdf_path)
    controller = Controller(model, project_root / "last_solution.json")
    service = RobotApplicationService(
        model,
        controller,
        JsonTeachPointRepository(project_root / "teach_points.json"),
        JsonConfigurationRepository(project_root / "robot_profiles.json"),
        hardware=MarvinRobotHardware(project_root),
    )
    urdf = yourdfpy.URDF.load(str(urdf_path))
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
        values = dict(model.initial_configuration)
        values.update(controller.aux)
        values.update(
            {
                name: float(value)
                for name, value in zip(model.arm_joint_names, controller.arm)
            }
        )
        return np.array([values.get(name, 0.0) for name in actuated_names])

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

    def refresh_target() -> None:
        target_frame.position = tuple(controller.target[:3, 3])
        target_frame.wxyz = _wxyz(controller.target[:3, :3])

    def refresh_scene() -> None:
        visual.update_cfg(full_cfg())
        actual = model.tcp_pose(controller.arm, controller.aux)
        actual_frame.position = tuple(actual[:3, 3])
        actual_frame.wxyz = _wxyz(actual[:3, :3])

    def refresh_solution(solution: object) -> None:
        reachable = bool(getattr(solution, "reachable", False))
        reachable_ball.visible = reachable
        unreachable_ball.visible = not reachable

    service.events = ApplicationEvents(
        scene_changed=refresh_scene,
        target_changed=refresh_target,
        solution_changed=refresh_solution,
        motion_sample=service.hardware.send_joint_radians,
    )
    service.solve()
    command_server = SimulationCommandServer(
        control_host,
        control_port,
        service.read_state,
        service.handle_command,
    )
    command_server.start()
    print(f"已加载机器人 {plugin.display_name}：{urdf_path}")
    print("Viser 仅用于三维实时显示，所有操作请在 zerithRobot 示教器中执行。")
    api_host, api_port = command_server.address
    print(f"示教器通信接口：http://{api_host}:{api_port}")
    print("请打开 zerithRobot 示教器中的“仿真”页面。按 Ctrl+C 退出。")
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        print("正在退出。")
    finally:
        service.close()
        command_server.close()
