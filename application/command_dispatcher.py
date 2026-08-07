"""Map transport-neutral commands to robot application use cases."""

from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .robot_service import RobotApplicationService


class CommandDispatcher:
    def __init__(self, application: RobotApplicationService) -> None:
        self.application = application

    def dispatch(self, command: dict) -> dict:
        app = self.application
        action = str(command.get("action", ""))
        with app.command_lock:
            if action == "start_continuous_jog":
                app.continuous_jog.start(
                    mode=str(command["mode"]),
                    direction=int(command["direction"]),
                    step=float(command["step"]),
                    axis=(
                        int(command["axis"])
                        if "axis" in command
                        else None
                    ),
                    joint=(
                        str(command["joint"])
                        if "joint" in command
                        else None
                    ),
                )
                return {"message": "连续点动已开始"}
            if action == "stop_continuous_jog":
                app.continuous_jog.stop()
                return {"message": "连续点动已停止"}
            if action == "connect_hardware":
                app.connect_hardware(str(command["ip"]))
                return {"message": "右臂真机反馈通道已连接，尚未上使能"}
            if action == "disconnect_hardware":
                app.disconnect_hardware()
                return {"message": "真机已断开，并已下使能"}
            if action == "enable_hardware":
                app.enable_hardware()
                return {"message": "右臂已启用关节阻抗 PD 前馈模式"}
            if action == "disable_hardware":
                app.disable_hardware()
                return {"message": "机器人已下使能"}
            if action == "release_hardware_brake":
                app.release_hardware_brake()
                return {"message": "已发送松闸命令"}
            if action == "apply_hardware_brake":
                app.apply_hardware_brake()
                return {"message": "已发送抱闸命令"}
            if action == "jog_step":
                app.continuous_jog.step_once(
                    mode=str(command["mode"]),
                    direction=int(command["direction"]),
                    step=float(command["step"]),
                    axis=(
                        int(command["axis"])
                        if "axis" in command
                        else None
                    ),
                    joint=(
                        str(command["joint"])
                        if "joint" in command
                        else None
                    ),
                )
                return {"message": "步进点动已执行"}
            if action == "set_target":
                app.set_target_values(
                    app.require_values(command, 6),
                    solve_live=bool(command.get("solve_live", False)),
                )
                return {"message": "TCP 输入目标已保存，尚未运动"}
            if action == "move_cartesian_input":
                app.move_cartesian_input(app.require_values(command, 6))
                return {"message": "笛卡尔输入目标已提交插补运动"}
            if action == "move_joint_input":
                app.move_joint_input(
                    app.require_values(command, len(app.model.arm_joint_names))
                )
                return {"message": "关节输入目标已提交插补运动"}
            if action == "move_nullspace_input":
                app.move_nullspace_input(float(command["delta_degrees"]))
                return {"message": "零空间输入目标已执行"}
            if action == "jog_target":
                axis = int(command["axis"])
                delta = float(command["delta"])
                if axis not in range(6) or not np.isfinite(delta):
                    raise ValueError("点动轴或增量无效")
                values = app.displayed_target_values()
                values[axis] += delta
                app.set_target_values(values)
                return {"message": f"笛卡尔轴 {axis + 1} 点动完成"}
            if action == "solve":
                app.solve()
                return {"message": "IK 求解完成"}
            if action == "recover":
                app.solve(force=True)
                return {"message": "多起点 IK 求解完成"}
            if action == "target_current":
                app.target_current()
                return {"message": "已读取当前实际 TCP"}
            if action == "toggle_drag":
                app.set_drag_unlocked(not app.settings.drag_unlocked)
                state = "解锁" if app.settings.drag_unlocked else "锁定"
                return {"message": f"场景 TCP 拖拽已{state}"}
            if action == "reset":
                app.reset()
                return {"message": "整机与目标已复位"}
            if action == "settings":
                seeds = int(command["recovery_count"])
                strength = float(command["guide_strength"])
                smooth = float(command.get("ik_smooth_strength", 0.3))
                # IK uses the teach-controller sampling period.  Keeping a
                # second independently editable period would make Cartesian
                # and null-space interpolation disagree with MOVJ.
                velocity_dt = 1.0 / app.settings.trajectory_frequency_hz
                manip = float(command.get("ik_manipulability_weight", 0.0))
                if not 4 <= seeds <= 24:
                    raise ValueError("恢复种子数必须在 4 到 24 之间")
                if not 0.0 <= strength <= 0.5:
                    raise ValueError("引导强度必须在 0 到 0.5 之间")
                if not 0.0 <= smooth <= 5.0:
                    raise ValueError("平滑强度必须在 0 到 5 之间")
                if not 0.001 <= float(velocity_dt) <= 0.02:
                    raise ValueError("IK 周期必须与 50 到 1000 Hz 采样频率一致")
                if not 0.0 <= manip <= 1.0:
                    raise ValueError("奇异规避权重必须在 0 到 1 之间")
                app.update_settings(
                    live_solve=bool(command["live"]),
                    orientation_lock=bool(command["orientation_lock"]),
                    auto_recovery=bool(command["auto_recovery"]),
                    recovery_count=seeds,
                    guide_enabled=bool(command["guide_enabled"]),
                    guide_strength=strength,
                    ik_smooth_strength=smooth,
                    ik_velocity_limit_dt=float(velocity_dt),
                    ik_manipulability_weight=manip,
                )
                return {"message": "求解参数已应用"}
            if action == "jog_joint":
                app.jog_joint(
                    str(command["joint"]),
                    float(command["delta_degrees"]),
                )
                return {"message": f"{command['joint']} 点动完成"}
            if action == "jog_aux":
                name = str(command["joint"])
                app.jog_auxiliary(name, float(command["delta"]))
                return {"message": f"{app.AUX_LABELS[name]}点动完成"}
            if action == "move_auxiliary_input":
                name = str(command["joint"])
                app.move_auxiliary_input(name, float(command["value"]))
                return {"message": f"{app.AUX_LABELS[name]}已运动到输入目标"}
            if action == "guide_current":
                app.guide_current()
                return {"message": "当前关节解已设为臂形参考"}
            if action == "switch_arm_shape":
                difference = app.switch_arm_shape(int(command["direction"]))
                return {
                    "message": (
                        "已保持 TCP 位姿并切换臂型，"
                        f"关节空间变化 {difference:.3f} rad"
                    )
                }
            if action == "set_active_arm":
                side = str(command["side"])
                app.set_active_arm(side)
                label = "左手" if side == "left" else "右手"
                return {"message": f"已切换到{label}"}
            if action == "motion_settings":
                app.update_motion_settings(command)
                return {"message": "全局速度与延时参数已应用"}
            if action == "save_configuration":
                name = str(command.get("name", ""))
                app.save_configuration(name)
                return {"message": f"配置文件 {name.strip()} 已保存"}
            if action == "load_configuration":
                name = str(command["name"])
                app.load_configuration(name)
                return {"message": f"配置文件 {name} 已调用"}
            if action == "delete_configuration":
                name = str(command["name"])
                app.delete_configuration(name)
                return {"message": f"配置文件 {name} 已删除"}
            if action == "create_base_frame":
                app.create_base_frame(
                    str(command["name"]), app.require_values(command, 6)
                )
                return {"message": "用户基坐标系已创建"}
            if action == "create_tcp_frame":
                app.create_tcp_frame(
                    str(command["name"]), app.require_values(command, 6)
                )
                return {"message": "用户 TCP 坐标系已创建"}
            if action == "select_coordinate_frames":
                app.select_frames(str(command["base"]), str(command["tcp"]))
                return {"message": "位姿显示与运动参考坐标系已切换"}
            if action == "save_teach_point":
                point = app.teach_program.save_current(
                    str(command["motion_type"]).upper(),
                    str(command.get("name", "")),
                )
                return {
                    "message": f"已保存 {point.name} ({point.motion_type})",
                    "point": {
                        "point_id": point.point_id,
                        "name": point.name,
                    },
                }
            if action == "update_teach_point":
                point = app.teach_points.get(int(command["point_id"]))
                motion_type = str(command["motion_type"]).upper()
                joints = app.require_values(
                    {"values": command["joint_values"]},
                    len(app.model.arm_joint_names),
                )
                cartesian = app.require_values({"values": command["cartesian_values"]}, 6)
                speed_percent = float(command["speed_percent"])
                point = app.teach_points.update(
                    point.point_id,
                    str(command.get("name", point.name)),
                    motion_type,
                    [float(value) for value in joints],
                    [float(value) for value in cartesian],
                    speed_percent,
                )
                app.teach_program.set_status(f"已修改 {point.name}")
                return {"message": f"已修改 {point.name} ({point.motion_type})"}
            if action == "move_teach_point":
                app.teach_program.move_point(int(command["point_id"]))
                return {"message": "示教点运动已启动"}
            if action == "delete_teach_point":
                point = app.teach_points.get(int(command["point_id"]))
                app.teach_points.delete(point.point_id)
                app.teach_program.set_status(f"已删除 {point.name}")
                return {"message": f"已删除 {point.name}"}
            if action == "save_teach_point_profile":
                name = str(command["name"])
                app.save_teach_point_profile(name)
                return {"message": f"示教点位配置 {name.strip()} 已保存"}
            if action == "load_teach_point_profile":
                name = str(command["name"])
                app.load_teach_point_profile(name)
                return {"message": f"示教点位配置 {name} 已调用"}
            if action == "set_teach_point_checked":
                app.teach_points.set_checked(
                    int(command["point_id"]),
                    bool(command["checked"]),
                )
                return {"message": "示教点勾选状态已更新"}
            if action in {"set_teach_program_settings", "run_teach_points"}:
                duration = float(command["duration"])
                frequency = float(command["frequency"])
                if duration < 0.2 or not 50.0 <= frequency <= 1000.0:
                    raise ValueError("单点时长至少 0.2 s，插补频率须在 50 到 1000 Hz")
                loop = bool(command.get("loop", False))
                app.update_settings(
                    point_duration_s=duration,
                    trajectory_frequency_hz=frequency,
                    loop_teach_program=loop,
                )
                if action == "set_teach_program_settings":
                    return {"message": "示教程序参数已更新"}
                app.teach_program.start(loop)
                return {
                    "message": (
                        "循环示教程序已启动"
                        if loop
                        else "勾选示教点已提交执行"
                    )
                }
            if action == "stop_teach_points":
                app.teach_program.stop()
                return {"message": "已发送示教程序停止命令"}
            # 以下为 URDF 选择相关命令：目录授权、扫描、选择。
            if action == "add_urdf_search_root":
                resolved = app.urdf_library.add_search_root(
                    str(command["directory"])
                )
                return {"message": f"已添加 URDF 目录：{resolved}"}
            if action == "remove_urdf_search_root":
                app.urdf_library.remove_search_root(str(command["directory"]))
                return {"message": "已移除 URDF 目录"}
            if action == "refresh_urdf_library":
                entries = app.urdf_library.refresh()
                return {"message": f"已扫描到 {len(entries)} 个 URDF 文件"}
            if action == "select_urdf":
                resolved = app.urdf_library.select(str(command["path"]))
                return {
                    "message": app.urdf_library.status,
                    "path": str(resolved),
                }
            raise ValueError(f"未知命令：{action}")
