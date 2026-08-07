"""Registry of robot-model plugins available to the application."""

from __future__ import annotations

from robot_framework.plugin import RobotPlugin

from .e1pro.plugin import E1PRO_PLUGIN
from .e1pro_dual.plugin import DUAL_ARM_PLUGIN


_PLUGINS = {
    E1PRO_PLUGIN.key: E1PRO_PLUGIN,
    DUAL_ARM_PLUGIN.key: DUAL_ARM_PLUGIN,
}


def available_robot_keys() -> tuple[str, ...]:
    return tuple(sorted(_PLUGINS))


def get_robot_plugin(key: str) -> RobotPlugin:
    try:
        return _PLUGINS[key]
    except KeyError as exc:
        choices = ", ".join(available_robot_keys())
        raise ValueError(f"未知机器人型号 {key!r}；可用型号：{choices}") from exc

