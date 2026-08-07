"""Logging layer for the robot framework.

Deliberately named ``robot_logging`` rather than ``logging`` so it can never
shadow the standard library module for any importer of this project.

Two distinct concerns live here:

- :mod:`robot_logging.setup` configures human/operator facing log output and
  hands out named loggers to every other layer.
- :mod:`robot_logging.audit` records an append-only machine-readable trace of
  motion decisions, which is what you actually need after an incident.

Nothing in this package imports application, trajectory, transport or robot
model code, so every layer is free to depend on it.
"""

from .audit import MotionAuditLog, NullMotionAuditLog
from .setup import configure_logging, get_logger

__all__ = [
    "MotionAuditLog",
    "NullMotionAuditLog",
    "configure_logging",
    "get_logger",
]
