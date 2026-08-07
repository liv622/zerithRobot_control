"""Process-wide logging configuration for every robot framework layer."""

from __future__ import annotations

import logging
import logging.handlers
import os
import sys
from pathlib import Path

ROOT_LOGGER_NAME = "robot"

_LEVELS = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "CRITICAL": logging.CRITICAL,
}

# Console output is what an operator reads while teaching, so it stays short.
# The rotating file keeps the full context for post-incident analysis.
_CONSOLE_FORMAT = "%(levelname)-7s %(name)s | %(message)s"
_FILE_FORMAT = (
    "%(asctime)s %(levelname)-7s %(name)s [%(threadName)s] "
    "%(filename)s:%(lineno)d | %(message)s"
)

_configured = False


def get_logger(name: str) -> logging.Logger:
    """Return the layer logger for ``name``.

    ``name`` is a dotted suffix such as ``"trajectory.double_s"``; the
    ``robot.`` prefix is applied here so all framework logs share one root and
    can be silenced or redirected in a single place.
    """
    suffix = name.strip().strip(".")
    if not suffix or suffix == ROOT_LOGGER_NAME:
        return logging.getLogger(ROOT_LOGGER_NAME)
    if suffix.startswith(f"{ROOT_LOGGER_NAME}."):
        return logging.getLogger(suffix)
    return logging.getLogger(f"{ROOT_LOGGER_NAME}.{suffix}")


def _resolve_level(level: str | int | None) -> int:
    if isinstance(level, int):
        return level
    text = (level or os.environ.get("ROBOT_LOG_LEVEL") or "INFO").upper()
    return _LEVELS.get(text, logging.INFO)


def configure_logging(
    *,
    level: str | int | None = None,
    log_directory: Path | None = None,
    file_name: str = "robot.log",
    console: bool = True,
    max_bytes: int = 4 * 1024 * 1024,
    backup_count: int = 5,
    force: bool = False,
) -> logging.Logger:
    """Install console and rotating-file handlers on the framework root logger.

    Safe to call more than once: repeated calls are ignored unless ``force``
    is set, so an entrypoint can configure logging without worrying about
    whether a test or an embedding application already did.
    """
    global _configured
    root = logging.getLogger(ROOT_LOGGER_NAME)
    if _configured and not force:
        return root
    for handler in list(root.handlers):
        root.removeHandler(handler)
        handler.close()

    resolved = _resolve_level(level)
    root.setLevel(resolved)
    # The framework root is the only place handlers are attached, so records
    # must not also travel to the stdlib root logger and be printed twice.
    root.propagate = False

    if console:
        stream = logging.StreamHandler(sys.stderr)
        stream.setLevel(resolved)
        stream.setFormatter(logging.Formatter(_CONSOLE_FORMAT))
        root.addHandler(stream)

    if log_directory is not None:
        directory = Path(log_directory)
        directory.mkdir(parents=True, exist_ok=True)
        rotating = logging.handlers.RotatingFileHandler(
            directory / file_name,
            maxBytes=max_bytes,
            backupCount=backup_count,
            encoding="utf-8",
        )
        rotating.setLevel(resolved)
        rotating.setFormatter(logging.Formatter(_FILE_FORMAT))
        root.addHandler(rotating)

    if not root.handlers:
        root.addHandler(logging.NullHandler())
    _configured = True
    return root
