"""Browser-based robot teach-pendant interface."""

from .app import run_pendant
from .template import PENDANT_HTML

__all__ = ["PENDANT_HTML", "run_pendant"]
