"""Viser robot-simulation interface."""


def run_ui(*args, **kwargs):
    """Import the optional Viser stack only when a scene is actually started."""
    from .viser_app import run_ui as implementation

    return implementation(*args, **kwargs)

__all__ = ["run_ui"]
