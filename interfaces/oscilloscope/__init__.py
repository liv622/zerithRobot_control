"""Oscilloscope — real-time joint-state chart module.

Provides a background sampler that continuously polls the robot controller's
joint positions, computes velocity and acceleration via finite differences,
and exposes the latest frame for SSE streaming to browser-based charts.

Usage (in simulator or any control loop)::

    from interfaces.oscilloscope import OscilloscopeService

    scope = OscilloscopeService(joint_count=len(model.arm_joint_names))
    scope.start(get_joints=lambda: controller.arm.copy())
    ...
    # Pass ``scope`` to the command server for SSE, and serve the assets
    # through the pendant gateway.
    ...
    scope.stop()
"""

from .service import OscilloscopeService

__all__ = ["OscilloscopeService"]
