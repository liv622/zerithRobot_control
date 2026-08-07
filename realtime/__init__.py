"""Real-time layer: pacing, non-blocking delivery and the final safety gate.

The three concerns here are what keep a fixed-rate motion loop honest:

- :mod:`realtime.clock` paces the loop on a non-drifting deadline grid.
- :mod:`realtime.motion_streamer` hands samples to hardware without ever
  blocking or letting a sink exception kill the motion thread.
- :mod:`realtime.safety` re-checks every command against position and rate
  limits, independently of whichever planner produced it.

This layer knows nothing about robot models, transports or UIs, so any control
loop can use it.
"""

from .clock import (
    MAX_FREQUENCY_HZ,
    MIN_FREQUENCY_HZ,
    LoopStatistics,
    PacedLoop,
    validate_frequency,
)
from .monitor import JointMonitor, JointSample, JointStreamFrame
from .motion_streamer import JointSampleSink, MotionStreamer, StreamStatistics
from .safety import (
    GuardedJointSink,
    JointCommandGuard,
    JointSafetyLimits,
    SafetyViolation,
)

__all__ = [
    "MAX_FREQUENCY_HZ",
    "MIN_FREQUENCY_HZ",
    "GuardedJointSink",
    "JointCommandGuard",
    "JointMonitor",
    "JointSafetyLimits",
    "JointSample",
    "JointSampleSink",
    "JointStreamFrame",
    "LoopStatistics",
    "MotionStreamer",
    "PacedLoop",
    "SafetyViolation",
    "StreamStatistics",
    "validate_frequency",
]
