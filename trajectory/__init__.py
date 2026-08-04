"""Robot trajectory planning public API."""

from .cartesian import interpolate_cartesian_segment, plan_cartesian_trajectory
from .joint import plan_joint_trajectory
from .models import JointTrajectory, Trajectory, TrajectoryError
from .profiles import quintic

# Backward-compatible name used by earlier integrations.
interpolate_segment = interpolate_cartesian_segment
plan_trajectory = plan_cartesian_trajectory

__all__ = [
    "JointTrajectory",
    "Trajectory",
    "TrajectoryError",
    "interpolate_cartesian_segment",
    "interpolate_segment",
    "plan_cartesian_trajectory",
    "plan_joint_trajectory",
    "plan_trajectory",
    "quintic",
]
