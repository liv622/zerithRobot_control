"""Robot trajectory planning layer.

Both joint-space (MOVJ) and Cartesian (MOVL) planning use a double-S
(jerk-limited S-curve) velocity profile by default, so commanded velocity and
acceleration are continuous.  See :mod:`trajectory.double_s` for the two
interchangeable backends and :mod:`trajectory.limits` for how operator speed
settings become velocity/acceleration/jerk ceilings.

This layer depends only on NumPy/SciPy, the IK solver protocol and the logging
layer.  It must never import UI, transport or robot-model specific code.
"""

from .cartesian import (
    interpolate_cartesian_segment,
    plan_cartesian_path_profile,
    plan_cartesian_trajectory,
    pose_from_position_and_rotation,
    segment_distance_and_angle,
)
from .double_s import (
    DoubleSProfile,
    ProfileSample,
    double_s_backend_name,
    plan_double_s,
    sample_double_s,
)
from .joint import joint_limits_from_speed, plan_joint_trajectory
from .limits import CartesianLimits, DoubleSLimits
from .models import JointTrajectory, Trajectory, TrajectoryError

# Backward-compatible names used by earlier integrations.
interpolate_segment = interpolate_cartesian_segment
plan_trajectory = plan_cartesian_trajectory

__all__ = [
    "CartesianLimits",
    "DoubleSLimits",
    "DoubleSProfile",
    "JointTrajectory",
    "ProfileSample",
    "Trajectory",
    "TrajectoryError",
    "double_s_backend_name",
    "interpolate_cartesian_segment",
    "interpolate_segment",
    "joint_limits_from_speed",
    "plan_cartesian_path_profile",
    "plan_cartesian_trajectory",
    "plan_double_s",
    "plan_joint_trajectory",
    "plan_trajectory",
    "pose_from_position_and_rotation",
    "sample_double_s",
    "segment_distance_and_angle",
]
