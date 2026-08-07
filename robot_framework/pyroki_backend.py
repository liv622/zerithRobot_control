"""PyRoki implementation of the generic solver interface.

This module is imported lazily. The small SciPy backend remains available for
headless installation checks when the optional visualization stack is absent.
"""

from __future__ import annotations

import jax
import jax.numpy as jnp
import jax_dataclasses as jdc
import jaxlie
import jaxls
import numpy as np
import pyroki as pk
import yourdfpy
from scipy.spatial.transform import Rotation

from .model_protocol import RobotModelProtocol
from .solver import IKSolution


def _velocity_limit_residual(
    vals: jaxls.VarValues,
    robot: pk.Robot,
    joint_var: jaxls.Var[jax.Array],
    prev_cfg: jax.Array,
    dt: float,
    weight: float,
) -> jax.Array:
    """Per-step joint velocity limit vs. a fixed previous configuration.

    Mirrors pyroki's ``limit_velocity_residual`` but takes the previous
    configuration as a constant instead of a second variable.  The installed
    jaxls version does not support variables that appear in costs yet are
    absent from the optimization variable list, so a second (fixed) variable
    would silently get a tangent slot and be optimized.
    """
    joint_vel = (vals[joint_var] - prev_cfg) / dt
    residual = jnp.maximum(
        0.0, jnp.abs(joint_vel) - robot.joints.velocity_limits
    )
    return (residual * weight).flatten()


velocity_limit_constraint = jaxls.Cost.factory(kind="constraint_leq_zero")(
    _velocity_limit_residual
)


@jdc.jit
def _solve_masked(
    robot: pk.Robot,
    target_link_index: jax.Array,
    target_wxyz: jax.Array,
    target_position: jax.Array,
    joint_mask: jax.Array,
    rest_cfg: jax.Array,
    fixed_prev_cfg: jax.Array,
    guide_cfg: jax.Array,
    guide_weight: jax.Array,
    orientation_weight: jax.Array,
    smooth_weight: jax.Array,
    velocity_dt: jdc.Static[float],
    velocity_weight: jdc.Static[float],
    manip_weight: jdc.Static[float],
) -> jax.Array:
    joint_var = robot.joint_var_cls(0)
    target_pose = jaxlie.SE3.from_rotation_and_translation(
        jaxlie.SO3(target_wxyz), target_position
    )
    costs = [
        pk.costs.pose_cost_analytic_jac(
            robot=robot,
            joint_var=joint_var,
            target_pose=target_pose,
            target_link_index=target_link_index,
            pos_weight=50.0,
            ori_weight=orientation_weight,
            joint_mask=joint_mask,
        ),
        # Temporal smoothness: pull the solution toward the configuration
        # this solve started from, so consecutive IK solutions do not flip
        # between branches.  The pose cost above still dominates whenever the
        # TCP is away from the target.
        pk.costs.rest_cost(
            joint_var=joint_var, rest_pose=rest_cfg, weight=smooth_weight
        ),
        pk.costs.rest_cost(
            joint_var=joint_var,
            rest_pose=guide_cfg,
            weight=guide_weight * joint_mask,
        ),
        pk.costs.limit_constraint(robot=robot, joint_var=joint_var),
    ]
    if velocity_weight > 0.0:
        # Hard per-step joint velocity limit vs. the configuration before this
        # solve: |q - q_prev| / dt <= velocity_limits (augmented Lagrangian).
        costs.append(
            velocity_limit_constraint(
                robot=robot,
                joint_var=joint_var,
                prev_cfg=fixed_prev_cfg,
                dt=velocity_dt,
                weight=velocity_weight,
            )
        )
    if manip_weight > 0.0:
        # Penalize low translation manipulability at the TCP, steering the
        # solution away from singularities that cause joint flips.
        costs.append(
            pk.costs.manipulability_cost(
                robot=robot,
                joint_var=joint_var,
                target_link_indices=target_link_index,
                weight=manip_weight,
            )
        )
    solution = (
        jaxls.LeastSquaresProblem(costs=costs, variables=[joint_var])
        .analyze()
        .solve(
            verbose=False,
            linear_solver="dense_cholesky",
            trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
            initial_vals=jaxls.VarValues.make(
                [joint_var.with_value(rest_cfg)]
            ),
        )
    )
    return solution[joint_var]


class PyRokiIKSolver:
    """Arm-only PyRoki IK; every non-arm Jacobian column is exactly zero."""

    backend_name = "PyRoki"

    def __init__(self, model: RobotModelProtocol) -> None:
        if model.urdf_path is None:
            raise ValueError("PyRoki requires a URDF-backed RobotModel")
        self.model = model
        urdf = yourdfpy.URDF.load(str(model.urdf_path))
        self.robot = pk.Robot.from_urdf(urdf)
        self.names = tuple(self.robot.joints.actuated_names)
        self.arm_indices = np.array(
            [self.names.index(name) for name in model.arm_joint_names],
            dtype=int,
        )
        self.mask = np.array(
            [
                1.0 if name in model.arm_joint_names else 0.0
                for name in self.names
            ],
            dtype=np.float32,
        )
        self.target_index = jnp.asarray(
            self.robot.links.names.index(model.tcp_link_name),
            dtype=jnp.int32,
        )
        self.lower = np.asarray(self.robot.joints.lower_limits, dtype=float)
        self.upper = np.asarray(self.robot.joints.upper_limits, dtype=float)
        self.last_good: np.ndarray | None = None
        self.batch_index = 0

    def _full_configuration(
        self, arm: np.ndarray, aux: dict[str, float]
    ) -> np.ndarray:
        values = dict(self.model.initial_configuration)
        values.update(aux)
        values.update(
            {
                name: value
                for name, value in zip(self.model.arm_joint_names, arm)
            }
        )
        return np.array([values.get(name, 0.0) for name in self.names], dtype=float)

    def solve(
        self,
        target: np.ndarray,
        seed: np.ndarray,
        aux: dict[str, float],
        *,
        lock_orientation: bool = True,
        guide: np.ndarray | None = None,
        guide_strength: float = 0.05,
        multi_start: bool = True,
        recovery_seeds: int = 10,
        force_recovery: bool = False,
        smooth_strength: float = 0.3,
        velocity_limit_dt: float | None = None,
        manipulability_weight: float = 0.0,
    ) -> IKSolution:
        seed = np.clip(np.asarray(seed), self.model.lower, self.model.upper)
        full_seed = self._full_configuration(seed, aux)
        guide_arm = seed if guide is None else np.asarray(guide, dtype=float)
        full_guide = self._full_configuration(guide_arm, aux)
        xyzw = Rotation.from_matrix(target[:3, :3]).as_quat()
        target_wxyz = np.array([xyzw[3], xyzw[0], xyzw[1], xyzw[2]])

        def run(start_arm: np.ndarray) -> IKSolution:
            start = self._full_configuration(start_arm, aux)
            solved = np.asarray(
                _solve_masked(
                    robot=self.robot,
                    target_link_index=self.target_index,
                    target_wxyz=jnp.asarray(target_wxyz),
                    target_position=jnp.asarray(target[:3, 3]),
                    joint_mask=jnp.asarray(self.mask),
                    rest_cfg=jnp.asarray(start),
                    # The velocity constraint compares against the
                    # configuration before this solve, not the per-run start.
                    fixed_prev_cfg=jnp.asarray(full_seed),
                    guide_cfg=jnp.asarray(full_guide),
                    guide_weight=jnp.asarray(
                        guide_strength if guide is not None else 0.0
                    ),
                    orientation_weight=jnp.asarray(
                        10.0 if lock_orientation else 0.0
                    ),
                    smooth_weight=jnp.asarray(smooth_strength),
                    velocity_dt=(
                        float(velocity_limit_dt)
                        if velocity_limit_dt is not None
                        else 1.0
                    ),
                    velocity_weight=(
                        1.0 if velocity_limit_dt is not None else 0.0
                    ),
                    manip_weight=float(manipulability_weight),
                )
            ).copy()
            # This assignment enforces the hard-lock contract even if a future
            # optimizer version introduces numerical drift in masked columns.
            solved[self.mask == 0.0] = full_seed[self.mask == 0.0]
            arm = solved[self.arm_indices]
            actual = self.model.tcp_pose(arm, aux)
            position, orientation = self._pose_errors(actual, target)
            position_norm = float(np.linalg.norm(position))
            orientation_norm = (
                float(np.linalg.norm(orientation)) if lock_orientation else 0.0
            )
            return IKSolution(
                arm=arm,
                position_error_m=position_norm,
                orientation_error_rad=orientation_norm,
                reachable=bool(
                    position_norm <= 0.010
                    and (
                        not lock_orientation
                        or orientation_norm <= np.deg2rad(10.0)
                    )
                ),
                attempts=1,
                recovered=False,
                joint_names=self.model.arm_joint_names,
            )

        best = run(seed)
        strict_failure = best.position_error_m > 0.002 or (
            lock_orientation and best.orientation_error_rad > np.deg2rad(1.0)
        )
        if multi_start and (strict_failure or force_recovery):
            for start in self._recovery_starts(seed, guide_arm, recovery_seeds):
                candidate = run(start)
                attempts = best.attempts + 1
                if self._score(candidate, lock_orientation) < self._score(
                    best, lock_orientation
                ):
                    candidate.recovered = True
                    best = candidate
                best.attempts = attempts
        if best.position_error_m <= 0.002 and (
            not lock_orientation
            or best.orientation_error_rad <= np.deg2rad(1.0)
        ):
            self.last_good = best.arm.copy()
        if force_recovery:
            self.batch_index += 1
        return best

    @staticmethod
    def _pose_errors(
        actual: np.ndarray, target: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        return (
            actual[:3, 3] - target[:3, 3],
            Rotation.from_matrix(
                actual[:3, :3] @ target[:3, :3].T
            ).as_rotvec(),
        )

    @staticmethod
    def _score(solution: IKSolution, lock_orientation: bool) -> float:
        value = solution.position_error_m / 0.002
        if lock_orientation:
            value += solution.orientation_error_rad / np.deg2rad(1.0)
        return value

    def _recovery_starts(
        self, seed: np.ndarray, guide: np.ndarray, count: int
    ) -> list[np.ndarray]:
        starts = [guide, (self.model.lower + self.model.upper) / 2.0]
        if self.last_good is not None:
            starts.insert(0, self.last_good)
        mirrored = seed.copy()
        mirrored[np.arange(0, len(mirrored), 2)] *= -1
        starts.append(np.clip(mirrored, self.model.lower, self.model.upper))
        rng = np.random.default_rng(260714 + self.batch_index)
        starts.extend(
            rng.uniform(self.model.lower, self.model.upper)
            for _ in range(max(0, count))
        )
        return starts

    def assert_aux_unchanged(
        self,
        before: dict[str, float],
        after: dict[str, float],
    ) -> None:
        for name in self.model.aux_joint_names:
            if before[name] != after[name]:
                raise AssertionError(f"IK changed locked auxiliary joint {name}")


def create_solver(model: RobotModelProtocol):
    return PyRokiIKSolver(model)
