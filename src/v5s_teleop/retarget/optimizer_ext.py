"""Extension that adds our own cost terms to the upstream DexPilot optimizer.

Two additions: a **self-collision penalty** and **shape matching**. Both are off
by default; a weight of 0 kills the whole code path.

Upstream `dex_retargeting` (MIT) is left unmodified. Here we subclass its public
`DexPilotOptimizer` and add one cost term. No upstream code is copied.

**Why the collision term.** Sampling random postures inside the V5 Sense joint
limits produces 6-10% of poses where neighbouring finger links interpenetrate
(index-middle 5.9%, middle-ring 7.8%, thumb-index 9.7%). The retargeting
optimizer solves for joint angles from fingertip positions alone, so without
this penalty it will happily return such a pose.

**How.** Each link pair gets a minimum separation (its margin), and any
encroachment beyond it is charged as a squared penalty. The margin must stay
below the natural separation of an open hand or the neutral pose gets distorted
(V5 Sense open-hand fingertip separation: index-middle 43.5 mm, middle-ring
41.7 mm).

─────────────────────────────────────────────────────────────────────────
**Shape matching**

All ten DexPilot targets are **fingertip positions** (6 fingertip pairs + 4
wrist-to-fingertip). But a robot finger has four joints against only three
positional constraints, which leaves **one degree of freedom undetermined**.
Measured: more than eight joint combinations produce the same fingertip position
(MCP/PIP/DIP = 50/40/30 works, and so does 65/0/65).

Today that freedom is settled only by upstream's `norm_delta` regularization --
"move as little as possible from the previous solution" -- which has nothing to
do with the shape of the human hand. So when a finger is curled slowly, the
robot converges on a pose where one phalanx bends and the rest stay stiff.

**Match directions instead.** Compare which way each segment (MCP-PIP, PIP-DIP,
DIP-TIP) points against the human. **Unit vectors** rather than positions,
because the robot's segments are not a scaled copy of a human's -- on the index
finger they are 1.01x / 1.59x / 1.91x for proximal / intermediate / distal. At
scale 1.5, matching positions would leave up to 21 mm of structural error per
segment, and that error fights the fingertip targets. Directions make the length
difference irrelevant.

The rest of the cost function (task-space vector matching plus pinch projection
and switching) is upstream's, following
[DexPilot, Handa et al., ICRA 2020 (arXiv:1910.03135), section VI-A].
"""

from __future__ import annotations

import numpy as np
from dex_retargeting.optimizer import DexPilotOptimizer


class SelfCollisionDexPilotOptimizer(DexPilotOptimizer):
    """DexPilot plus a self-collision penalty plus shape matching.

    (The name dates from when self-collision was the only addition; it now also
    carries shape matching.)

    Added cost term:

        C_collision = w * sum_(a,b) max(0, m_ab - ||p_a - p_b||)^2

    `p_a` and `p_b` are link origins in world coordinates and `m_ab` is that
    pair's margin. Using link-origin distance makes this a **conservative
    approximation** rather than mesh-accurate collision; choosing margins to fit
    the mesh geometry absorbs that approximation.
    """

    retargeting_type = "DEXPILOT_EXT"

    def __init__(
        self,
        *args,
        collision_link_pairs: list[tuple[str, str, float]] | None = None,
        collision_weight: float = 0.0,
        shape_segments: list[tuple[str, str, int, int]] | None = None,
        shape_weight: float = 0.0,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)

        self._init_shape(shape_segments, shape_weight)

        self.collision_weight = float(collision_weight)
        self._collision_active = bool(collision_link_pairs) and self.collision_weight > 0.0

        if not self._collision_active:
            self._collision_link_indices: list[int] = []
            return

        # Collect the links appearing in the pairs without duplicates, so FK and
        # the Jacobians are computed once each.
        names: list[str] = []
        for a, b, _ in collision_link_pairs:
            for n in (a, b):
                if n not in names:
                    names.append(n)
        self._collision_link_names = names
        # pinocchio's updateFramePlacement rejects numpy integers (its
        # Boost.Python signature only accepts unsigned long). Always store
        # these as plain Python ints.
        self._collision_link_indices = [int(i) for i in self.get_link_indices(names)]

        pos = {n: i for i, n in enumerate(names)}
        self._pair_a = np.array([pos[a] for a, _, _ in collision_link_pairs], dtype=int)
        self._pair_b = np.array([pos[b] for _, b, _ in collision_link_pairs], dtype=int)
        self._pair_margin = np.array([float(m) for _, _, m in collision_link_pairs])

        if np.any(self._pair_margin <= 0):
            raise ValueError("every self-collision margin must be positive")

    def _init_shape(
        self,
        shape_segments: list[tuple[str, str, int, int]] | None,
        shape_weight: float,
    ) -> None:
        """Prepare shape matching.

        Args:
            shape_segments: `(robot_start_link, robot_end_link,
                human_start_point, human_end_point)`. Human points are MANO 21
                indices (index MCP=5, PIP=6, DIP=7, TIP=8, ...).
            shape_weight: 0 turns the term off. **It can be changed later** --
                the value has to be swept on real hardware, so the link indices
                are always prepared regardless of the weight (they are just name
                lookups, so there is no cost).
        """
        self.shape_weight = float(shape_weight)
        self._shape_configured = bool(shape_segments)
        if not self._shape_configured:
            self._shape_link_indices: list[int] = []
            return

        names: list[str] = []
        for a, b, _, _ in shape_segments:
            for n in (a, b):
                if n not in names:
                    names.append(n)
        # Plain Python ints for the same reason as the collision term
        # (pinocchio Boost.Python).
        self._shape_link_indices = [int(i) for i in self.get_link_indices(names)]
        pos = {n: i for i, n in enumerate(names)}
        self._shape_a = np.array([pos[a] for a, _, _, _ in shape_segments], dtype=int)
        self._shape_b = np.array([pos[b] for _, b, _, _ in shape_segments], dtype=int)
        self._shape_hu_a = np.array([ha for _, _, ha, _ in shape_segments], dtype=int)
        self._shape_hu_b = np.array([hb for _, _, _, hb in shape_segments], dtype=int)

    @property
    def _shape_active(self) -> bool:
        """Whether to evaluate the shape term this frame. Dropping the weight to
        0 on real hardware disables it immediately, and raising it enables it
        immediately -- no rebuild needed."""
        return self._shape_configured and self.shape_weight > 0.0

    def set_human_keypoints(self, keypoints: np.ndarray) -> None:
        """Supply the 21 human hand points (in the MANO frame). The shape
        matching target directions are built here.

        Upstream `retarget()` only receives ten vectors, so it cannot know the
        intermediate joint positions. They are passed in separately each frame
        through this method. Does nothing when shape matching is off.

        Raises:
            ValueError: if the array is not (21, 3).
        """
        if not self._shape_active:
            return
        kp = np.asarray(keypoints, dtype=np.float64)
        if kp.shape != (21, 3):
            raise ValueError(f"human keypoints must be (21, 3), got {kp.shape}")
        # Only **direction** is used, not length -- the robot's segments are not
        # a scaled copy of a human's, so matching length too would fight the
        # fingertip targets (see the module docstring).
        self._shape_target, _ = self._unit(kp[self._shape_hu_b] - kp[self._shape_hu_a])

    @staticmethod
    def _unit(v: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Row-wise unit vectors and their original lengths, guarding against
        near-zero lengths."""
        n = np.linalg.norm(v, axis=1)
        return v / (n[:, None] + 1e-9), n

    def get_objective_function(
        self, target_vector: np.ndarray, fixed_qpos: np.ndarray, last_qpos: np.ndarray
    ):
        base_objective = super().get_objective_function(target_vector, fixed_qpos, last_qpos)
        if not self._collision_active and not self._shape_active:
            return base_objective
        if self._shape_active and not hasattr(self, "_shape_target"):
            raise RuntimeError(
                "shape matching is on but no human keypoints were supplied -- "
                "call optimizer.set_human_keypoints(mano21) before retarget()."
            )

        qpos = np.zeros(self.num_joints)
        qpos[self.idx_pin2fixed] = fixed_qpos

        def objective(x: np.ndarray, grad: np.ndarray) -> float:
            # Evaluate the upstream term first; it fills grad with the upstream
            # gradient.
            value = base_objective(x, grad)

            # Rebuild qpos the same way upstream does (upstream's qpos is a local
            # inside its own closure).
            qpos[self.idx_pin2target] = x
            if self.adaptor is not None:
                qpos[:] = self.adaptor.forward_qpos(qpos)[:]

            # Upstream just ran FK on this same qpos, but we do not rely on that
            # and run it explicitly again -- so this term does not go silently
            # wrong if upstream's internals change.
            self.robot.compute_forward_kinematics(qpos)

            # ── Shape matching: align segment directions with the human ────
            if self._shape_active:
                s_poses = [self.robot.get_link_pose(i) for i in self._shape_link_indices]
                s_pos = np.array([p[:3, 3] for p in s_poses])
                s_vec = s_pos[self._shape_b] - s_pos[self._shape_a]      # (S, 3)
                s_unit, s_len = self._unit(s_vec)
                s_err = s_unit - self._shape_target                      # (S, 3)
                value += self.shape_weight * float((s_err**2).sum())

                if grad.size > 0:
                    # d/dv ‖u(v) − t‖²  where u = v/‖v‖
                    #   = 2·(I − u uᵀ)/‖v‖ · (u − t)
                    proj = s_err - s_unit * (s_unit * s_err).sum(axis=1)[:, None]
                    coeff = (2.0 * self.shape_weight) * proj / (s_len[:, None] + 1e-9)

                    s_grad_pos = np.zeros_like(s_pos)
                    np.add.at(s_grad_pos, self._shape_b, coeff)
                    np.add.at(s_grad_pos, self._shape_a, -coeff)

                    s_jac = []
                    for i, link_index in enumerate(self._shape_link_indices):
                        lj = self.robot.compute_single_link_local_jacobian(qpos, link_index)[:3, ...]
                        s_jac.append(s_poses[i][:3, :3] @ lj)
                    s_jac = np.stack(s_jac, axis=0)
                    if self.adaptor is not None:
                        s_jac = self.adaptor.backward_jacobian(s_jac)
                    else:
                        s_jac = s_jac[..., self.idx_pin2target]
                    grad[:] += np.einsum("li,lij->j", s_grad_pos, s_jac)

            if not self._collision_active:
                return value

            # ── Self-collision penalty ─────────────────────────────────────
            poses = [self.robot.get_link_pose(i) for i in self._collision_link_indices]
            body_pos = np.array([p[:3, 3] for p in poses])

            diff = body_pos[self._pair_a] - body_pos[self._pair_b]     # (P, 3)
            dist = np.linalg.norm(diff, axis=1)                         # (P,)
            penetration = np.maximum(0.0, self._pair_margin - dist)     # (P,)
            value += self.collision_weight * float((penetration**2).sum())

            if grad.size > 0:
                # d/dp_a [w·pen²] = -2·w·pen·û,  û = (p_a - p_b)/‖·‖
                # The direction is undefined as dist approaches 0, so guard it.
                unit = diff / (dist[:, None] + 1e-9)
                coeff = (-2.0 * self.collision_weight * penetration)[:, None] * unit

                grad_pos = np.zeros_like(body_pos)
                np.add.at(grad_pos, self._pair_a, coeff)
                np.add.at(grad_pos, self._pair_b, -coeff)

                jacobians = []
                for i, link_index in enumerate(self._collision_link_indices):
                    local_j = self.robot.compute_single_link_local_jacobian(qpos, link_index)[:3, ...]
                    jacobians.append(poses[i][:3, :3] @ local_j)
                jacobians = np.stack(jacobians, axis=0)

                # Move the Jacobian's joint ordering from pinocchio order to the
                # optimization target order.
                if self.adaptor is not None:
                    jacobians = self.adaptor.backward_jacobian(jacobians)
                else:
                    jacobians = jacobians[..., self.idx_pin2target]

                grad[:] += np.einsum("li,lij->j", grad_pos, jacobians)

            return value

        return objective
