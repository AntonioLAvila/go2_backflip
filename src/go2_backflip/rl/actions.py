"""Joint PD position target + reference joint velocity + reference feedforward torque.

This is the Unitree low-level command (q, dq, kp, kd, tau) driven the way tools/mj_track.py drives
it: the policy supplies the position target, the reference tape supplies ``dq`` and ``tau_ff``.
mjlab's ``DcMotorActuator`` sums ``kp (q* - q) + kd (dq* - dq) + tau_ff`` and clips to the
torque-speed envelope, and its command delay buffer delays all three targets together.

With ``offset_mode="reference"`` the position target is ``q_ref[t] + scale * action``, so a zero
action *is* the mj_track controller that already lands the flip; the policy learns a residual.
``"default"`` keeps mjlab's convention (offset = standing pose), in which case the policy has to
produce the whole tuck itself (a +13 action on the thigh at the stock scale).

Frame bookkeeping: after a reset the robot sits at frame ``s`` while ``MotionCommand.time_steps``
already reads ``s + 1`` (the env's command update runs once with dt = 0), and that one-frame lead
persists, so everything indexed at ``time_steps`` -- the ``command`` observation, the position
target, dq_ref and ctrl_ff -- is the frame the robot should reach by the end of the coming 20 ms.
``ff_frame_offset`` / ``target_frame_offset`` let tools/rl_env_check.py test the alternative
(the interval's start frame, ``-1``): both land the reference; ``0`` is kept because it is what
the observation shows the policy.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, Literal, cast

import numpy as np
import torch
from mjlab.envs.mdp.actions import JointPositionAction, JointPositionActionCfg
from mjlab.tasks.tracking.mdp import MotionCommand
from mjlab.utils.lab_api.math import sample_uniform

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv


@dataclass(kw_only=True)
class JointPositionFeedforwardActionCfg(JointPositionActionCfg):
    command_name: str = "motion"
    """The MotionCommand term whose clip supplies q_ref, dq_ref and ctrl_ff."""
    offset_mode: Literal["reference", "default"] = "reference"
    """Position target = action * scale + (q_ref[t] | default joint pos)."""
    ff_scale_range: tuple[float, float] | None = (0.8, 1.2)
    """Per-env multiplier on the feedforward torque, resampled every reset (feedforward
    mismatch is the one model error a torque-level DR does not cover). None = exactly 1."""
    ff_frame_offset: int = 0
    """ctrl_ff / dq_ref are indexed at ``time_steps + ff_frame_offset`` (see module doc)."""
    target_frame_offset: int = 0
    """In "reference" mode the position offset is q_ref at ``time_steps + target_frame_offset``."""
    feedforward_velocity: bool = True
    """Also send dq_ref as the PD velocity target (mj_track does; kd * dq_ref reaches 38 N.m on
    the rear thigh at 19 rad/s, so it is not a detail)."""
    ff_key: str = "ctrl_ff"

    def build(self, env: ManagerBasedRlEnv) -> JointPositionFeedforwardAction:
        return JointPositionFeedforwardAction(self, env)


class JointPositionFeedforwardAction(JointPositionAction):
    """Keeps ``isinstance(..., JointPositionAction)`` so mjlab's ONNX metadata export accepts it."""

    cfg: JointPositionFeedforwardActionCfg

    def __init__(self, cfg: JointPositionFeedforwardActionCfg, env: ManagerBasedRlEnv):
        super().__init__(cfg=cfg, env=env)
        # Commands are built before actions (ManagerBasedRlEnv.load_managers), so the clip the
        # command loaded -- including a --env.commands.motion.motion-file override -- is final.
        self._cmd = cast(MotionCommand, env.command_manager.get_term(cfg.command_name))
        data = np.load(self._cmd.cfg.motion_file)
        if cfg.ff_key not in data or "joint_names" not in data:
            raise KeyError(
                f"{self._cmd.cfg.motion_file} lacks '{cfg.ff_key}' / 'joint_names'; "
                "export it with tools/export_mjlab.py"
            )
        npz_joints = [str(n) for n in data["joint_names"]]
        missing = [n for n in self._target_names if n not in npz_joints]
        if missing:
            raise ValueError(f"reference clip has no feedforward for joints {missing}")
        perm = [npz_joints.index(n) for n in self._target_names]
        ff = np.asarray(data[cfg.ff_key], dtype=np.float32)[:, perm]
        self._ctrl_ff = torch.tensor(ff, device=self.device)
        if self._ctrl_ff.shape[0] != self._cmd.motion.time_step_total:
            raise ValueError("ctrl_ff frame count differs from the motion's")
        # MotionCommand.joint_vel is in entity joint order; assert once so target_ids index it.
        entity_joints = list(self._entity.joint_names)
        if [entity_joints[i] for i in self._target_ids.tolist()] != list(self._target_names):
            raise ValueError("actuated joints are not a prefix-ordered subset of entity joints")
        self._ff_scale = torch.ones(self.num_envs, 1, device=self.device)

    # -- reference frame index for the interval about to be simulated ------------------------

    def _index(self, offset: int) -> torch.Tensor:
        n = self._cmd.motion.time_step_total
        return (self._cmd.time_steps + offset).clamp(0, n - 1)

    def _ff_index(self) -> torch.Tensor:
        return self._index(self.cfg.ff_frame_offset)

    @property
    def feedforward_torque(self) -> torch.Tensor:
        """What is being fed forward right now, per env (N.m), for logging / observations."""
        return self._ff_scale * self._ctrl_ff[self._ff_index()]

    # -- ActionTerm ----------------------------------------------------------------------------

    def process_actions(self, actions: torch.Tensor) -> None:
        if self.cfg.offset_mode == "reference":
            q_ref = self._cmd.motion.joint_pos[self._index(self.cfg.target_frame_offset)]
            self._offset = q_ref[:, self._target_ids]
        super().process_actions(actions)

    def apply_actions(self) -> None:
        super().apply_actions()  # position target (minus encoder bias)
        i = self._ff_index()
        self._entity.set_joint_effort_target(
            self._ff_scale * self._ctrl_ff[i], joint_ids=self._target_ids
        )
        if self.cfg.feedforward_velocity:
            self._entity.set_joint_velocity_target(
                self._cmd.motion.joint_vel[i][:, self._target_ids], joint_ids=self._target_ids
            )

    def reset(self, env_ids: torch.Tensor | slice | None = None) -> None:
        super().reset(env_ids)
        if self.cfg.ff_scale_range is None:
            return
        if env_ids is None:
            env_ids = slice(None)
        n = self.num_envs if isinstance(env_ids, slice) else len(env_ids)
        lo, hi = self.cfg.ff_scale_range
        self._ff_scale[env_ids] = sample_uniform(lo, hi, (n, 1), self.device)
