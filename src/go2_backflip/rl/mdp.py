"""MDP terms the stock tracking task lacks: a clip-end truncation, gravity randomisation, and a
privileged contact-schedule observation."""

from __future__ import annotations

from typing import TYPE_CHECKING, cast

import numpy as np
import torch
from mjlab.entity import Entity
from mjlab.envs.mdp.events import resolve_env_ids
from mjlab.managers.event_manager import requires_model_fields
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.tracking.mdp import MotionCommand
from mjlab.utils.lab_api.math import sample_uniform

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnv

_ROBOT = SceneEntityCfg("robot")


def motion_finished(env: ManagerBasedRlEnv, command_name: str = "motion") -> torch.Tensor:
    """True on the clip's last frame. Register with ``time_out=True``.

    A backflip is not cyclic: without this, ``MotionCommand`` wraps around at the end of the clip
    and teleports the robot to a random frame mid-episode. Terminations are evaluated before the
    command update in ``ManagerBasedRlEnv.step``, so this fires first. As a time-out it is a
    truncation: no failure penalty, and the adaptive frame sampler does not count it as a fall.
    """
    cmd = cast(MotionCommand, env.command_manager.get_term(command_name))
    return cmd.time_steps >= cmd.motion.time_step_total - 1


@requires_model_fields("body_mass")
def gravity_offset(
    env: ManagerBasedRlEnv,
    env_ids: torch.Tensor | None,
    g_range: tuple[float, float] = (-0.5, 0.5),
    tilt_range: tuple[float, float] = (-0.3, 0.3),
    asset_cfg: SceneEntityCfg = _ROBOT,
) -> None:
    """Gravity randomisation, emulated. Use with ``mode="reset"``.

    mjlab cannot batch ``opt.gravity`` per world, but a gravity perturbation dg is physically the
    constant world-frame force ``m_b * dg`` on every body, which ``xfrc_applied`` provides and
    which persists for the episode (``Entity.reset`` does not clear it; the next reset overwrites
    it). ``g_range`` is the vertical component, ``tilt_range`` the x/y components, in m/s^2.
    Reads the (possibly randomised) per-world masses, so it composes with ``pseudo_inertia``.
    Do not combine with another wrench-writing event on the same bodies.
    """
    env_ids = resolve_env_ids(env, env_ids)
    asset: Entity = env.scene[asset_cfg.name]
    body_ids = asset.indexing.body_ids
    mass = env.sim.model.body_mass
    if mass.ndim == 1:
        m = mass[body_ids].unsqueeze(0).expand(len(env_ids), -1)
    else:
        m = mass[env_ids.long()[:, None], body_ids[None, :]]
    n = len(env_ids)
    dg = torch.stack(
        [
            sample_uniform(*tilt_range, (n,), env.device),
            sample_uniform(*tilt_range, (n,), env.device),
            sample_uniform(*g_range, (n,), env.device),
        ],
        dim=-1,
    )
    forces = m.to(torch.float32)[..., None] * dg[:, None, :]
    asset.write_external_wrench_to_sim(forces, torch.zeros_like(forces), env_ids=env_ids)


class reference_contact:
    """The clip's planned foot-contact flags at the current frame (privileged, critic-only).

    Class-based so the npz is read once; mjlab instantiates class terms with ``(cfg, env)``.
    """

    def __init__(self, cfg: ObservationTermCfg, env: ManagerBasedRlEnv):
        cmd = cast(MotionCommand, env.command_manager.get_term(cfg.params.get("command_name", "motion")))
        data = np.load(cmd.cfg.motion_file)
        contact = np.asarray(data["contact"], dtype=np.float32)
        if contact.shape[0] != cmd.motion.time_step_total:
            raise ValueError("contact frame count differs from the motion's")
        self._contact = torch.tensor(contact, device=env.device)
        self._cmd = cmd

    def __call__(self, env: ManagerBasedRlEnv, command_name: str = "motion") -> torch.Tensor:
        del env, command_name
        return self._contact[self._cmd.time_steps]
