"""Build the mjlab Go2 backflip environment and replay the reference through it.

The mjlab-side counterpart of tools/mj_track.py. That script proves the reference lands under
``ctrl_ff + PD`` in plain MuJoCo; this one proves the *RL environment* -- mjlab's DcMotor
actuators, delay buffers, contact sensors, motion command and terminations, with domain
randomisation and noise switched off -- delivers the same controller when the policy outputs zero
(the action is a residual on the reference). If this does not land, training starts from a
broken baseline and the env config is wrong, not the policy.

    uv run tools/rl_env_check.py                 # build, print managers, one zero-action step
    uv run tools/rl_env_check.py --replay        # roll the reference to its end and score it
    uv run tools/rl_env_check.py --replay --ff-frame-offset 0 --action-noise 0.3 --view

Exit status 0 iff the build succeeds and, with --replay, every env reaches the end of the clip
standing (rotated ~360 deg, tilt < 15 deg, base above 0.20 m, legs at the reference) with no
early termination -- mj_track's "landed". "Clean" (no non-foot geom ever touches the floor) is
reported separately, as mj_track does; at nominal it is marginal (under the 50 Hz hold the head
sphere strikes the floor for ~2 steps at touchdown, ~3 kN peak; 2 ms of latency decides it in
plain MuJoCo too).
"""

from __future__ import annotations

import argparse
import sys

import mjlab  # noqa: F401  (loads the mjlab.tasks entry points, including go2_backflip.rl)
import numpy as np
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
from mjlab.tasks.tracking.mdp import MotionCommand, compute_mpkpe

import go2_backflip.rl  # noqa: F401  (belt and braces: register even if the entry point is stale)
from go2_backflip.rl import TASK_ID
from go2_backflip.rl.actions import JointPositionFeedforwardActionCfg


def pitch_unwrapped(quat_wxyz: np.ndarray) -> np.ndarray:
    """Base pitch about world y, unwrapped, from the body x axis (as in tools/mj_track.py)."""
    w, x, y, z = quat_wxyz.T
    return np.unwrap(np.arctan2(-2 * (x * z - w * y), 1 - 2 * (y * y + z * z)))


def build(args) -> ManagerBasedRlEnv:
    cfg = load_env_cfg(args.task, play=True)
    cfg.scene.num_envs = args.num_envs
    act = cfg.actions["joint_pos"]
    assert isinstance(act, JointPositionFeedforwardActionCfg)
    act.ff_frame_offset = args.ff_frame_offset
    act.target_frame_offset = args.target_frame_offset
    act.ff_scale_range = (args.ff_scale, args.ff_scale)
    if args.no_ff:
        act.ff_scale_range = (0.0, 0.0)
        act.feedforward_velocity = False
    return ManagerBasedRlEnv(cfg, device=args.device)


def replay(env: ManagerBasedRlEnv, args) -> bool:
    robot = env.scene["robot"]
    cmd = env.command_manager.get_term("motion")
    assert isinstance(cmd, MotionCommand)
    stray = env.scene["stray_ground"]
    gen = torch.Generator(device=env.device).manual_seed(0)
    n_act = env.action_manager.total_action_dim

    env.reset()
    log: dict[str, list] = {k: [] for k in ("z", "quat", "stray", "stray_f", "mpkpe", "q")}
    ended_by, steps = None, 0
    for _ in range(cmd.motion.time_step_total + 5):
        action = torch.zeros(env.num_envs, n_act, device=env.device)
        if args.action_noise > 0:
            action += args.action_noise * torch.randn(action.shape, device=env.device, generator=gen)
        _, _, terminated, truncated, _ = env.step(action)
        steps += 1
        log["z"].append(robot.data.root_link_pos_w[:, 2].cpu().numpy())
        log["quat"].append(robot.data.root_link_quat_w.cpu().numpy())
        log["stray"].append((stray.data.found > 0).cpu().numpy())  # [N, P]
        log["stray_f"].append(stray.data.force.norm(dim=-1).cpu().numpy())  # [N, P]
        log["mpkpe"].append(compute_mpkpe(cmd).cpu().numpy())
        log["q"].append(robot.data.joint_pos.cpu().numpy())
        if terminated.any() or truncated.any():
            active = {n: int(env.termination_manager.get_term(n).sum()) for n in env.termination_manager.active_terms}
            ended_by = {n: v for n, v in active.items() if v > 0}
            break

    z = np.stack(log["z"])                      # [T, N]
    quat = np.stack(log["quat"])                # [T, N, 4]
    stray_all = np.stack(log["stray"])          # [T, N, P]
    stray_hits = stray_all.any(0).any(-1)       # [N]
    stray_force = np.stack(log["stray_f"])      # [T, N, P]
    mpkpe = np.stack(log["mpkpe"])              # [T, N]
    # step() auto-resets done envs before returning, so the entry logged on the final step is
    # post-reset; score the last pre-reset frame instead.
    k = max(steps - 2, 0)
    rot = np.array([np.degrees(pitch_unwrapped(quat[: k + 1, i])[-1] - pitch_unwrapped(quat[: k + 1, i])[0])
                    for i in range(env.num_envs)])
    _, x, y, _ = quat[k].T
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (x * x + y * y), -1, 1)))
    height = z[k]
    q_ref_end = cmd.motion.joint_pos[-1].cpu().numpy()
    legs_err = np.abs(np.stack(log["q"])[k] - q_ref_end).max(-1)

    finished = ended_by is not None and set(ended_by) == {"motion_finished"} and ended_by["motion_finished"] == env.num_envs
    landed = (np.abs(np.abs(rot) - 360) < 25) & (tilt < 15) & (height > 0.20) & (legs_err < 0.35)
    clean = not stray_hits.any()
    ok = bool(finished and landed.all())

    print(f"\nreplay: {steps} steps, ended by {ended_by}")
    print(f"  rotation  deg   {np.round(rot, 1)}")
    print(f"  final tilt deg  {np.round(tilt, 1)}")
    print(f"  final height m  {np.round(height, 3)}   apex {z.max():.3f}")
    print(f"  legs err rad    {np.round(legs_err, 3)}")
    print(f"  mpkpe max/mean  {mpkpe.max():.3f} / {mpkpe.mean():.3f} m")
    probe = [i for i in (0, 25, 50, 70, 80, 90, 100, 120, 168) if i < steps]
    print("  mpkpe by step   " + "  ".join(f"{i}:{mpkpe[i, 0]:.3f}" for i in probe))
    print(f"  stray contact   {stray_hits}")
    for pi, name in enumerate(stray.primary_names):
        hit_steps = np.nonzero(stray_all[:, :, pi].any(-1))[0]
        if hit_steps.size:
            fmax = stray_force[:, :, pi].max()
            print(f"      {name}: steps {hit_steps.min()}-{hit_steps.max()} ({hit_steps.size} steps), max force {fmax:.1f} N")
    print(f"  => {'LANDED' if ok else 'FAILED'}{' CLEAN' if ok and clean else ''}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--task", default=TASK_ID)
    ap.add_argument("--num-envs", type=int, default=4)
    ap.add_argument("--device", default="cuda:0" if torch.cuda.is_available() else "cpu")
    ap.add_argument("--replay", action="store_true", help="roll the reference with zero actions and score it")
    ap.add_argument("--action-noise", type=float, default=0.0, help="std of Gaussian noise on the (zero) action")
    ap.add_argument("--ff-scale", type=float, default=1.0)
    ap.add_argument("--ff-frame-offset", type=int, default=0)
    ap.add_argument("--target-frame-offset", type=int, default=0)
    ap.add_argument("--no-ff", action="store_true", help="drop feedforward torque and velocity (PD only)")
    ap.add_argument("--view", action="store_true", help="interactive viewer with the zero policy")
    args = ap.parse_args()

    env = build(args)
    print(env.observation_manager)
    print(env.action_manager)
    print(env.reward_manager)
    print(env.termination_manager)
    print(env.event_manager)
    obs, _ = env.reset()
    actor = obs["actor"]
    print(f"actor obs {tuple(actor.shape)}, critic obs {tuple(obs['critic'].shape)}, "
          f"actions {env.action_manager.total_action_dim}, step_dt {env.step_dt:.3f} s")
    obs, rew, _, _, _ = env.step(torch.zeros(env.num_envs, env.action_manager.total_action_dim, device=env.device))
    assert torch.isfinite(obs["actor"]).all() and torch.isfinite(rew).all(), "NaN after one step"
    print(f"one zero-action step ok: reward {rew.cpu().numpy().round(4)}")

    if args.view:
        from mjlab.viewer import NativeMujocoViewer

        n_act = env.action_manager.total_action_dim
        NativeMujocoViewer(env, lambda o: torch.zeros(env.num_envs, n_act, device=env.device)).run()
        return 0
    if args.replay:
        return 0 if replay(env, args) else 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
