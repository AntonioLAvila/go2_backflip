"""Closed-loop MuJoCo rollout of a reference: feedforward torque + joint PD. The acceptance test.

The audit asks whether a trajectory is consistent with the model it was solved on;
mj_divergence.py asks how fast the open-loop tape diverges in MuJoCo, which for a contact-rich
unstable motion is "immediately" and says little. Neither answers the question the next stage
cares about: *if a controller as dumb as a joint PD follows this reference through MuJoCo's own
contact model and the hardware's own torque-speed envelope, does the robot do a backflip and
stay standing?* If yes, the RL stage starts from a reference that already nearly works and only
has to learn robustness; if no, it has to learn the maneuver despite its reference.

    uv run tools/mj_track.py [traj_opt/reference/backflip.npz] [--kp 60 --kd 2] [--view]

Exit status 0 iff the flip lands and holds. Perturbation flags (--mass-scale, --friction,
--latency-steps, --push) give a cheap robustness sweep with no learning involved.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import mujoco
import numpy as np

from go2_backflip import constants as K

REF = Path(K.REPO_ROOT) / "traj_opt" / "reference" / "backflip.npz"
SCENE = str(Path(K.MODEL_PATH).parent / "scene.xml")
FOOT_GEOMS = set(K.FEET)


def hardware_clip(tau: np.ndarray, qd: np.ndarray) -> np.ndarray:
    """What the motor will actually deliver: datasheet peak and the torque-speed halfplanes."""
    pk = K.hardware_torque_limits()
    k, stall = K.torque_speed_halfplanes(pk)
    return np.clip(tau, np.maximum(-pk, -stall - k * qd), np.minimum(pk, stall - k * qd))


def pitch_unwrapped(quat_wxyz: np.ndarray) -> np.ndarray:
    """Base pitch about world y, unwrapped, from the body x axis (valid for sagittal motion)."""
    w, x, y, z = quat_wxyz.T
    bx_x = 1 - 2 * (y * y + z * z)
    bx_z = 2 * (x * z - w * y)
    return np.unwrap(np.arctan2(-bx_z, bx_x))


def rollout(ref, kp, kd, hold=1.0, mass_scale=1.0, friction=None, latency_steps=0,
            push=0.0, substeps=2, viewer=None, seed=0, q_noise=0.0):
    t, qpos, qvel, ctrl = ref["t"], ref["qpos"], ref["qvel"], ref["ctrl"]
    dt = float(np.diff(t).mean())
    m = mujoco.MjModel.from_xml_path(SCENE)
    m.opt.timestep = dt / substeps
    if mass_scale != 1.0:
        m.body_mass[1] *= mass_scale
        m.body_inertia[1] *= mass_scale
    if friction is not None:
        m.geom_friction[:, 0] = friction
    d = mujoco.MjData(m)
    d.qpos[:], d.qvel[:] = qpos[0], qvel[0]
    rng = np.random.default_rng(seed)
    d.qpos[7:] += q_noise * rng.standard_normal(12)
    mujoco.mj_forward(m, d)

    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    n_ref, n_hold = t.size, int(round(hold / dt))
    log = dict(qpos=[], tau=[], sat=[], bad_contact=set(), err=[])
    meas = [(d.qpos[7:].copy(), d.qvel[6:].copy())] * (latency_steps + 1)
    for i in range(n_ref + n_hold):
        j = min(i, n_ref - 1)
        q_ref, qd_ref = qpos[j, 7:], (qvel[j, 6:] if i < n_ref else 0.0)
        ff = ctrl[j]
        meas.append((d.qpos[7:].copy(), d.qvel[6:].copy()))
        q_m, qd_m = meas.pop(0)
        want = ff + kp * (q_ref - q_m) + kd * (qd_ref - qd_m)
        for _ in range(substeps):
            tau = hardware_clip(want, d.qvel[6:])
            d.ctrl[:] = tau
            if push and 0 <= i - int(0.5 * n_ref) < int(0.05 / dt):      # a 50 ms shove
                d.xfrc_applied[1, :3] = push * rng.standard_normal(3)
            else:
                d.xfrc_applied[1, :3] = 0.0
            mujoco.mj_step(m, d)
        for c in d.contact[: d.ncon]:
            g = c.geom2 if c.geom1 == floor else c.geom1 if c.geom2 == floor else None
            if g is not None:
                name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
                if name not in FOOT_GEOMS and i < n_ref:
                    log["bad_contact"].add(name)
        log["qpos"].append(d.qpos.copy())
        log["tau"].append(tau.copy())
        log["sat"].append(np.any(np.abs(tau - want) > 1e-9))
        if i < n_ref:
            log["err"].append(np.abs(d.qpos[7:] - q_ref).max())
        if viewer is not None:
            viewer.sync()
    return {k: (np.array(v) if isinstance(v, list) else v) for k, v in log.items()}, n_ref, dt


def score(log, n_ref, ref):
    qp = log["qpos"]
    pitch = pitch_unwrapped(qp[:, 3:7])
    w, x, y, z = qp[-1, 3:7]
    up_z = 1 - 2 * (x * x + y * y)                       # world-z component of body z
    legs_err = np.abs(qp[-1, 7:] - ref["qpos"][-1, 7:]).max()
    out = dict(
        rotation_deg=np.degrees(pitch[-1] - pitch[0]),
        final_tilt_deg=np.degrees(np.arccos(np.clip(up_z, -1, 1))),
        final_height=qp[-1, 2],
        final_legs_err=legs_err,
        apex=qp[:n_ref, 2].max(),
        lateral_drift=np.abs(qp[:, 1]).max(),
        track_err_max=log["err"].max(),
        saturated_frac=float(np.mean(log["sat"][:n_ref])),
        bad_contact=sorted(log["bad_contact"]),
    )
    out["landed"] = bool(abs(abs(out["rotation_deg"]) - 360.0) < 25.0
                         and out["final_tilt_deg"] < 15.0
                         and out["final_height"] > 0.20
                         and legs_err < 0.35)
    return out


SWEEP = (
    [("nominal", {})]
    + [(f"kp={kp:g} kd={kd:g}", dict(kp=kp, kd=kd)) for kp, kd in ((20, 0.5), (30, 1), (40, 1), (100, 3))]
    + [(f"mass x{s:g}", dict(mass_scale=s)) for s in (0.85, 0.95, 1.05, 1.15)]
    + [(f"friction {f:g}", dict(friction=f)) for f in (0.4, 0.5, 1.0)]
    + [(f"latency {n*2} ms", dict(latency_steps=n)) for n in (1, 2, 4)]
    + [(f"push {f:g} N", dict(push=f)) for f in (50, 150)]
    + [("no feedforward", dict(no_ff=True))]
)


def sweep(ref) -> int:
    """One perturbation at a time around kp=60 kd=2. A cheap map of what RL has to add."""
    n_ok = 0
    print(f"  {'case':18s} landed  rot(deg)  tilt(deg)  track(rad)  sat%   stray contact")
    for name, kw in SWEEP:
        kw = dict(kw)
        r = dict(ref)
        if kw.pop("no_ff", False):
            r["ctrl"] = np.zeros_like(r["ctrl"])
        kp, kd = kw.pop("kp", 60.0), kw.pop("kd", 2.0)
        log, n_ref, _ = rollout(r, kp, kd, **kw)
        s = score(log, n_ref, r)
        n_ok += s["landed"]
        print(f"  {name:18s} {'yes' if s['landed'] else 'NO ':6s}  {s['rotation_deg']:8.1f}  "
              f"{s['final_tilt_deg']:9.1f}  {s['track_err_max']:10.3f}  "
              f"{100*s['saturated_frac']:4.1f}   {','.join(s['bad_contact']) or '-'}")
    print(f"  {n_ok}/{len(SWEEP)} land")
    return 0


def _trial(args):
    ref, seed = args
    rng = np.random.default_rng(seed)
    kw = dict(mass_scale=rng.uniform(0.9, 1.1), friction=rng.uniform(0.5, 1.0),
              latency_steps=int(rng.integers(0, 4)), push=rng.uniform(0, 100),
              q_noise=0.02, seed=seed)
    log, n_ref, _ = rollout(ref, rng.uniform(40, 80), rng.uniform(1, 3), **kw)
    s = score(log, n_ref, ref)
    return s["landed"], bool(s["bad_contact"])


def monte_carlo(ref, n) -> int:
    """All perturbations at once, randomised: payload +-10%, friction 0.5-1.0, 0-6 ms latency,
    kp 40-80, kd 1-3, 0.02 rad initial joint noise, a 0-100 N 50 ms shove mid-flight."""
    from multiprocessing import Pool
    with Pool() as pool:
        r = np.array(pool.map(_trial, [(ref, i) for i in range(n)]))
    print(f"  Monte Carlo, {n} trials: lands {100*r[:,0].mean():.1f}%, "
          f"lands with no stray contact {100*(r[:,0] & ~r[:,1]).mean():.1f}%")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz", nargs="?", type=Path, default=REF)
    ap.add_argument("--kp", type=float, default=60.0)
    ap.add_argument("--kd", type=float, default=2.0)
    ap.add_argument("--hold", type=float, default=1.0, help="seconds to hold the final pose")
    ap.add_argument("--mass-scale", type=float, default=1.0)
    ap.add_argument("--friction", type=float, default=None)
    ap.add_argument("--latency-steps", type=int, default=0)
    ap.add_argument("--push", type=float, default=0.0, help="N, random impulse-ish shove mid-tape")
    ap.add_argument("--no-ff", action="store_true", help="PD only, no feedforward torque")
    ap.add_argument("--view", action="store_true")
    ap.add_argument("--save", type=Path, default=None,
                    help="write the closed-loop rollout as an npz that traj_opt/replay.py plays")
    ap.add_argument("--sweep", action="store_true", help="robustness table instead of one run")
    ap.add_argument("--monte-carlo", type=int, default=0, metavar="N")
    args = ap.parse_args()
    if args.sweep:
        return sweep(dict(np.load(args.npz)))
    if args.monte_carlo:
        return monte_carlo(dict(np.load(args.npz)), args.monte_carlo)

    ref = dict(np.load(args.npz))
    if args.no_ff:
        ref["ctrl"] = np.zeros_like(ref["ctrl"])
    kw = dict(hold=args.hold, mass_scale=args.mass_scale, friction=args.friction,
              latency_steps=args.latency_steps, push=args.push)
    if args.view:
        import time
        import mujoco.viewer
        m = mujoco.MjModel.from_xml_path(SCENE)
        print("viewer: replaying closed-loop result kinematically, 0.25x")
        log, n_ref, dt = rollout(ref, args.kp, args.kd, **kw)
        d = mujoco.MjData(m)
        with mujoco.viewer.launch_passive(m, d) as v:
            while v.is_running():
                for q in log["qpos"]:
                    d.qpos[:] = q
                    mujoco.mj_forward(m, d)
                    v.sync()
                    time.sleep(dt * 4)
    else:
        log, n_ref, dt = rollout(ref, args.kp, args.kd, **kw)

    if args.save:
        n = log["qpos"].shape[0]
        np.savez(args.save, t=dt * np.arange(n), qpos=log["qpos"], ctrl=log["tau"])
        print(f"wrote {args.save}")
    s = score(log, n_ref, ref)
    print(f"closed-loop MuJoCo rollout of {args.npz}  (kp={args.kp:g}, kd={args.kd:g})")
    for k, v in s.items():
        print(f"  {k:16s} {v:.4g}" if isinstance(v, float) else f"  {k:16s} {v}")
    return 0 if s["landed"] else 1


if __name__ == "__main__":
    sys.exit(main())
