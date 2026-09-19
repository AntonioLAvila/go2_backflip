"""Physics audit of a shipped tape, asked of MuJoCo rather than of the solver.

Everything here runs on the 500 Hz npz the downstream stages read -- between the knots, where a
transcription can hide things -- and everything except the last check uses MuJoCo alone, so a
bug in the CasADi model or the NLP cannot vouch for itself:

  * inverse dynamics: mj_inverse on (qpos, qvel, qacc) must equal ctrl + sum J^T grf
  * actuators: torque against the design AND the hardware torque-speed halfplanes, joint
    position and speed limits
  * contact: unilateral, inside the friction cone, stance feet on the floor and not sliding
  * geometry: nothing but a foot within `clearance` of the floor, swing feet above it
  * flight: CoM ballistic, angular momentum about the CoM conserved
  * ends: starts at HOME at rest, ends one full backward rotation later at HOME at rest
  * re-integration: each phase re-simulated from its initial state by a tight adaptive
    integrator under the tape's torques (stance as constrained dynamics) and compared at the end

    uv run traj_opt/validate.py [traj_opt/reference/backflip.npz]
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import NamedTuple

import mujoco
import numpy as np
from scipy.integrate import solve_ivp
from scipy.interpolate import interp1d

from go2_backflip import constants as K
from go2_backflip import sagittal as S

REF = Path(K.REPO_ROOT) / "traj_opt" / "reference" / "backflip.npz"
SCENE = str(Path(K.MODEL_PATH).parent / "scene.xml")


class Check(NamedTuple):
    name: str
    value: float
    limit: float
    unit: str

    @property
    def ok(self):
        return self.value <= self.limit


def _halfplane_over(tau, qd, peak):
    k, stall = K.torque_speed_halfplanes(peak)
    return np.max([np.abs(tau) - peak, tau + k * qd - stall, -tau - k * qd - stall], axis=0).max()


def _pitch(qpos):
    w, y = qpos[:, 3], qpos[:, 5]
    return np.unwrap(2 * np.arctan2(y, w), period=4 * np.pi)


def audit(d, mu=K.MU_TO, clearance=0.02, torque_sf=None) -> list[Check]:
    t, qpos, qvel, qacc, ctrl = d["t"], d["qpos"], d["qvel"], d["qacc"], d["ctrl"]
    contact, grf, phase = d["contact"], d["grf"], d["phase"]
    torque_sf = float(d["torque_sf"]) if torque_sf is None and "torque_sf" in d else (torque_sf or 1.0)
    m = mujoco.MjModel.from_xml_path(SCENE)
    md = mujoco.MjData(m)
    floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
    calf = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{f}_calf") for f in K.FEET]
    foot_geom = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f) for f in K.FEET]
    body_geoms = [g for g in range(m.ngeom) if g != floor and g not in foot_geom
                  and m.geom_contype[g] | m.geom_conaffinity[g]]

    n = t.size
    resid = np.zeros(n)
    foot_z, foot_x = np.zeros((n, 4)), np.zeros((n, 4))
    body_gap = np.zeros(n)
    com, L = np.zeros((n, 3)), np.zeros((n, 3))
    fromto = np.zeros(6)
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    for i in range(n):
        md.qpos[:], md.qvel[:], md.qacc[:] = qpos[i], qvel[i], qacc[i]
        mujoco.mj_inverse(m, md)
        applied = np.zeros(m.nv)
        applied[6:] = ctrl[i]
        for f in range(4):
            c = md.xpos[calf[f]] + md.xmat[calf[f]].reshape(3, 3) @ K.P_ANKLE
            foot_z[i, f], foot_x[i, f] = c[2] - K.R_FOOT, c[0]
            if contact[i, f]:
                jp = np.zeros((3, m.nv))
                mujoco.mj_jac(m, md, jp, None, c - [0, 0, K.R_FOOT], calf[f])
                applied += jp.T @ grf[i, f]
        resid[i] = np.abs(md.qfrc_inverse - applied)[[0, 2, 4] + list(range(6, 18))].max()
        body_gap[i] = min(mujoco.mj_geomDistance(m, md, g, floor, 1.0, fromto) for g in body_geoms)
        mujoco.mj_subtreeVel(m, md)
        com[i], L[i] = md.subtree_com[1], md.subtree_angmom[1]

    checks = []
    add = lambda *a: checks.append(Check(*a))                                   # noqa: E731
    pitch = _pitch(qpos)
    add("net rotation is one backflip", abs(pitch[-1] - pitch[0] + 2 * np.pi), 1e-3, "rad")
    add("starts at HOME at rest", max(np.abs(qpos[0] - S.embed_qpos(S.HOME)).max(),
                                       np.abs(qvel[0]).max()), 1e-6, "")
    add("ends at HOME legs, at rest", max(np.abs(qpos[-1, 7:] - K.HOME_LEGS).max(),
                                          np.abs(qvel[-1]).max()), 2e-2, "rad, rad/s")
    add("tape satisfies MuJoCo inverse dynamics", resid.max(), 0.5, "N, N.m")

    add("torque inside the DESIGN torque-speed envelope",
        _halfplane_over(ctrl, qvel[:, 6:], torque_sf * np.array(
            [K.NOMINAL_TORQUE[K.joint_kind(j)] for j in K.JOINT_NAMES])), 0.05, "N.m over")
    add("torque inside the HARDWARE torque-speed envelope",
        _halfplane_over(ctrl, qvel[:, 6:], K.hardware_torque_limits()), 0.0, "N.m over")
    add("joint speed limits", (np.abs(qvel[:, 6:]) / K.speed_limits()).max(), 1.0, "of max")
    jr = m.jnt_range[1:]
    add("joint position limits", np.max([jr[:, 0] - qpos[:, 7:], qpos[:, 7:] - jr[:, 1]]), 1e-6, "rad over")

    fz, fx = grf[..., 2][contact], grf[..., 0][contact]
    add("ground force unilateral", max(0.0, -fz.min()), 0.5, "N")
    add("ground force inside friction cone", (np.abs(fx) - mu * fz).max(), 0.5, "N over")
    add("stance feet on the floor", np.abs(foot_z[contact]).max(), 1e-3, "m")
    slip = 0.0
    for f in range(4):
        for p in np.unique(phase[contact[:, f]]):
            sel = contact[:, f] & (phase == p)
            cp = np.unwrap(2 * np.arctan2(qpos[sel, 5], qpos[sel, 3]), period=4 * np.pi) \
                + qpos[sel, 7 + 3 * f + 1] + qpos[sel, 7 + 3 * f + 2]
            roll = foot_x[sel, f] - K.R_FOOT * cp                 # rolling-sphere invariant
            slip = max(slip, np.ptp(roll))
    add("stance feet roll without slipping", slip, 1e-3, "m")
    add("swing feet above the floor", max(0.0, -foot_z[~contact].min()), 1e-4, "m below")
    add(f"non-foot geometry keeps {clearance*1e3:.0f} mm off the floor",
        max(0.0, clearance - body_gap.min()), 1e-3, "m short")

    fl = ~contact.any(1)
    tf = t[fl] - t[fl][0]
    fit = np.polyfit(tf, com[fl, 2], 2)
    add("flight CoM is ballistic", abs(-2 * fit[0] - S.G) / S.G, 1e-3, "rel. error in g")
    add("flight angular momentum conserved", np.ptp(L[fl, 1]), 1e-3, "N.m.s p-p")

    add("phases re-integrate under a tight integrator", _reintegrate(d), 5e-3, "rad, m")
    return checks


def _reintegrate(d):
    """Worst end-of-phase state error re-simulating each phase open loop (RK45, rtol 1e-10)."""
    sag = S.build()
    t, phase, contact = d["t"], d["phase"], d["contact"]
    planar_q = lambda qp: np.array([qp[0], qp[2], 0, qp[8], qp[9], qp[14], qp[15]])   # noqa: E731
    pitch = _pitch(d["qpos"])
    worst = 0.0
    for p in np.unique(phase):
        idx = np.flatnonzero(phase == p)
        if idx.size < 4:
            continue
        rows = [r for c, r in ((contact[idx[0], 0], (0, 1)), (contact[idx[0], 2], (2, 3))) if c
                for r in r]
        u_of = interp1d(t[idx], d["ctrl"][idx][:, [1, 2, 7, 8]], axis=0, kind="cubic",
                        fill_value="extrapolate")
        q0 = planar_q(d["qpos"][idx[0]]); q0[2] = pitch[idx[0]]
        v0 = d["qvel"][idx[0]][[0, 2, 4, 7, 8, 13, 14]]
        g0 = np.array(sag.foot(q0)).ravel()
        alpha = 20.0

        def f(tt, x, rows=rows, u_of=u_of, g0=g0):
            q, v = x[:7], x[7:]
            M = np.array(sag.mass(q))
            rhs = sag.act @ u_of(tt) - np.array(sag.bias(q, v)).ravel()
            if not rows:
                return np.concatenate([v, np.linalg.solve(M, rhs)])
            J = np.array(sag.foot_J(q))[rows]
            gam = np.array(sag.foot_acc(q, v, np.zeros(7))).ravel()[rows]
            bg = gam + 2 * alpha * (J @ v) + alpha**2 * (np.array(sag.foot(q)).ravel() - g0)[rows]
            nc = len(rows)
            A = np.block([[M, -2 * J.T], [J, np.zeros((nc, nc))]])
            return np.concatenate([v, np.linalg.solve(A, np.concatenate([rhs, -bg]))[:7]])

        r = solve_ivp(f, (t[idx[0]], t[idx[-1]]), np.concatenate([q0, v0]), rtol=1e-10, atol=1e-12)
        qf = planar_q(d["qpos"][idx[-1]]); qf[2] = pitch[idx[-1]]
        worst = max(worst, np.abs(r.y[:7, -1] - qf).max())
    return worst


def report(checks) -> bool:
    for c in checks:
        print(f"  {'PASS' if c.ok else 'FAIL'}  {c.name:52s} {c.value:10.3e}  (limit {c.limit:g} {c.unit})")
    n_ok = sum(c.ok for c in checks)
    print(f"  {n_ok}/{len(checks)} checks pass")
    return n_ok == len(checks)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz", nargs="?", type=Path, default=REF)
    args = ap.parse_args()
    return 0 if report(audit(dict(np.load(args.npz)))) else 1


if __name__ == "__main__":
    sys.exit(main())
