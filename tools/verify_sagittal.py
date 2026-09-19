"""Prove the CasADi sagittal model IS go2.xml restricted to its symmetry plane.

sagittal.py assembles a Lagrangian from MuJoCo's compiled model and lets CasADi differentiate
it. This checks the result against MuJoCo itself on random states over the whole flip envelope:

  A  equations of motion   sagittal.dyn / .hip   vs  mj_inverse          (planar + hip rows)
  B  contact Jacobian      sagittal.foot_J       vs  mj_jac at the sphere's floor point
  C  contact force map     2 J^T lam             vs  sum over all four feet of J_i^T lam_i
  D  symmetry leak         the rows the plane ignores (y, roll, yaw) -- report only. go2.xml's
                           base inertia is not exactly mirror-symmetric, so this is nonzero; it
                           is the out-of-plane disturbance a tracking controller inherits.
  E  momentum              sagittal.momentum     vs  mj_subtree momentum about the CoM

    uv run tools/verify_sagittal.py
"""

from __future__ import annotations

import sys

import mujoco
import numpy as np

from go2_backflip import constants as K
from go2_backflip import sagittal as S

TOL = 1e-9


def main() -> int:
    m = mujoco.MjModel.from_xml_path(K.MODEL_PATH)
    m.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    d = mujoco.MjData(m)
    sag = S.build()
    rng = np.random.default_rng(0)

    E = np.array([S.embed_qvel(e) for e in np.eye(S.NQ)]).T            # 18x7
    lo = np.where(np.isfinite(sag.q_lo), sag.q_lo, [-1, 0.2, -7, 0, 0, 0, 0])
    hi = np.where(np.isfinite(sag.q_hi), sag.q_hi, [1, 1.0, 1, 0, 0, 0, 0])
    worst = dict(A=0.0, A_hip=0.0, B=0.0, C=0.0, D=0.0, E=0.0)
    for _ in range(200):
        q = rng.uniform(lo, hi)
        v = rng.normal(size=S.NQ) * [2, 2, 8, 10, 10, 10, 10]
        a = rng.normal(size=S.NQ) * 50
        d.qpos[:], d.qvel[:], d.qacc[:] = S.embed_qpos(q), S.embed_qvel(v), S.embed_qvel(a)
        mujoco.mj_inverse(m, d)
        f = d.qfrc_inverse.copy()

        z2, z4 = np.zeros(2), np.zeros(4)
        r = np.array(sag.dyn(q, v, a, z4, z2, z2)).ravel()
        worst["A"] = max(worst["A"], np.abs(r - E.T @ f).max())
        h = np.array(sag.hip(q, v, a, z2, z2)).ravel()
        worst["A_hip"] = max(worst["A_hip"], abs(h[0] - f[6]), abs(h[1] - f[12]))
        worst["D"] = max(worst["D"], np.abs(f[[1, 3, 5]]).max(), abs(f[6] + f[9]))

        # contact: floor point of each foot sphere, and the force map through all four feet
        lam = {"front": rng.normal(size=2) * 100, "rear": rng.normal(size=2) * 100}
        gen = np.zeros(18)
        Jg = np.array(sag.foot_J(q))
        for i, pair in enumerate(S.PAIRS):
            for leg, sy in ((S._LEFT[pair], 1.0), (S._LEFT[pair][0] + "R", -1.0)):
                b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{leg}_calf")
                pt = d.xpos[b] + d.xmat[b].reshape(3, 3) @ K.P_ANKLE - [0, 0, K.R_FOOT]
                jp = np.zeros((3, 18))
                mujoco.mj_jac(m, d, jp, None, pt, b)
                gen += jp.T @ [lam[pair][0], 0.0, lam[pair][1]]
                if sy > 0:
                    worst["B"] = max(worst["B"], np.abs((jp @ E)[[0, 2]] - Jg[2*i:2*i+2]).max())
        rc = np.array(sag.dyn(q, v, a, z4, lam["front"], lam["rear"])).ravel()
        worst["C"] = max(worst["C"], np.abs((r - rc) - E.T @ gen).max())

        # momentum about the CoM
        mujoco.mj_forward(m, d)
        mujoco.mj_subtreeVel(m, d)
        mom = np.array(sag.momentum(q, v)).ravel()
        ref = np.array([m.body_subtreemass[1] * d.subtree_linvel[1][0],
                        m.body_subtreemass[1] * d.subtree_linvel[1][2], d.subtree_angmom[1][1]])
        worst["E"] = max(worst["E"], np.abs(mom - ref).max())
        worst["E"] = max(worst["E"], np.abs(np.array(sag.com(q)).ravel()
                                            - d.subtree_com[1][[0, 2]]).max())

    ok = True
    for k, val in worst.items():
        if k == "D":
            print(f"  D  symmetry leak (report only)    {val:.3e}  N / N.m")
            continue
        good = val < TOL
        ok &= good
        print(f"  {k:5s} {'PASS' if good else 'FAIL'}  max abs error {val:.3e}")
    print(f"  total mass {sag.total_mass:.4f} kg, MuJoCo {m.body_subtreemass[1]:.4f}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
