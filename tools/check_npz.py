"""Geometric review of a saved trajectory, straight from the .npz replay.py plays.

audit.py answers the same questions about a *solver result*; this answers them about the file
on disk, which is what actually gets replayed and what the RL stage will consume. That gap is
not hypothetical: out/backflip.npz sat for days holding a pre-geometry-fix trajectory that
drove the head 72 mm through the floor and never tucked, while the program in the repo had
constraints forbidding both.

Reports, per frame, over MuJoCo's own kinematics:
  * lowest point of every collision geom (exact for box/sphere/capsule/cylinder), so floor
    penetration is measured, not assumed;
  * I_yy about the CoM during flight, the number that decides whether the flip is cheap
    (0.45 tucked, 0.48 standing, 0.66 sprawled);
  * net base pitch, and the flight window it was measured over.

    uv run tools/check_npz.py [--npz traj_opt/out/backflip.npz]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from go2_backflip import constants as K

NPZ = Path(__file__).resolve().parents[1] / "traj_opt" / "out" / "backflip.npz"
FLIGHT_FOOT_Z = 0.005     # a foot this far up is not carrying the robot
# Fraction of the flight left out at each end when judging the tuck: program.py's own
# TUCK_RAMP / flight knots, so the same knots are being judged, sampled in time instead.
RAMP = 6 / 32


def lowest_z(m: mujoco.MjModel, d: mujoco.MjData, g: int) -> float:
    """Exact lowest world z of geom `g`, for every collision primitive go2.xml uses."""
    t, s = m.geom_type[g], m.geom_size[g]
    z, R = d.geom_xpos[g][2], d.geom_xmat[g].reshape(3, 3)
    if t == mujoco.mjtGeom.mjGEOM_SPHERE:
        return z - s[0]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        return z - float(np.abs(R[2, :3]) @ s[:3])
    if t in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        r, h, az = s[0], s[1], R[2, 2]        # local +z is the axis
        if t == mujoco.mjtGeom.mjGEOM_CAPSULE:
            return z - h * abs(az) - r        # spherical caps: the sphere always hangs r below
        return z - h * abs(az) - r * np.sqrt(max(0.0, 1.0 - az * az))
    raise ValueError(f"geom {mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)}: "
                     f"unhandled type {t}")


def inertia_yy(m: mujoco.MjModel, d: mujoco.MjData) -> float:
    """I_yy of the whole robot about its CoM, in world axes."""
    com = np.average(d.xipos[1:], axis=0, weights=m.body_mass[1:])
    tot = 0.0
    for b in range(1, m.nbody):
        R = d.ximat[b].reshape(3, 3)
        I = R @ np.diag(m.body_inertia[b]) @ R.T
        r = d.xipos[b] - com
        tot += I[1, 1] + m.body_mass[b] * (r[0] ** 2 + r[2] ** 2)
    return tot


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", type=Path, default=NPZ)
    args = ap.parse_args()

    d0 = np.load(args.npz)
    t, qpos = d0["t"], d0["qpos"]

    m = mujoco.MjModel.from_xml_path(K.MODEL_PATH)
    data = mujoco.MjData(m)
    geoms = [g for g in range(m.ngeom) if m.geom_group[g] == 3]
    feet = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"{f}_foot") for f in K.FEET]
    if any(g < 0 for g in feet):                      # foot geoms are unnamed in some revisions
        feet = [g for g in geoms if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE
                and m.geom_size[g][0] == K.R_FOOT]

    low = np.zeros((t.size, len(geoms)))
    iyy = np.zeros(t.size)
    pitch = np.zeros(t.size)
    for k in range(t.size):
        data.qpos[:] = qpos[k]
        mujoco.mj_kinematics(m, data)
        mujoco.mj_comPos(m, data)
        low[k] = [lowest_z(m, data, g) for g in geoms]
        iyy[k] = inertia_yy(m, data)
        w, y = qpos[k][3], qpos[k][5]                 # sagittal: quat is [cos(th/2),0,sin(th/2),0]
        pitch[k] = 2.0 * np.arctan2(y, w)

    # Unwrap so a full turn reads as -360, not as a wrap back to 0.
    turn = np.degrees(np.unwrap(pitch))
    flight = np.flatnonzero(low[:, [geoms.index(g) for g in feet]].min(axis=1) > FLIGHT_FOOT_Z)

    print(f"{args.npz}  {t.size} samples, {t[0]:.3f}-{t[-1]:.3f} s")
    print(f"net base pitch {turn[-1] - turn[0]:+.4f} deg")

    print("\nfloor clearance (lowest point of each collision geom):")
    worst = np.argsort(low.min(axis=0))
    for i in worst[:6]:
        k = int(low[:, i].argmin())
        name = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, geoms[i]) or f"geom{geoms[i]}"
        print(f"  {name:<18} {low[k, i] * 1000:+8.1f} mm  at t={t[k]:.3f}s")
    pen = -low.min()
    print(f"  worst penetration {pen * 1000:+.1f} mm "
          f"({'OK' if pen < 2e-3 else 'CLIPS THE FLOOR'})")

    print("\ntuck (I_yy about the CoM):")
    if flight.size:
        a, b = flight[0], flight[-1]
        # Judge the tuck over the middle of the flight only, the same window program.py holds
        # FLIGHT_TUCK over: the ends are the fold-in and the extension for landing, where a
        # high I_yy is the intended behaviour, not a missing tuck.
        i, j = (int(a + RAMP * (b - a)), int(b - RAMP * (b - a)))
        peak = iyy[i:j + 1].max()
        print(f"  flight {t[a]:.3f}-{t[b]:.3f}s ({t[b] - t[a]:.3f}s), "
              f"I_yy over it: min {iyy[a:b + 1].min():.4f} max {iyy[a:b + 1].max():.4f} kg.m^2")
        print(f"  held {t[i]:.3f}-{t[j]:.3f}s (middle, ramps excluded): peak {peak:.4f}")
        print(f"  reference: tucked 0.452, standing 0.484, sprawled 0.659 -- "
              f"{'OK' if peak < 0.55 else 'NOT TUCKED'}")
    else:
        print("  no flight phase found (no frame has all four feet clear of the floor)")


if __name__ == "__main__":
    main()
