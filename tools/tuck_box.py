"""Largest self-collision-free box in the sagittal tuck variables, for the TO flight phase.

With abduction pinned to zero and the legs mirrored left/right, the tuck is four numbers:
(thigh_front, calf_front, thigh_rear, calf_rear). Every cross-side pair in
TUCK_COLLISION_PAIRS is then unreachable, but the same-side front<->rear pairs are not, so
the flight knots need a bound. A box is enough and costs the NLP nothing.

go2.xml carries no floor, so any contact MuJoCo reports here is a self-collision.

    uv run tools/tuck_box.py
"""

from __future__ import annotations

import itertools
import json

import mujoco
import numpy as np

from go2_backflip import constants as K

# The flight phase has to tuck AND extend again for the landing, so the box is grown from the
# region spanned by both poses, not from the tuck alone.
SEED_LO = (0.9, -2.7, 0.9, -2.7)        # min of HOME_LEGS, TUCK_LEGS per sagittal axis
SEED_HI = (2.2, -1.8, 2.2, -1.8)        # max
N = 21                                  # grid points per axis
MARGIN = 0.005                          # m of clearance demanded, via geom margin


def _axes(m) -> list[np.ndarray]:
    lim = {n: m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in K.JOINT_NAMES}
    return [
        np.linspace(*lim["FL_thigh_joint"], N),
        np.linspace(*lim["FL_calf_joint"], N),
        np.linspace(*lim["RL_thigh_joint"], N),
        np.linspace(*lim["RL_calf_joint"], N),
    ]


def _legs(tf: float, cf: float, tr: float, cr: float) -> np.ndarray:
    return np.array([0, tf, cf, 0, tf, cf, 0, tr, cr, 0, tr, cr], dtype=float)


def sweep(m, axes) -> np.ndarray:
    d = mujoco.MjData(m)
    free = np.zeros((N, N, N, N), dtype=bool)
    for idx in itertools.product(range(N), repeat=4):
        d.qpos[:] = K.mj_qpos(_legs(*[axes[a][i] for a, i in enumerate(idx)]), 1.0)
        mujoco.mj_kinematics(m, d)
        mujoco.mj_collision(m, d)
        free[idx] = d.ncon == 0
    return free


def grow(free: np.ndarray, box: list[list[int]]) -> list[tuple[int, int]]:
    """Greedily widen a box one grid step at a time, while it stays collision-free."""
    box = [list(b) for b in box]
    while True:
        grew = False
        for axis in range(4):
            for end, step in ((1, 1), (0, -1)):
                nxt = box[axis][end] + step
                if not 0 <= nxt < N:
                    continue
                trial = [list(b) for b in box]
                trial[axis][end] = nxt
                sl = tuple(slice(lo, hi + 1) for lo, hi in trial)
                if free[sl].all():
                    box[axis][end] = nxt
                    grew = True
        if not grew:
            return [tuple(b) for b in box]


def main() -> None:
    m = mujoco.MjModel.from_xml_path(K.MODEL_PATH)
    m.geom_margin[:] = MARGIN
    axes = _axes(m)

    free = sweep(m, axes)
    print(f"grid {N}^4 = {free.size} poses, {free.sum()} collision-free ({100*free.mean():.1f}%)")

    # Snap the seed region outward to grid cells so the reported box really contains it.
    seed = [[int(np.searchsorted(axes[a], SEED_LO[a], "right") - 1),
             int(np.searchsorted(axes[a], SEED_HI[a], "left"))] for a in range(4)]
    if not free[tuple(slice(lo, hi + 1) for lo, hi in seed)].all():
        raise SystemExit("seed region collides -- nothing to grow from")

    names = ["thigh_front", "calf_front", "thigh_rear", "calf_rear"]
    box = grow(free, seed)
    out = {n: [float(axes[a][lo]), float(axes[a][hi])] for a, (n, (lo, hi)) in enumerate(zip(names, box))}
    for a, n in enumerate(names):
        print(f"  {n:12s} [{out[n][0]:+.4f}, {out[n][1]:+.4f}]  seed [{SEED_LO[a]:+.2f}, {SEED_HI[a]:+.2f}]")
    print(json.dumps(out))


if __name__ == "__main__":
    main()
