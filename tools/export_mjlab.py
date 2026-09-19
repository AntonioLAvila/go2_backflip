"""Export the reference as an mjlab motion-tracking clip.

mjlab's tracking task (mjlab.tasks.tracking, MotionLoader) reads an npz of

    fps, joint_pos [T,12], joint_vel [T,12],
    body_pos_w [T,B,3], body_quat_w [T,B,4] (wxyz), body_lin_vel_w [T,B,3], body_ang_vel_w [T,B,3]

with bodies in MJCF order (world excluded) and ONE FRAME PER POLICY STEP, so `--fps` must equal
the environment's control rate (50 Hz for mjlab's stock tracking config). Frames are taken by
exact decimation of the 500 Hz tape, never re-interpolated.

A standing hold is padded on both ends: a tracking episode ends with the clip, and a policy that
never sees "and then stand still" does not learn to stick the landing. Carried along as extras,
for feedforward / reward shaping / a contact-schedule observation: `ctrl_ff`, `contact`, `grf`,
`phase`, `body_names`, `joint_names`.

    uv run tools/export_mjlab.py [--fps 50] [--pre-hold 0.5] [--hold 1.0]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from go2_backflip import constants as K

REF = Path(K.REPO_ROOT) / "traj_opt" / "reference"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("npz", nargs="?", type=Path, default=REF / "backflip.npz")
    ap.add_argument("--out", type=Path, default=REF / "backflip_mjlab.npz")
    ap.add_argument("--fps", type=float, default=50.0)
    ap.add_argument("--pre-hold", type=float, default=0.5)
    ap.add_argument("--hold", type=float, default=1.0)
    args = ap.parse_args()

    d = np.load(args.npz)
    hz = 1.0 / float(np.diff(d["t"]).mean())
    step = hz / args.fps
    assert abs(step - round(step)) < 1e-9, f"tape rate {hz:g} is not a multiple of fps {args.fps:g}"
    idx = np.arange(0, d["t"].size, round(step))
    n_pre, n_post = round(args.pre_hold * args.fps), round(args.hold * args.fps)

    def pad(a, rest_first=None, rest_last=None):
        a = a[idx]
        first = a[:1] if rest_first is None else rest_first[None]
        last = a[-1:] if rest_last is None else rest_last[None]
        return np.concatenate([np.repeat(first, n_pre, 0), a, np.repeat(last, n_post, 0)])

    qpos = pad(d["qpos"])
    zero_v = np.zeros(d["qvel"].shape[1])
    qvel = pad(d["qvel"], zero_v, zero_v)
    stand = np.ones(4, bool)

    m = mujoco.MjModel.from_xml_path(K.MODEL_PATH)
    md = mujoco.MjData(m)
    nb = m.nbody - 1
    T = qpos.shape[0]
    pos, quat = np.zeros((T, nb, 3)), np.zeros((T, nb, 4))
    lin, ang = np.zeros((T, nb, 3)), np.zeros((T, nb, 3))
    vel6 = np.zeros(6)
    for i in range(T):
        md.qpos[:], md.qvel[:] = qpos[i], qvel[i]
        mujoco.mj_forward(m, md)
        pos[i], quat[i] = md.xpos[1:], md.xquat[1:]
        for b in range(nb):
            mujoco.mj_objectVelocity(m, md, mujoco.mjtObj.mjOBJ_BODY, b + 1, vel6, 0)
            ang[i, b], lin[i, b] = vel6[:3], vel6[3:]

    np.savez(
        args.out, fps=np.array([args.fps]),
        joint_pos=qpos[:, 7:], joint_vel=qvel[:, 6:],
        body_pos_w=pos, body_quat_w=quat, body_lin_vel_w=lin, body_ang_vel_w=ang,
        ctrl_ff=pad(d["ctrl"]), contact=pad(d["contact"], stand, stand),
        grf=pad(d["grf"]), phase=pad(d["phase"]),
        body_names=np.array([m.body(b + 1).name for b in range(nb)]),
        joint_names=np.array(K.JOINT_NAMES),
    )
    print(f"wrote {args.out}: {T} frames at {args.fps:g} fps "
          f"({n_pre} hold + {idx.size} flip + {n_post} hold), {nb} bodies")


if __name__ == "__main__":
    main()
