"""Replay the optimized torques open-loop in MuJoCo and report the divergence. TODO-10.

Drake and MuJoCo agree on the rigid-body model to 1e-13 (verify_parity checks A-E) but not on
contact: Drake sees condim=3 / mu=0.8 point contact, MuJoCo uses condim=6, an elliptic cone
and solimp compliance. The number this prints is the robustness budget the RL stage inherits,
not a bug -- it is expected to grow the moment a foot touches down.

    uv run traj_opt/mj_divergence.py
"""

from __future__ import annotations

import argparse
from pathlib import Path

import mujoco
import numpy as np

from go2_backflip import constants as K

NPZ = Path(__file__).resolve().parent / "out" / "backflip.npz"
SCENE = str(Path(K.MODEL_PATH).parent / "scene.xml")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", type=Path, default=NPZ)
    args = ap.parse_args()

    d = np.load(args.npz)
    t, qpos, qvel, ctrl = d["t"], d["qpos"], d["qvel"], d["ctrl"]
    dt = float(np.diff(t).mean())

    m = mujoco.MjModel.from_xml_path(SCENE)
    m.opt.timestep = dt
    data = mujoco.MjData(m)
    data.qpos[:], data.qvel[:] = qpos[0], qvel[0]

    base, joint, first_contact = [], [], None
    for k in range(t.size):
        data.ctrl[:] = np.clip(ctrl[k], -K.torque_limits(), K.torque_limits())
        mujoco.mj_step(m, data)
        if first_contact is None and data.ncon:
            first_contact = t[k]
        base.append(np.abs(data.qpos[:7] - qpos[k, :7]).max())
        joint.append(np.abs(data.qpos[7:] - qpos[k, 7:]).max())

    base, joint = np.array(base), np.array(joint)
    flight = t < (first_contact if first_contact else t[-1])
    print(f"open-loop torque tape, {t.size} steps at {1/dt:.0f} Hz")
    print(f"  first MuJoCo contact at t = {first_contact}")
    print(f"  before contact : base {base[flight].max():.2e}, joints {joint[flight].max():.2e} rad")
    print(f"  whole tape     : base {base.max():.2e}, joints {joint.max():.2e} rad")
    print(f"  final          : base {base[-1]:.3f}, joints {joint[-1]:.3f} rad")


if __name__ == "__main__":
    main()
