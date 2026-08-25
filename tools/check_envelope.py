"""torque_speed_halfplanes() must reproduce torque_speed_bound(), and differ only where intended.

    uv run tools/check_envelope.py
"""

from __future__ import annotations

import sys

import numpy as np

from go2_backflip import constants as K


def main() -> int:
    k, tau_stall = K.torque_speed_halfplanes()
    rng = np.random.default_rng(0)
    qd = rng.uniform(-1, 1, (10000, 12)) * K.speed_limits()

    # Half-plane cap in the motoring quadrant, i.e. tau taking the sign of qd.
    hp = np.minimum(K.torque_limits(), tau_stall - k * np.abs(qd))
    ref = np.array([K.torque_speed_bound(row) for row in qd])

    err = np.abs(hp - ref).max()
    ok = err < 1e-12
    print(f"[{'PASS' if ok else 'FAIL'}] motoring quadrant agrees -- max err {err:.2e} N.m")

    # Braking: the half-planes allow peak torque at any speed, torque_speed_bound does not.
    brake = np.minimum(K.torque_limits(), tau_stall + k * np.abs(qd))
    n = int((brake > ref + 1e-9).sum())
    print(f"[INFO] regenerating quadrant underated at {n}/{ref.size} samples, as intended")

    print(f"tau_stall {np.round(tau_stall[:3], 2)}  k {np.round(k[:3], 2)}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
