"""Check the resampled 500 Hz tape, not the knots.

`audit.py` evaluates its checks at the collocation knots, because that is where the
constraints bind. The artifact downstream stages actually consume is
`traj_opt/reference/backflip.npz` -- a uniform 500 Hz resampling of the cubic state spline
and the first-order-held torque -- and both ring between knots. This repo has been bitten by
that twice:

  * A trajectory shipped with the audit reporting `worst overshoot 0.00e+00 N.m` while the
    tape demanded up to 4.86 N.m more than the enforced envelope on 6.1% of its steps. That is
    what the 2% actuator safety factor was introduced to absorb.
  * The 2026-09-06 convergence campaign added a hard flight angular-momentum box. It holds to
    6.5e-04 at the knots and rings to 1.7e-03 between them -- over the audit's own 1e-3 bound,
    measured where the RL stage reads.

So this is deliberately NOT a twelfth audit check: it is the same physics asked at the samples
the audit cannot see, and a trajectory can pass audit.py and fail here.

    uv run tools/check_tape.py [traj_opt/reference/backflip.npz]
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "traj_opt"))

from go2_backflip import constants as K                                    # noqa: E402
from program import make_plant                                             # noqa: E402

AMOM_BOUND = 1e-3          # the same bound audit.py holds the knots to
# Foot-sphere clearance that counts as "off the ground". The detected window is insensitive
# to this between 5e-3 and 2e-4 (it moves by one 2 ms sample); below ~1e-4 it collapses,
# because a foot in stance is only numerically at zero clearance and the whole trajectory
# then reads as airborne.
FOOT_AIR = 1e-3


def flight_window(plant, ctx, qpos, qvel):
    """The samples where every foot sphere is clear of the floor, plus L_y at each sample.

    Detected from foot geometry rather than base height. Base height is well above its
    starting value throughout launch and absorb, and those are CONTACT phases where angular
    momentum is not conserved -- including them reports ~1.9 N.m.s of meaningless "drift".
    """
    feet = [plant.GetFrameByName(f"{f}_calf") for f in ("FL", "FR", "RL", "RR")]
    P = K.P_ANKLE.reshape(3, 1)
    L, clear = [], []
    for qp, qv in zip(qpos, qvel):
        q = K.mj_to_drake_q(qp)
        plant.SetPositions(ctx, q)
        plant.SetVelocities(ctx, K.mj_to_drake_v(qv, q[:4]))
        com = plant.CalcCenterOfMassPositionInWorld(ctx)
        L.append(plant.CalcSpatialMomentumInWorldAboutPoint(ctx, com).rotational()[1])
        clear.append(min(float(plant.CalcPointsPositions(ctx, fr, P, plant.world_frame())[2, 0])
                         - K.R_FOOT for fr in feet))
    air = np.array(clear) > FOOT_AIR
    return np.array(L), int(np.argmax(air)), len(air) - int(np.argmax(air[::-1]))


def main() -> int:
    path = Path(sys.argv[1] if len(sys.argv) > 1 else "traj_opt/reference/backflip.npz")
    d = np.load(path)
    t, qpos, qvel, tau = d["t"], d["qpos"], d["qvel"], d["ctrl"]
    print(f"{path}: {t.size} samples, {t[-1]:.3f} s")
    ok = True

    # Against torque_speed_halfplanes(), which is both what program.py enforces AND the
    # physically right envelope. tools/check_envelope.py proves the two agree to 1e-12 in the
    # motoring quadrant and differ only in braking, where torque_speed_bound() derates on
    # |qd| and so under-rates a back-driven motor that can in fact hold peak torque. Checking
    # the tape against torque_speed_bound() therefore invents large failures in exactly the
    # quadrant the formulation deliberately models the other way -- it reports +28 N.m on the
    # shipped reference against the halfplanes' +2.3. The quadrant of the worst sample is
    # printed because that is what says whether an overshoot is a real over-demand.
    qd = qvel[:, 6:]
    k_ts, tau_stall = K.torque_speed_halfplanes()
    over = np.maximum.reduce([np.abs(tau) - K.torque_limits(),
                              tau + k_ts * qd - tau_stall, -tau - k_ts * qd - tau_stall])
    worst = float(over.max())
    frac = 100.0 * (over.max(axis=1) > 0).mean()
    i, j = np.unravel_index(int(over.argmax()), over.shape)
    quadrant = "regenerating" if tau[i, j] * qd[i, j] < 0 else "motoring"
    # Against the HARDWARE torque-speed envelope, not merely the flat peak. The flat peak on
    # its own gives false comfort: it is |tau| <= tau_peak with no speed derating, so a tape
    # can read "0.479 N.m of clearance" while asking, at speed, for 0.51 N.m more than the
    # motor can actually produce. Same halfplane form as torque_speed_halfplanes(), built on
    # the datasheet peaks instead of the design ones -- so this is the real physical question,
    # and the design-envelope line above is the stricter one the NLP was given.
    hw_pk = K.hardware_torque_limits()
    stall_hw = hw_pk / (1.0 - K.CORNER_SPEED_FRAC)
    k_hw = stall_hw / K.speed_limits()
    hw = float(np.maximum.reduce([np.abs(tau) - hw_pk,
                                  tau + k_hw * qd - stall_hw,
                                  -tau - k_hw * qd - stall_hw]).max())
    hw_flat = float((np.abs(tau) - hw_pk).max())

    ok &= worst <= 0
    print(f"  [{'PASS' if worst <= 0 else 'FAIL'}] torque inside the enforced design envelope"
          f"  -- worst {worst:+.4f} N.m on {frac:.2f}% of samples "
          f"({K.JOINT_NAMES[j]} at t={t[i]:.3f}s, {quadrant})")
    ok &= hw <= 0
    print(f"  [{'PASS' if hw <= 0 else 'FAIL'}] torque inside the HARDWARE torque-speed "
          f"envelope  -- worst {hw:+.4f} N.m (negative is clearance); "
          f"against the flat peak alone {hw_flat:+.4f}")

    plant = make_plant()
    L, i0, i1 = flight_window(plant, plant.CreateDefaultContext(), qpos, qvel)
    # PEAK-TO-PEAK, not |L - L[first sample]|. The audit anchors on the first flight KNOT;
    # this window starts a sample or two later, so anchoring here understates by however much
    # L had already moved -- measured 8.8e-04 anchored against the audit's 1.50e-03 on the
    # same trajectory, where the peak-to-peak is 1.47e-03. Peak-to-peak needs no reference
    # point and upper-bounds every anchored drift, so it cannot flatter the tape.
    seg = L[i0:i1]
    drift = float(seg.max() - seg.min())
    ok &= drift < AMOM_BOUND
    print(f"  [{'PASS' if drift < AMOM_BOUND else 'FAIL'}] flight angular momentum conserved"
          f"  -- L_y = {L[i0]:.3f} N.m.s, peak-to-peak {drift:.2e}, bound {AMOM_BOUND:.0e}"
          f"  (flight = t[{t[i0]:.3f}, {t[i1 - 1]:.3f}] s, {i1 - i0} samples)")
    print("tape checks passed" if ok else "TAPE CHECKS FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
