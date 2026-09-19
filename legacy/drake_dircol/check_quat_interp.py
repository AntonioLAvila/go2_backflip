"""Diagnostic: how far does DirectCollocation's flat cubic-Hermite state spline stray from
the unit-quaternion manifold BETWEEN knots, and is that discrepancy actually the same thing
as interpolating the (sagittally-collapsed) pitch angle directly.

audit.check_rotation only ever samples at knots (bp.dc[p].GetStateSamples), so it cannot see
this. Not a tracked tool -- a one-off check for whether the "DirectCollocation interpolates
quaternions in flat R^4, not on SO(3)" concern is actually costing anything on THIS problem,
where sagittal symmetry has already collapsed the rotation to a single scalar angle
(quat = [cos(t/2), 0, sin(t/2), 0]).
"""
import numpy as np

from program import BackflipProgram, XQ, XV
from schedule import PHASES
from solve_backflip import extract, result_from_vector

CKPT = "traj_opt/reference/backflip.npy"
N_SUB = 40  # samples per collocation interval


def main():
    x = np.load(CKPT)
    bp = BackflipProgram(amom=1e-6)
    result = result_from_vector(bp, x)
    phases = extract(bp, result)

    print(f"{'phase':<8} {'interval':>8} {'h (s)':>8} {'max|q0^2+q2^2-1|':>18} "
          f"{'max|theta_flat-theta_angle| (rad)':>34}")

    grand_max_norm = 0.0
    grand_max_ang = 0.0
    worst = None

    for ph in phases:
        t, traj = ph["t"], ph["traj"]
        n = len(t) - 1
        for k in range(n):
            t0, t1 = t[k] - ph["t0"], t[k + 1] - ph["t0"]
            h = t1 - t0
            ss = np.linspace(t0, t1, N_SUB)

            # The actual flat cubic Hermite the NLP's constraints and the audit both see.
            q0q2 = np.array([traj.value(s).ravel()[[0, 2]] for s in ss])
            norm2 = q0q2[:, 0] ** 2 + q0q2[:, 1] ** 2
            norm_defect = np.abs(norm2 - 1.0)

            # The alternative: treat the (already sagittally-collapsed) rotation as a scalar
            # angle theta = 2*atan2(q2, q0) and cubic-Hermite-interpolate THAT directly, using
            # the knot angles and the knot pitch rate v[1] (= wy = dtheta/dt for this axis).
            x0, x1 = traj.value(t0).ravel(), traj.value(t1).ravel()
            th0 = 2 * np.arctan2(x0[XQ][2], x0[XQ][0])
            th1 = 2 * np.arctan2(x1[XQ][2], x1[XQ][0])
            wy0, wy1 = x0[XV][1], x1[XV][1]
            # unwrap: th1 should be reached by integrating wy across the interval, not by the
            # atan2 branch -- use th0 + h*wy0 as a same-branch reference to pick the winding.
            th1 = th0 + np.round((th0 + h * 0.5 * (wy0 + wy1) - th1) / (2 * np.pi)) * 2 * np.pi + (th1 - th0)
            # cubic Hermite on theta(t) with slopes wy0, wy1 over [0, h]
            tau = (ss - t0) / h
            h00 = 2 * tau ** 3 - 3 * tau ** 2 + 1
            h10 = tau ** 3 - 2 * tau ** 2 + tau
            h01 = -2 * tau ** 3 + 3 * tau ** 2
            h11 = tau ** 3 - tau ** 2
            th_angle = h00 * th0 + h10 * h * wy0 + h01 * th1 + h11 * h * wy1

            th_flat = 2 * np.arctan2(q0q2[:, 1], q0q2[:, 0])
            # keep th_flat on the same branch as th_angle
            th_flat = th_flat + np.round((th_angle - th_flat) / (2 * np.pi)) * 2 * np.pi
            ang_defect = np.abs(th_flat - th_angle)

            mnd, mad = norm_defect.max(), ang_defect.max()
            grand_max_norm = max(grand_max_norm, mnd)
            if mad > grand_max_ang:
                grand_max_ang = mad
                worst = (ph["name"], k, h)
            if mnd > 1e-6 or mad > 1e-4:
                print(f"{ph['name']:<8} {k:>8} {h:>8.4f} {mnd:>18.3e} {mad:>34.3e}")

    print()
    print(f"grand max |q0^2+q2^2 - 1| between knots: {grand_max_norm:.3e}  "
          f"(QUAT_BOX at knots is 1e-4)")
    print(f"grand max |theta_flat - theta_angle| between knots: {grand_max_ang:.3e} rad "
          f"({np.degrees(grand_max_ang):.4f} deg), worst at {worst}")


if __name__ == "__main__":
    raise SystemExit(main())
