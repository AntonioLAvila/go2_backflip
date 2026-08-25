"""Mesh-refinement warm-starting: export a solved/checkpointed trajectory as a portable,
knot-count-independent guess, so growing a phase's knot count doesn't mean going back to
guess.py's analytic guess and re-discovering a good point from scratch.

Export resamples each phase's CONTINUOUS reconstruction (state via the Hermite-Simpson
spline DirectCollocation itself builds, u/lambda via first-order hold, matching the
transcription's own interpolation assumptions) at a new set of knot times, so a phase
whose knot count is unchanged round-trips through this essentially exactly, and a phase
being grown gets new interior points sampled off the same curve rather than any new
information invented -- the risk this defuses is a *worse* guess, not a materially
different one.

    uv run traj_opt/warm_start.py --checkpoint traj_opt/out/checkpoints/best.npy \
            --out traj_opt/out/checkpoints/warm_flight20.npz --flight-knots 20

Run this BEFORE editing traj_opt/schedule.py's knot counts -- it must build the OLD
BackflipProgram to interpret the checkpoint vector, so the schedule on disk at export
time has to still match the schedule the checkpoint was solved under.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
from pydrake.trajectories import PiecewisePolynomial

from guess import Guess
from program import BackflipProgram
from schedule import PHASES
from solve_backflip import result_from_vector


def export(bp: BackflipProgram, result, out_path: str, new_n_knots: dict[int, int]) -> None:
    g_helper = Guess(bp.plant)  # only used for its _gen_force/_jac physics helpers
    data = {}
    for p, ph in enumerate(PHASES):
        nc = len(ph.contacts)
        ts = bp.dc[p].GetSampleTimes(result)
        traj = bp.dc[p].ReconstructStateTrajectory(result)
        u_old = result.GetSolution(bp.u[p])
        lam_old = (result.GetSolution(bp.lam[p].reshape(ph.n_knots, -1)).reshape(ph.n_knots, nc, 3)
                   if nc else np.zeros((ph.n_knots, 0, 3)))

        n_new = new_n_knots.get(p, ph.n_knots)
        t_new = np.linspace(ts[0], ts[-1], n_new)
        x_new = np.array([traj.value(tk).ravel() for tk in t_new])
        u_hold = PiecewisePolynomial.FirstOrderHold(ts, u_old.T)
        u_new = np.array([u_hold.value(tk).ravel() for tk in t_new])
        if nc:
            lam_hold = PiecewisePolynomial.FirstOrderHold(ts, lam_old.reshape(ph.n_knots, -1).T)
            lam_new = np.array([lam_hold.value(tk).ravel() for tk in t_new]).reshape(n_new, nc, 3)
        else:
            lam_new = np.zeros((n_new, 0, 3))
        gen_new = np.array([g_helper._gen_force(x_new[k, :19], u_new[k], ph.contacts, lam_new[k])
                            for k in range(n_new)])

        data[f"t_{p}"] = t_new - t_new[0]
        data[f"x_{p}"] = x_new
        data[f"u_{p}"] = u_new
        data[f"lam_{p}"] = lam_new.reshape(n_new, -1)
        data[f"gen_{p}"] = gen_new
        data[f"nc_{p}"] = nc
        data[f"foothold_{p}"] = result.GetSolution(bp.foothold[p]) if nc else np.zeros((0, 3))
    data["impulse"] = result.GetSolution(bp.impulse)
    np.savez(out_path, **data)
    print(f"wrote {out_path}: knots {[new_n_knots.get(p, ph.n_knots) for p, ph in enumerate(PHASES)]}")


def load(path: str):
    """(g, footholds, impulse) in exactly the shape set_guess()/main() expect."""
    d = np.load(path)
    g, footholds = [], []
    for p, ph in enumerate(PHASES):
        nc, n = int(d[f"nc_{p}"]), d[f"t_{p}"].shape[0]
        g.append(dict(t=d[f"t_{p}"], x=d[f"x_{p}"], u=d[f"u_{p}"],
                       lam=d[f"lam_{p}"].reshape(n, nc, 3), gen=d[f"gen_{p}"]))
        footholds.append(d[f"foothold_{p}"])
    return g, footholds, d["impulse"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True, help="raw decision-variable .npy, "
                    "e.g. from traj_opt/out/checkpoints/, at the CURRENT on-disk schedule")
    ap.add_argument("--out", required=True)
    ap.add_argument("--flight-knots", type=int, default=None)
    ap.add_argument("--load-knots", type=int, default=None)
    ap.add_argument("--launch-knots", type=int, default=None)
    ap.add_argument("--absorb-knots", type=int, default=None)
    args = ap.parse_args()

    bp = BackflipProgram()
    x = np.load(args.checkpoint)
    if x.size != bp.prog.num_vars():
        raise SystemExit(f"checkpoint has {x.size} vars but the on-disk schedule builds "
                          f"a program with {bp.prog.num_vars()} -- schedule.py has already "
                          f"changed since this checkpoint was solved; export first, then edit.")
    result = result_from_vector(bp, x)

    new_n_knots = {}
    for name, val in (("load", args.load_knots), ("launch", args.launch_knots),
                      ("flight", args.flight_knots), ("absorb", args.absorb_knots)):
        if val is not None:
            new_n_knots[[p.name for p in PHASES].index(name)] = val
    export(bp, result, args.out, new_n_knots)
    return 0


if __name__ == "__main__":
    sys.exit(main())
