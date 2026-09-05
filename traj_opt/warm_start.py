"""Mesh-refinement warm-starting: export a solved/checkpointed trajectory as a portable,
knot-count-independent guess, so growing a phase's knot count doesn't mean going back to
guess.py's analytic guess and re-discovering a good point from scratch.

A phase whose knot count is unchanged is copied straight out of the solution and round-trips
BIT-EXACTLY (the shipped 50-knot point comes back at violation 0.128651, audit 10/11, worst
1.61x -- its own numbers). A phase being grown keeps the source's time grid shape and takes
its states from re-shooting the true dynamics through the source's own first-order-held
generalized force, restarting each shot at the nearest source knot.

PREFER A KNOT COUNT OF 2n-1. The new grid is then a superset of the old one, so the resampled
force is the source's force exactly rather than a piecewise-linear approximation of it, and
every original knot keeps its solved state. Flight torques swing ~13 N.m between adjacent
knots on this problem, so a non-aligned grid distorts the force badly: refining 50 -> 76 left
the guess at violation 587, while the bisection 50 -> 99 gives 18.

    uv run traj_opt/warm_start.py --checkpoint traj_opt/reference/backflip.npy \
            --out traj_opt/out/refine/warm99.npz --flight-knots 99

Run this BEFORE editing traj_opt/schedule.py's knot counts -- it must build the OLD
BackflipProgram to interpret the checkpoint vector, so the schedule on disk at export
time has to still match the schedule the checkpoint was solved under.

None of the above was true until 2026-09-05: this file round-tripped a solved point to
violation 321, and STATUS.md recorded that as warm-starting being the wrong tool for an
under-resolved source. It was not. Three separate bugs, all of them re-timing knots --
see set_guess() in solve_backflip.py and the comments in export() below.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np
from pydrake.trajectories import PiecewisePolynomial

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource, TrajectorySource

from guess import Guess
from program import BackflipProgram, make_plant
from schedule import PHASES
from solve_backflip import result_from_vector


def export(bp: BackflipProgram, result, out_path: str, new_n_knots: dict[int, int]) -> None:
    g_helper = Guess(bp.plant)  # for _gen_force and _simulate
    nq = bp.plant.num_positions()
    data = {}
    for p, ph in enumerate(PHASES):
        nc = len(ph.contacts)
        ts = bp.dc[p].GetSampleTimes(result)
        u_old = result.GetSolution(bp.u[p])
        lam_old = (result.GetSolution(bp.lam[p].reshape(ph.n_knots, -1)).reshape(ph.n_knots, nc, 3)
                   if nc else np.zeros((ph.n_knots, 0, 3)))
        x_old = np.array([result.GetSolution(bp.dc[p].state(k)) for k in range(ph.n_knots)])
        gen_old = np.array([result.GetSolution(bp.dc[p].input(k)) for k in range(ph.n_knots)])

        n_new = new_n_knots.get(p, ph.n_knots)
        if n_new == ph.n_knots:
            # Nothing to refine: hand back the solved knots untouched. Resampling a phase onto
            # its own grid is a no-op in exact arithmetic but not in Drake's, and only the phase
            # actually being grown should pay any interpolation error at all.
            t_new, x_new, u_new, lam_new, gen_new = ts, x_old, u_old, lam_old, gen_old
        else:
            # Resample the GRID, not just the curve. The equal-time-interval constraints are a
            # chain of adjacent equalities (h_k == h_k+1), each satisfied to ~1e-4 with nothing
            # bounding the accumulated drift, so a solved grid is never uniform -- the shipped
            # 50-knot flight phase runs h from 0.011469 to 0.013467, 17% off. Interpolating the
            # old grid against index fraction preserves its shape at the new resolution.
            s_old = np.linspace(0.0, 1.0, ph.n_knots)
            t_new = np.interp(np.linspace(0.0, 1.0, n_new), s_old, ts)
            u_new = _foh(ts, u_old, t_new)
            lam_new = (_foh(ts, lam_old.reshape(ph.n_knots, -1), t_new).reshape(n_new, nc, 3)
                       if nc else np.zeros((n_new, 0, 3)))
            # States come from FORWARD-INTEGRATING the true dynamics through the same
            # first-order-held generalized force the transcription itself applies -- not from
            # sampling DirectCollocation's reconstruction spline, which is what this used to do.
            # The knots of a solved phase are a genuine trajectory (audit's integration check
            # confirms it), but the cubic through them is only an interpolant: sampling it
            # between knots broke sagittal symmetry from 2.8e-4 to 3.1e-2 rad and left the
            # refined guess at violation 588. Re-simulating is exactly the single-shooting step
            # guess.py already uses for launch/flight, and for the same reason.
            x_new = _shoot(ts, gen_old, x_old, t_new, nq)
            # Recompute the port force from the resampled knots so the guess satisfies the
            # port constraint (gen == B@u + sum J^T lambda) exactly. Contact-free phases make
            # this identical to gen_ref, since there gen is just B@u and B is constant.
            gen_new = np.array([g_helper._gen_force(x_new[k, :nq], u_new[k], ph.contacts,
                                                    lam_new[k]) for k in range(n_new)])

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


def _shoot(ts, gen_old, x_old, t_new, nq):
    """States at t_new, by re-shooting the true dynamics from each SOURCE knot.

    Every new knot is integrated from the nearest source knot at or before it, through the
    generalized force held over the SOURCE grid -- the identical piecewise-linear function
    audit.check_integration validates the source solve against. Two things follow. A new time
    that coincides with a source knot gets that knot's solved state back exactly, so a
    refinement whose grid contains the old one (bisection, n_new == 2n-1) keeps every original
    knot untouched. And error never accumulates: each shot is at most one source interval long
    instead of the whole phase. Shooting straight across the flight phase instead left the
    99-knot guess at violation 17.4 with 7.0e-2 of integration drift; restarting each shot
    takes that to the source solve's own numbers.
    """
    builder = DiagramBuilder()
    plant = builder.AddSystem(make_plant())
    src = builder.AddSystem(TrajectorySource(PiecewisePolynomial.FirstOrderHold(ts, gen_old.T)))
    zero = builder.AddSystem(ConstantVectorSource(np.zeros(12)))
    builder.Connect(src.get_output_port(), plant.GetInputPort("applied_generalized_force"))
    builder.Connect(zero.get_output_port(), plant.get_actuation_input_port())
    diagram = builder.Build()

    out = np.empty((len(t_new), x_old.shape[1]))
    # searchsorted(side="right") - 1 is the source interval each new time falls in; a new time
    # sitting exactly on a source knot resolves to that knot, i.e. a zero-length shot. The
    # upper clip is len(ts)-1, NOT len(ts)-2: t_new[-1] lands exactly on the final source knot,
    # and clipping to the last INTERVAL instead sends it through a full shot across that
    # interval rather than pinning it. That is the one knot the flight->absorb stitching
    # constraint reads, and it was coming out 4.3e-2 off while every other knot was exact.
    # Whenever tk is strictly past ts[i] the index is below the last by construction, so the
    # shot below never indexes out of range.
    src_of = np.clip(np.searchsorted(ts, t_new, side="right") - 1, 0, len(ts) - 1)
    for j, tk in enumerate(t_new):
        i = src_of[j]
        if np.isclose(tk, ts[i], rtol=0.0, atol=1e-12):
            out[j] = x_old[i]
            continue
        sim = Simulator(diagram)
        sim.get_mutable_integrator().set_target_accuracy(1e-10)
        sim.get_mutable_context().SetTime(ts[i])
        ctx = plant.GetMyMutableContextFromRoot(sim.get_mutable_context())
        plant.SetPositions(ctx, x_old[i][:nq])
        plant.SetVelocities(ctx, x_old[i][nq:])
        sim.Initialize()
        sim.AdvanceTo(tk)
        out[j] = np.concatenate([plant.GetPositions(ctx), plant.GetVelocities(ctx)])
    return out


def _foh(ts, vals, t_new):
    """First-order hold of `vals` (knots x m) sampled at t_new -- the transcription's own
    interpolation for everything that is not a state."""
    hold = PiecewisePolynomial.FirstOrderHold(ts, np.asarray(vals).T)
    return np.array([hold.value(tk).ravel() for tk in t_new])


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
