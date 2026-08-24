"""Solve the backflip and write traj_opt/out/backflip.npz in MuJoCo convention.

    PYTHONPATH=/opt/drake/lib/python3.12/site-packages:tools:traj_opt \
        .venv/bin/python traj_opt/solve_backflip.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from pydrake.solvers import IpoptSolver, SnoptSolver, SolverOptions
from pydrake.trajectories import PiecewisePolynomial

import go2_constants as K
import audit
from guess import Guess
from program import BackflipProgram, XQ, XV
from schedule import PHASES

OUT = Path(__file__).resolve().parent / "out" / "backflip.npz"
RATE = 500.0


def set_guess(bp: BackflipProgram, g: list[dict], footholds, impulse=None):
    for p, ph in enumerate(PHASES):
        t = g[p]["t"]
        bp.dc[p].SetInitialTrajectory(
            PiecewisePolynomial.FirstOrderHold(t, g[p]["gen"].T),
            PiecewisePolynomial.FirstOrderHold(t, g[p]["x"].T))
        bp.prog.SetInitialGuess(bp.u[p], g[p]["u"])
        if ph.contacts:
            n = ph.n_knots
            bp.prog.SetInitialGuess(bp.lam[p].reshape(n, -1),
                                    g[p]["lam"].reshape(n, -1))
            bp.prog.SetInitialGuess(bp.foothold[p], footholds[p])
    bp.prog.SetInitialGuess(bp.impulse,
                            impulse if impulse is not None else np.tile([0.0, 0.0, 12.0], (4, 1)))


def ipopt_options(feas: float, opt: float, iters: int) -> SolverOptions:
    o = SolverOptions()
    sid = IpoptSolver.id()
    o.SetOption(sid, "tol", opt)
    o.SetOption(sid, "constr_viol_tol", feas)
    o.SetOption(sid, "acceptable_tol", opt * 10)
    o.SetOption(sid, "acceptable_constr_viol_tol", feas * 10)
    o.SetOption(sid, "max_iter", iters)
    # No exact Hessian is supplied (constraints only give first derivatives via autodiff),
    # so IPOPT must build its own quasi-Newton approximation instead of the exact one it
    # defaults to expecting.
    o.SetOption(sid, "hessian_approximation", "limited-memory")
    o.SetOption(sid, "print_level", 5)
    return o


def snopt_options(feas: float, opt: float, iters: int) -> SolverOptions:
    o = SolverOptions()
    sid = SnoptSolver.id()
    o.SetOption(sid, "Major feasibility tolerance", feas)
    o.SetOption(sid, "Major optimality tolerance", opt)
    o.SetOption(sid, "Major iterations limit", iters)
    o.SetOption(sid, "Iterations limit", 1_000_000)
    o.SetOption(sid, "Superbasics limit", 6000)
    o.SetOption(sid, "Scale option", 2)
    return o


def solve(bp: BackflipProgram, options, label: str, solver: str = "ipopt"):
    t0 = time.time()
    if solver == "snopt":
        result = SnoptSolver().Solve(bp.prog, bp.prog.initial_guess(), options)
        status = f"SNOPT info={result.get_solver_details().info}"
    else:
        result = IpoptSolver().Solve(bp.prog, bp.prog.initial_guess(), options)
        status = f"IPOPT status={result.get_solver_details().ConvertStatusToString()}"
    print(f"[{label}] success={result.is_success()} {status} "
          f"cost={result.get_optimal_cost():.4f} viol={max_violation(bp.prog, result):.4f} "
          f"in {time.time() - t0:.1f}s")
    return result


def result_from_vector(bp: BackflipProgram, x: np.ndarray):
    """Wrap a raw decision-variable vector (e.g. a saved restart checkpoint) as a real
    MathematicalProgramResult, unchanged, so extract()/audit.run()/max_violation() all
    work on it without a fresh solve. No pydrake binding exposes
    set_decision_variable_index directly, so this goes through an actual IPOPT call
    with max_iter=0 and the initial bound-push/pull disabled (confirmed bit-exact:
    GetSolution afterwards reproduces x exactly)."""
    o = SolverOptions()
    sid = IpoptSolver.id()
    o.SetOption(sid, "max_iter", 0)
    o.SetOption(sid, "bound_push", 1e-12)
    o.SetOption(sid, "bound_frac", 1e-12)
    return IpoptSolver().Solve(bp.prog, x, o)


def max_violation(prog, result) -> float:
    """Max infeasibility over every constraint -- the number IPOPT calls unscaled constraint
    violation, computed directly so it can be checked mid-restart-loop, not just parsed from
    a printed final summary."""
    worst = 0.0
    for b in prog.GetAllConstraints():
        ev = b.evaluator()
        y = ev.Eval(result.GetSolution(b.variables()))
        lo, hi = ev.lower_bound(), ev.upper_bound()
        worst = max(worst, float(np.maximum(np.maximum(lo - y, 0.0), y - hi).max()))
    return worst


def restart_loop(bp: BackflipProgram, make_opts, solver: str, feas_tol: float, opt_tol: float,
                  burst_iters: int, n_restarts: int, result):
    """Re-solve in short bursts, chaining forward from each burst's raw result.

    Continuous runs reliably wander away from good points once reached (confirmed
    repeatedly on this problem -- see STATUS.md); restarting resets IPOPT's own internal
    state each burst and, empirically, finds much better points than any single long run.
    Reverting to the best-seen point between bursts does NOT help -- IPOPT is deterministic
    given the same start and options, so that just reproduces the same result every time.
    Chaining forward through temporary regressions is what actually explores new territory.
    Every burst's checkpoint is saved to disk immediately so a good point is never lost to a
    worse one two bursts later.
    """
    best_result, best_viol = result, max_violation(bp.prog, result)
    ckpt_dir = OUT.parent / "checkpoints"
    ckpt_dir.mkdir(parents=True, exist_ok=True)
    np.save(ckpt_dir / "burst_start.npy", result.GetSolution(bp.prog.decision_variables()))

    for r in range(n_restarts):
        result = solve(bp, make_opts(feas_tol, opt_tol, burst_iters), f"restart {r}", solver)
        x = result.GetSolution(bp.prog.decision_variables())
        np.save(ckpt_dir / f"burst_{r}.npy", x)
        viol = max_violation(bp.prog, result)
        if viol < best_viol:
            best_result, best_viol = result, viol
            np.save(ckpt_dir / "best.npy", x)
            print(f"  new best: viol={best_viol:.4f}")
        bp.prog.SetInitialGuess(bp.prog.decision_variables(), x)   # chain forward, always
        if result.is_success():
            return result
    return best_result


def extract(bp: BackflipProgram, result):
    """Per-phase knot samples plus the reconstructed state trajectory, on one global clock."""
    phases, t0 = [], 0.0
    for p, ph in enumerate(PHASES):
        ts = bp.dc[p].GetSampleTimes(result)
        phases.append(dict(
            name=ph.name,
            t=t0 + ts,
            x=bp.dc[p].GetStateSamples(result).T,
            u=result.GetSolution(bp.u[p]),
            lam=(result.GetSolution(bp.lam[p].reshape(ph.n_knots, -1))
                 .reshape(ph.n_knots, len(ph.contacts), 3)
                 if ph.contacts else np.zeros((ph.n_knots, 0, 3))),
            contacts=ph.contacts,
            traj=bp.dc[p].ReconstructStateTrajectory(result),
            t0=t0))
        t0 = phases[-1]["t"][-1]
    return phases


def resample(phases):
    t_end = phases[-1]["t"][-1]
    t = np.arange(0.0, t_end, 1.0 / RATE)
    x = np.zeros((t.size, 37))
    u = np.zeros((t.size, 12))
    for ph in phases:
        m = (t >= ph["t"][0]) & (t <= ph["t"][-1])
        # ReconstructInputTrajectory would give the generalized force, not the torques.
        uh = PiecewisePolynomial.FirstOrderHold(ph["t"], ph["u"].T)
        for i in np.flatnonzero(m):
            x[i] = ph["traj"].value(t[i] - ph["t0"]).ravel()
            u[i] = uh.value(t[i]).ravel()
    return t, x, u


def save(t, x, u):
    q, v = x[:, XQ], x[:, XV]
    qpos = np.array([K.drake_to_mj_q(qi) for qi in q])
    qvel = np.array([K.drake_to_mj_v(vi, qi[:4]) for qi, vi in zip(q, v)])
    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez(OUT, t=t, qpos=qpos, qvel=qvel, ctrl=u)
    print(f"wrote {OUT}  {t.size} samples @ {RATE:g} Hz, {t[-1]:.3f} s")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--solver", choices=["ipopt", "snopt"], default="ipopt")
    ap.add_argument("--iters", type=int, default=3000)
    ap.add_argument("--feas-tol", type=float, default=1e-6)
    ap.add_argument("--opt-tol", type=float, default=1e-3)
    ap.add_argument("--feasibility-only", action="store_true")
    ap.add_argument("--restarts", type=int, default=0,
                     help="extra short-burst restarts after the initial solve, chaining "
                          "forward from each burst's result -- the single biggest lever "
                          "found for this problem so far, see STATUS.md")
    ap.add_argument("--burst-iters", type=int, default=300)
    ap.add_argument("--warm-start", type=str, default=None,
                     help="traj_opt/warm_start.py-exported .npz to seed the guess from "
                          "(a resampled prior solve/checkpoint) instead of guess.py's "
                          "analytic guess -- for mesh-refinement continuation")
    args = ap.parse_args()

    bp = BackflipProgram()
    print(f"program: {bp.prog.num_vars()} vars, {len(bp.prog.GetAllConstraints())} constraints")

    if args.warm_start:
        import warm_start
        g, footholds, impulse = warm_start.load(args.warm_start)
        set_guess(bp, g, footholds, impulse)
    else:
        g = Guess(bp.plant)
        set_guess(bp, g.build(), g.footholds())

    make_opts = snopt_options if args.solver == "snopt" else ipopt_options
    result = solve(bp, make_opts(args.feas_tol, args.opt_tol, args.iters), "feasibility", args.solver)
    if not args.feasibility_only:
        bp.add_cost()
        bp.prog.SetInitialGuess(bp.prog.decision_variables(),
                                result.GetSolution(bp.prog.decision_variables()))
        result = solve(bp, make_opts(args.feas_tol, args.opt_tol / 10, args.iters), "optimal", args.solver)

    if args.restarts and not result.is_success():
        result = restart_loop(bp, make_opts, args.solver, args.feas_tol, args.opt_tol / 10,
                               args.burst_iters, args.restarts, result)

    if not result.is_success():
        viol = max_violation(bp.prog, result)
        print(f"NOT SOLVED (best viol={viol:.4f}) -- writing best-effort output anyway "
              f"for inspection, do not treat as a validated trajectory")
        phases = extract(bp, result)
        audit.run(bp, result, phases)
        t, x, u = resample(phases)
        save(t, x, u)
        return 1

    phases = extract(bp, result)
    ok = audit.run(bp, result, phases)
    t, x, u = resample(phases)
    save(t, x, u)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
