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


def set_guess(bp: BackflipProgram, g: list[dict], footholds):
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
    bp.prog.SetInitialGuess(bp.impulse, np.tile([0.0, 0.0, 12.0], (4, 1)))


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
          f"cost={result.get_optimal_cost():.4f} in {time.time() - t0:.1f}s")
    return result


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
            lam=result.GetSolution(bp.lam[p]) if ph.contacts else np.zeros((ph.n_knots, 0, 3)),
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
    args = ap.parse_args()

    bp = BackflipProgram()
    print(f"program: {bp.prog.num_vars()} vars, {len(bp.prog.GetAllConstraints())} constraints")

    g = Guess(bp.plant)
    set_guess(bp, g.build(), g.footholds())

    make_opts = snopt_options if args.solver == "snopt" else ipopt_options
    result = solve(bp, make_opts(args.feas_tol, args.opt_tol, args.iters), "feasibility", args.solver)
    if not args.feasibility_only:
        bp.add_cost()
        bp.prog.SetInitialGuess(bp.prog.decision_variables(),
                                result.GetSolution(bp.prog.decision_variables()))
        result = solve(bp, make_opts(args.feas_tol, args.opt_tol / 10, args.iters), "optimal", args.solver)

    if not result.is_success():
        print("NOT SOLVED -- writing nothing")
        return 1

    phases = extract(bp, result)
    ok = audit.run(bp, result, phases)
    t, x, u = resample(phases)
    save(t, x, u)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
