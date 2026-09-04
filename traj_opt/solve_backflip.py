"""Solve the backflip and write traj_opt/out/backflip.npz in MuJoCo convention.

    uv run traj_opt/solve_backflip.py
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np
from pydrake.solvers import IpoptSolver, SnoptSolver, SolverOptions
from pydrake.trajectories import PiecewisePolynomial

from go2_backflip import constants as K
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


# Extra IPOPT options from --ipopt-opt, applied last so they can override anything below.
# The two failing audit checks are both proxies for unconverged collocation defects (see
# STATUS.md), so the levers that matter now are IPOPT's own, not the transcription's.
IPOPT_EXTRA: dict[str, object] = {}


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
    # IPOPT moves any variable sitting within bound_push of a bound into the interior BEFORE
    # iteration 0. At the default 0.01 that move is catastrophic here: re-entering from the
    # best known point measured viol 0.0495 -> 16.76, because 282 variables sit on bounds at
    # any good point (launch torques on their +-45 N.m limit, foot pins, the quaternion
    # boxes). The damage is linear in the setting -- 1e-3 -> 1.67, 1e-4 -> 0.157 -- and at
    # 1e-8 the point survives bit-for-bit. This matters most for restart_loop, which chains
    # forward by re-solving from the previous burst's vector: every burst was starting ~340x
    # worse than the point it had just saved, which is what "IPOPT reliably wanders away from
    # good points" turned out to be.
    for k in ("bound_push", "bound_frac", "slack_bound_push", "slack_bound_frac"):
        o.SetOption(sid, k, 1e-8)
    o.SetOption(sid, "print_level", 5)
    for k, v in IPOPT_EXTRA.items():
        o.SetOption(sid, k, v)
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


def add_proximal_cost(bp: BackflipProgram, x_ref: np.ndarray, weight: float):
    """min ||x - x_ref||^2 alongside the constraints, instead of a flat feasibility problem.

    --feasibility-only leaves the objective identically zero, and on THIS problem that is a
    liability rather than a simplification: the active set at a solved point is rank-deficient
    by ~200 (cond(A) = 5e20, measured), so there is a large subspace the constraints do not
    pin down, and a flat objective gives IPOPT's limited-memory quasi-Newton approximation
    nothing at all to resolve it with -- which is what a 6e5-norm Newton step cut back to
    alpha ~ 1e-9 looks like. A proximal term makes the reduced Hessian the identity on exactly
    that subspace, and it pulls toward a point already known to be physically good.

    Weighted per variable by 1/max(1, x_ref)^2, because the vector spans h ~ 0.02 to lambda
    ~ 200 and a uniform weight would be a de-facto constraint on the contact forces alone.
    Added in blocks: one binding per variable would put 4934 of them in every evaluation, and
    a single dense 4934x4934 Q is 195 MB.
    """
    v = bp.prog.decision_variables()
    scale = weight / np.maximum(1.0, np.abs(x_ref)) ** 2
    for i in range(0, v.size, 250):
        blk = slice(i, min(i + 250, v.size))
        bp.prog.AddQuadraticErrorCost(np.diag(scale[blk]), x_ref[blk], v[blk])
    print(f"proximal cost: weight {weight:g} toward the seed, {(v.size + 249) // 250} blocks")


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


def checkpoint_dir(out: Path) -> Path:
    """Where a run writes its bursts. Derived from --out so two independent restart searches
    can run side by side without overwriting each other's checkpoints.

    A concurrent run is only a real second sample if something about it DIFFERS -- a different
    seed, --burst-iters, tolerance or --cost-scale. IPOPT is deterministic here: a run seeded
    from another run's checkpoint with identical options reproduces its chain bit-for-bit
    (measured 2026-09-04, nine bursts identical to the digit). This docstring used to claim the
    search was stochastic in practice. It is not."""
    return out.parent / ("checkpoints" if out == OUT else f"checkpoints_{out.stem}")


def restart_loop(bp: BackflipProgram, make_opts, solver: str, feas_tol: float, opt_tol: float,
                  burst_iters: int, n_restarts: int, result, ckpt_dir: Path):
    """Re-solve in short bursts, chaining forward from each burst's raw result.

    Continuous runs reliably wander away from good points once reached (confirmed
    repeatedly on this problem -- see STATUS.md); restarting resets IPOPT's own internal
    state each burst and, empirically, finds much better points than any single long run.
    Reverting to the best-seen point between bursts does NOT help -- IPOPT is deterministic
    given the same start and options, so that just reproduces the same result every time.
    Chaining forward through temporary regressions is what actually explores new territory.
    Every burst's checkpoint is saved to disk immediately so a good point is never lost to a
    worse one two bursts later.

    Bursts are RANKED BY THE AUDIT, not by max_violation. That is the 2026-09-04 change, and
    it exists because the two anti-correlate here: a search run to 0.0004 -- 124x below the
    0.0495 point this repo ships -- audits 7/11 against the shipped point's 9/11, with 5x the
    integration drift and a 0.131 deg pitch reversal. It is a spurious discrete solution, a
    cubic ringing between collocation points while satisfying the defects exactly AT them,
    and max_violation is blind to it by construction because it only ever looks at the knots.
    Ranking is on (checks failed, worst overrun ratio, violation), lexicographic -- not on a
    weighted sum, which over these eleven checks would be adding radians to metres to N.m.s to
    kg.m^2 and would rank fine while meaning nothing. The chain still runs on the raw result of
    every burst regardless of how it ranks -- selection and exploration are separate, and
    filtering what gets chained would collapse the search back to a deterministic fixed point.
    """
    ckpt_dir.mkdir(parents=True, exist_ok=True)

    def rank(res):
        """(checks failed, worst overrun, violation). Lexicographic; violation breaks ties.

        A burst can be bad enough that this throws -- a phase whose duration went
        non-positive, or an all-zero quaternion, which Drake refuses to convert to a rotation
        matrix. That is a legitimate "worst possible" answer, not a crash worth losing a
        40-burst search over, so max_violation is inside the guard too: it evaluates every
        constraint at the point, so it throws on exactly the same degeneracies the audit does.
        """
        try:
            viol = max_violation(bp.prog, res)
        except Exception as e:                                  # noqa: BLE001
            inf = float("inf")
            return (99, inf, inf), f"unevaluable ({type(e).__name__}: {e})"
        try:
            a = audit.run(bp, res, extract(bp, res), quiet=True)
            return a.key + (viol,), f"viol={viol:.4f} audit={a}"
        except Exception as e:                                  # noqa: BLE001
            inf = float("inf")
            return (99, inf, viol), f"viol={viol:.4f} audit=FAILED ({type(e).__name__}: {e})"

    best_key, best_desc = rank(result)
    best_result = result
    best_viol_key = max_violation(bp.prog, result)
    x0 = result.GetSolution(bp.prog.decision_variables())
    np.save(ckpt_dir / "burst_start.npy", x0)
    # Start burst 0 from the point we were HANDED, not from whatever guess is still sitting on
    # the program. Only the costed path re-seeded it after the feasibility pass, so
    # --feasibility-only spent its first burst restarting from guess.py -- measured at
    # inf_pr 99.7 against the 0.0025 the pass had just reached, and the chain then carries that
    # burst's result forward, so it is the whole search that starts in the wrong place.
    bp.prog.SetInitialGuess(bp.prog.decision_variables(), x0)
    np.save(ckpt_dir / "best.npy", x0)
    np.save(ckpt_dir / "best_viol.npy", x0)
    print(f"  restart loop starting from {best_desc}")

    for r in range(n_restarts):
        result = solve(bp, make_opts(feas_tol, opt_tol, burst_iters), f"restart {r}", solver)
        x = result.GetSolution(bp.prog.decision_variables())
        np.save(ckpt_dir / f"burst_{r}.npy", x)
        key, desc = rank(result)
        print(f"    {desc}")
        if key < best_key:
            best_key, best_result, best_desc = key, result, desc
            np.save(ckpt_dir / "best.npy", x)
            print(f"  new best: {desc}")
        # The old criterion, kept alongside rather than dropped. It is the number every
        # earlier run in STATUS.md is quoted in, and keeping it costs one file: without it a
        # search under the new rule could not be compared against any of them.
        if key[2] < best_viol_key:
            best_viol_key = key[2]
            np.save(ckpt_dir / "best_viol.npy", x)
        bp.prog.SetInitialGuess(bp.prog.decision_variables(), x)   # chain forward, always
        if result.is_success():
            # Has never happened on this problem. If it does, best.npy must be the point that
            # is returned, not whatever scored best before it -- the two files and the caller's
            # trajectory have to describe the same solve.
            np.save(ckpt_dir / "best.npy", x)
            print(f"  SOLVED on restart {r}: {desc}")
            return result
    print(f"  restart loop best: {best_desc} "
          f"(lowest violation seen was {best_viol_key:.4f}, in best_viol.npy)")
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


def save(t, x, u, out: Path = OUT):
    q, v = x[:, XQ], x[:, XV]
    qpos = np.array([K.drake_to_mj_q(qi) for qi in q])
    qvel = np.array([K.drake_to_mj_v(vi, qi[:4]) for qi, vi in zip(q, v)])
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez(out, t=t, qpos=qpos, qvel=qvel, ctrl=u)
    print(f"wrote {out}  {t.size} samples @ {RATE:g} Hz, {t[-1]:.3f} s")


def main() -> int:
    # These runs are hours long and are watched by tailing a redirected log. Python
    # block-buffers stdout to a file, so the per-burst rankings -- the only lines worth
    # watching -- arrive in 8 KB clumps well behind the solve. IPOPT's own output is written
    # from C++ and is unaffected either way.
    sys.stdout.reconfigure(line_buffering=True)
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
    ap.add_argument("--from-checkpoint", type=Path, default=None,
                     help="skip solving: wrap a restart-loop checkpoint (out/checkpoints/*.npy) "
                          "as a result and run the audit/resample/save pipeline on it. Lets a "
                          "good point from a still-running solve be replayed and audited "
                          "without waiting for, or disturbing, that run -- pair it with --out")
    ap.add_argument("--start-checkpoint", type=Path, default=None,
                     help="seed the solve from a checkpoint of THIS formulation instead of "
                          "guess.py, then solve/restart normally. Unlike --warm-start there is "
                          "no resampling: the vector is used as-is, so it only works at the "
                          "same knot counts. Give it different --burst-iters than the run that "
                          "produced it -- IPOPT is deterministic from a fixed start and "
                          "options, so an identical start reproduces the identical chain")
    ap.add_argument("--no-prepass", action="store_true",
                    help="With --start-checkpoint: add the cost and go straight into the "
                         "restart bursts, skipping the feasibility and costed passes. Those "
                         "two passes are long continuous solves, which is exactly what the "
                         "burst structure exists to avoid -- seeded with the 16.05-scoring "
                         "burst_38 they handed the restart loop a 41.97 instead, and the "
                         "search then spent bursts climbing back to where it started.")
    ap.add_argument("--cost-scale", type=float, default=1.0, metavar="S",
                     help="scale the performance costs (torque, rate, time, tuck) in the "
                          "costed pass. The symmetry cost is never scaled. At the default 1.0 "
                          "the costed pass walks off the feasible manifold (0.0025 -> 0.73); "
                          "use 0 for a symmetry-only pass that keeps feasibility")
    ap.add_argument("--proximal", type=float, default=0.0, metavar="W",
                     help="add W*||x - seed||^2 (per-variable normalised) to the objective, "
                          "with the seed from --start-checkpoint. Regularises the degenerate "
                          "null space that a pure feasibility pass leaves flat")
    ap.add_argument("--ipopt-opt", action="append", default=[], metavar="KEY=VALUE",
                     help="extra IPOPT option, repeatable; ints/floats are parsed as such, "
                          "everything else passed as a string (e.g. mu_strategy=adaptive)")
    ap.add_argument("--out", type=Path, default=OUT,
                     help="where to write the trajectory (default traj_opt/out/backflip.npz)")
    ap.add_argument("--warm-start", type=str, default=None,
                     help="traj_opt/warm_start.py-exported .npz to seed the guess from "
                          "(a resampled prior solve/checkpoint) instead of guess.py's "
                          "analytic guess -- for mesh-refinement continuation")
    args = ap.parse_args()

    for kv in args.ipopt_opt:
        k, _, v = kv.partition("=")
        for cast in (int, float, str):
            try:
                IPOPT_EXTRA[k] = cast(v)
                break
            except ValueError:
                continue
    if IPOPT_EXTRA:
        print(f"ipopt: {IPOPT_EXTRA}")

    bp = BackflipProgram()
    print(f"program: {bp.prog.num_vars()} vars, {len(bp.prog.GetAllConstraints())} constraints")

    def load_checkpoint(path: Path):
        x = np.load(path)
        if x.shape != (bp.prog.num_vars(),):
            raise SystemExit(f"{path}: {x.shape} decision variables, this program has "
                             f"{bp.prog.num_vars()} -- a different schedule or formulation")
        return x

    if args.from_checkpoint:
        result = result_from_vector(bp, load_checkpoint(args.from_checkpoint))
        print(f"[{args.from_checkpoint.name}] viol={max_violation(bp.prog, result):.4f}")
        phases = extract(bp, result)
        ok = audit.run(bp, result, phases).ok
        t, xs, u = resample(phases)
        save(t, xs, u, args.out)
        return 0 if ok else 2

    if args.start_checkpoint:
        x = load_checkpoint(args.start_checkpoint)
        bp.prog.SetInitialGuess(bp.prog.decision_variables(), x)
        print(f"seeded from {args.start_checkpoint.name}")
        if args.proximal:
            add_proximal_cost(bp, x, args.proximal)
    elif args.warm_start:
        import warm_start
        g, footholds, impulse = warm_start.load(args.warm_start)
        set_guess(bp, g, footholds, impulse)
    else:
        g = Guess(bp.plant)
        set_guess(bp, g.build(), g.footholds())

    make_opts = snopt_options if args.solver == "snopt" else ipopt_options
    if args.no_prepass:
        if not args.start_checkpoint:
            raise SystemExit("--no-prepass only means anything with --start-checkpoint: "
                             "without a start point there is nothing to preserve")
        if not args.restarts:
            raise SystemExit("--no-prepass with no --restarts would solve nothing at all")
        if not args.feasibility_only:
            bp.add_cost(scale=args.cost_scale)
        # Bit-exact (see result_from_vector), so the restart loop begins on the point that was
        # handed in rather than on whatever two long continuous solves made of it.
        result = result_from_vector(bp, x)
    else:
        result = solve(bp, make_opts(args.feas_tol, args.opt_tol, args.iters),
                       "feasibility", args.solver)
        if not args.feasibility_only:
            bp.add_cost(scale=args.cost_scale)
            bp.prog.SetInitialGuess(bp.prog.decision_variables(),
                                    result.GetSolution(bp.prog.decision_variables()))
            result = solve(bp, make_opts(args.feas_tol, args.opt_tol / 10, args.iters),
                           "optimal", args.solver)

    if args.restarts and not result.is_success():
        result = restart_loop(bp, make_opts, args.solver, args.feas_tol, args.opt_tol / 10,
                               args.burst_iters, args.restarts, result,
                               checkpoint_dir(args.out))

    if not result.is_success():
        viol = max_violation(bp.prog, result)
        print(f"NOT SOLVED (best viol={viol:.4f}) -- writing best-effort output anyway "
              f"for inspection, do not treat as a validated trajectory")
        phases = extract(bp, result)
        audit.run(bp, result, phases)
        t, x, u = resample(phases)
        save(t, x, u, args.out)
        return 1

    phases = extract(bp, result)
    ok = audit.run(bp, result, phases).ok
    t, x, u = resample(phases)
    save(t, x, u, args.out)
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
