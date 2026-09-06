"""Rank of the ACTIVE constraint set at a solved point — the diagnostic for `is_success()`.

`nullity_check.py` answers "will IPOPT refuse to start": it stacks only *equality* bindings and
only at the analytic guess. That is the right question for a `TOO_FEW_DOF` error and it cannot
see what stops a solve from converging. Once the optimizer drives an iterate onto a TIGHT box,
that box is an **active inequality** and is back in the KKT system, rank-deficient against the
defect rows that already imply it.

Why this is the thing to measure. At the 2026-09-06 shipped point IPOPT reports, after 250
iterations:

    Dual infeasibility......:   3.5037e+01 (scaled)
    Constraint violation....:   2.4458e-04 (scaled)
    Complementarity.........:   1.2841e-05 (scaled)

`is_success()` needs the overall NLP error under `tol`, and the overall error *is* the dual
infeasibility here — six orders above the other two. The point is essentially feasible and
essentially complementary; what it is not is **stationary**. Dual infeasibility that will not
fall, next to a primal residual that already has, is the signature of an active-constraint
Jacobian without full row rank: the multipliers that would make the Lagrangian stationary are
not determined, so IPOPT cannot drive `||grad L||` down no matter how many iterations it gets.

    uv run traj_opt/kkt_check.py [checkpoint.npy] [--active-tol 1e-6]
"""

from __future__ import annotations

import argparse
import inspect
import sys
from collections import defaultdict

import numpy as np
from pydrake.solvers import BoundingBoxConstraint, MathematicalProgram

_counters: defaultdict[str, int] = defaultdict(int)


def _tag(binding, caller):
    ev = binding.evaluator()
    if not ev.get_description():
        _counters[caller] += 1
        ev.set_description(f"{caller}#{_counters[caller]}")


def _wrap(orig):
    """Auto-tag each binding with its program.py call site, purely for readable grouping.

    Same monkeypatch as nullity_check.py and for the same reason: AddBoundingBoxConstraint and
    AddLinearConstraint carry no description, and DirectCollocation names every phase's states
    "x(i)" from 0 independently, so raw variable names collide across phases.
    """
    def wrapped(self, *a, **kw):
        f = inspect.currentframe().f_back
        caller = f"{f.f_code.co_name}:{f.f_lineno}"
        result = orig(self, *a, **kw)
        _tag(result, caller)
        return result
    return wrapped


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", nargs="?", default="traj_opt/reference/backflip.npy")
    ap.add_argument("--active-tol", type=float, default=1e-6,
                    help="an inequality row counts as active when it sits within this of its "
                         "bound. IPOPT keeps iterates strictly interior, so nothing is ever "
                         "exactly at a bound; this is what 'on the bound' has to mean")
    ap.add_argument("--flight-amom", type=float, default=None)
    ap.add_argument("--sym-penalty", type=float, default=0.0)
    ap.add_argument("--sym-box", type=float, default=None)
    ap.add_argument("--top", type=int, default=20)
    args = ap.parse_args()

    MathematicalProgram.AddBoundingBoxConstraint = _wrap(
        MathematicalProgram.AddBoundingBoxConstraint)
    MathematicalProgram.AddLinearConstraint = _wrap(MathematicalProgram.AddLinearConstraint)

    from program import AMOM_BOX, BackflipProgram

    amom = AMOM_BOX if args.flight_amom is None else (args.flight_amom or None)
    bp = BackflipProgram(amom=amom, sym_penalty=args.sym_penalty,
                         sym_box=args.sym_box)
    x = np.load(args.checkpoint)
    if x.shape != (bp.prog.num_vars(),):
        raise SystemExit(f"{args.checkpoint}: {x.shape[0]} variables, program has "
                         f"{bp.prog.num_vars()}")
    print(f"{args.checkpoint}: {bp.prog.num_vars()} vars, "
          f"{len(bp.prog.GetAllConstraints())} bindings")

    # IPOPT eliminates a variable whose bounding box has lb == ub; count rows the way it does.
    fixed = np.zeros(bp.prog.num_vars(), dtype=bool)
    for b in bp.prog.GetAllConstraints():
        ev = b.evaluator()
        if not isinstance(ev, BoundingBoxConstraint):
            continue
        same = np.abs(ev.upper_bound() - ev.lower_bound()) < 1e-12
        if np.any(same):
            fixed[np.array(bp.prog.FindDecisionVariableIndices(b.variables()))[same]] = True
    print(f"fixed variables eliminated: {int(fixed.sum())}")

    rows, labels, kinds = [], [], []
    n_eq = n_act = n_empty = 0
    eps = 1e-6
    for b in bp.prog.GetAllConstraints():
        ev = b.evaluator()
        lo, hi = ev.lower_bound(), ev.upper_bound()
        idx = bp.prog.FindDecisionVariableIndices(b.variables())
        xloc = x[idx].copy()
        y = ev.Eval(xloc)
        eq = np.abs(hi - lo) < 1e-12
        # Active-at-a-bound, either side. Equalities are always in.
        act = (~eq) & ((np.abs(y - lo) < args.active_tol) | (np.abs(y - hi) < args.active_tol))
        take = eq | act
        if not np.any(take):
            continue
        J = np.zeros((y.size, xloc.size))
        for j in range(xloc.size):
            h = eps * max(1.0, abs(xloc[j]))
            xp, xm = xloc.copy(), xloc.copy()
            xp[j] += h
            xm[j] -= h
            J[:, j] = (ev.Eval(xp) - ev.Eval(xm)) / (2 * h)
        desc = ev.get_description()
        for r in np.flatnonzero(take):
            full = np.zeros(bp.prog.num_vars())
            full[idx] = J[r]
            full[fixed] = 0.0
            if not np.any(full):
                n_empty += 1
                continue
            rows.append(full)
            labels.append(f"{desc}[{r}]")
            kinds.append("eq" if eq[r] else "act")
            n_eq += int(eq[r])
            n_act += int(not eq[r])

    M = np.array(rows)[:, ~fixed]
    print(f"rows: {M.shape[0]} ({n_eq} equality, {n_act} active inequality, "
          f"{n_empty} empty after elimination), free cols: {M.shape[1]}")

    s = np.linalg.svd(M, compute_uv=False)
    tol = s.max() * max(M.shape) * np.finfo(float).eps * 100
    nullity = int((s < tol).sum())
    cond = s.max() / s[s > 0].min()
    print(f"rank = {M.shape[0] - nullity} of {M.shape[0]} rows; "
          f"sigma max {s.max():.3e} min {s.min():.3e}; cond {cond:.2e}")
    print(f"NULLITY = {nullity}")

    if nullity:
        U, s2, _ = np.linalg.svd(M, full_matrices=False)
        energy = (U[:, np.argsort(s2)[:nullity]] ** 2).sum(axis=1)
        fam_e, fam_n = defaultdict(float), defaultdict(int)
        for lbl, kind, e in zip(labels, kinds, energy):
            fam = f"{lbl.split('#')[0]} ({kind})"
            fam_e[fam] += e
            fam_n[fam] += 1
        print(f"\nnull-space energy by call site (top {args.top}):")
        for fam, e in sorted(fam_e.items(), key=lambda kv: -kv[1])[:args.top]:
            print(f"  {e:8.2f}  ({fam_n[fam]:4d} rows)  {fam}")
    return 0 if nullity == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
