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

Rank alone is necessary, not sufficient. Even a full-rank active set (M21: nullity 0, cond
1.4e9) still floors dual infeasibility at ~3 -- LICQ holding does not mean the KKT system has a
*good-conditioned, strictly complementary* solution, only that it has a unique one. The other
classical ingredient for a solver to actually reach that solution is STRICT COMPLEMENTARITY:
every active inequality should have a multiplier bounded away from zero, not just a constraint
value near its bound. A "weakly active" row (both near zero) is invisible to a rank check and
invisible to IPOPT's own complementarity residual (`|lambda_i * g_i|`, small whenever EITHER
factor is small -- which weakly-active rows are, by definition) but is exactly the kind of
degeneracy that leaves a solver's dual iterates undetermined even in a full-rank system.

This computes multipliers directly rather than trusting IPOPT's own reported ones from
whatever restart chain produced the checkpoint: it takes the same active-set Jacobian M this
file already builds, adds the objective gradient g at the same point (from bp.add_cost() plus
any penalty costs the constructor flags turned on), and solves the linear least-squares system
`M^T @ lambda ~= -g` for the multiplier vector that makes the Lagrangian's gradient as close to
zero as possible. That gives, self-contained from just (x, constraints, objective):
  * a stationarity residual directly comparable to IPOPT's own dual infeasibility;
  * per-row multipliers, so an inequality's SIGN can be checked against what stationarity of a
    minimization actually requires (>=0 active-at-upper, <=0 active-at-lower) -- a wrong sign
    means the point is not even a KKT candidate along that direction, a strictly stronger
    finding than "weakly active";
  * a magnitude, reported as the row's actual contribution `|lambda_i| * ||row_i||` to
    cancelling the gradient (comparable across families of wildly different natural units,
    same reasoning as the row-normalised Jacobian above) rather than the raw multiplier, which
    is not.

When nullity > 0 the multiplier decomposition is not unique (any left-null-space combination of
rows can be added to a solution without changing `M^T @ lambda`); `np.linalg.lstsq` returns the
minimum-norm solution, and the per-row breakdown should be read as suggestive, not exact, until
nullity is actually fixed (--sym-penalty/--sym-box/--amom-penalty).

    uv run traj_opt/kkt_check.py [checkpoint.npy] [--active-tol 1e-6]
    uv run traj_opt/kkt_check.py --sym-box 1e-2 --sym-penalty 10 --flight-amom 0 --amom-penalty 1e3
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
    ap.add_argument("--amom-penalty", type=float, default=0.0)
    ap.add_argument("--amom-mode", choices=["chain", "anchor"], default="chain")
    ap.add_argument("--sym-penalty", type=float, default=0.0)
    ap.add_argument("--sym-box", type=float, default=None)
    ap.add_argument("--top", type=int, default=20)
    ap.add_argument("--no-cost", action="store_true",
                    help="skip bp.add_cost() -- the gradient in the complementarity check is "
                         "then whatever penalty costs the flags above added, or exactly zero")
    ap.add_argument("--w-torque", type=float, default=1.0)
    ap.add_argument("--w-rate", type=float, default=0.1)
    ap.add_argument("--w-time", type=float, default=1.0)
    ap.add_argument("--w-tuck", type=float, default=0.5)
    ap.add_argument("--w-vrate", type=float, default=0.0)
    ap.add_argument("--cost-scale", type=float, default=1.0,
                    help="matches solve_backflip.py's --cost-scale; the weights above are its "
                         "add_cost() defaults, so a checkpoint produced with default flags gets "
                         "a faithful objective gradient with no flags needed here")
    args = ap.parse_args()

    MathematicalProgram.AddBoundingBoxConstraint = _wrap(
        MathematicalProgram.AddBoundingBoxConstraint)
    MathematicalProgram.AddLinearConstraint = _wrap(MathematicalProgram.AddLinearConstraint)

    from program import AMOM_BOX, BackflipProgram

    amom = AMOM_BOX if args.flight_amom is None else (args.flight_amom or None)
    bp = BackflipProgram(amom=amom, amom_mode=args.amom_mode, amom_penalty=args.amom_penalty,
                         sym_penalty=args.sym_penalty, sym_box=args.sym_box)
    if not args.no_cost:
        bp.add_cost(w_torque=args.w_torque, w_rate=args.w_rate, w_time=args.w_time,
                    w_tuck=args.w_tuck, w_vrate=args.w_vrate, scale=args.cost_scale)
    x = np.load(args.checkpoint)
    if x.shape != (bp.prog.num_vars(),):
        raise SystemExit(f"{args.checkpoint}: {x.shape[0]} variables, program has "
                         f"{bp.prog.num_vars()}")
    print(f"{args.checkpoint}: {bp.prog.num_vars()} vars, "
          f"{len(bp.prog.GetAllConstraints())} bindings, "
          f"{len(bp.prog.GetAllCosts())} cost bindings")

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

    rows, labels, kinds, sides = [], [], [], []
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
            # Which bound this inequality rides -- needed for the KKT sign check below.
            # Stationarity is grad f + sum lambda_i grad c_i = 0; writing lo<=c(x)<=hi as two
            # standard-form g(x)<=0 constraints gives active-at-hi -> lambda>=0 and
            # active-at-lo -> lambda<=0 in THIS convention (grad c_i, not grad g_i). Meaningless
            # for equalities, which carry no sign requirement.
            sides.append("eq" if eq[r] else ("hi" if abs(y[r] - hi[r]) <= abs(y[r] - lo[r])
                                             else "lo"))
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

    # ...and again with every row normalised to unit inf-norm. IPOPT does not see the raw
    # Jacobian: nlp_scaling_max_gradient rescales each constraint so its largest gradient
    # element is at most that value, which is exactly a row scaling. Quoting the unscaled
    # condition number overstates what the linear solver faces, and on this problem the two
    # differ by orders of magnitude -- the raw sigma_max is set by whichever defect row has
    # the biggest accelerations, not by any coupling.
    R = M / np.maximum(np.abs(M).max(axis=1, keepdims=True), 1e-300)
    sr = np.linalg.svd(R, compute_uv=False)
    tr = sr.max() * max(R.shape) * np.finfo(float).eps * 100
    nr = int((sr < tr).sum())
    print(f"row-normalised (what IPOPT's scaling gives it): "
          f"sigma max {sr.max():.3e} min {sr.min():.3e}; "
          f"cond {sr.max() / sr[sr > 0].min():.2e}; nullity {nr}")
    # The tail of the spectrum decides whether the ill-conditioning is CONFINED to the null
    # space. If sigma just above the nullity is healthy, the deficiency is a finite set of
    # redundant rows and removing them leaves a well-conditioned problem; if the spectrum
    # decays smoothly into the noise floor there is no such set, and no amount of row
    # surgery makes the KKT system solvable in double precision.
    tail = np.sort(sr)[:nr + 12]
    print("  smallest singular values: " + " ".join(f"{v:.2e}" for v in tail))
    if nr < len(sr):
        eff = sr.max() / np.sort(sr)[nr]
        print(f"  effective cond excluding the {nr} null directions: {eff:.2e}")

    if nr:
        # Family breakdown for the ROW-NORMALISED null space, which is the one that matters:
        # those are the directions IPOPT's own scaling cannot see either, and the spectrum
        # shows them separated from the rest by seven orders. Removing exactly these rows is
        # what would leave a KKT system solvable in double precision.
        Ur, sru, _ = np.linalg.svd(R, full_matrices=False)
        er = (Ur[:, np.argsort(sru)[:nr]] ** 2).sum(axis=1)
        fe, fn = defaultdict(float), defaultdict(int)
        for lbl, kind, e in zip(labels, kinds, er):
            fam = f"{lbl.split('#')[0]} ({kind})"
            fe[fam] += e
            fn[fam] += 1
        print(f"\nROW-NORMALISED null space ({nr} directions), energy by call site:")
        for fam, e in sorted(fe.items(), key=lambda kv: -kv[1])[:args.top]:
            if e < 1e-3:
                break
            print(f"  {e:8.3f}  ({fn[fam]:4d} rows)  {fam}")
        print("  individual rows above 0.10:")
        for i in np.argsort(-er)[:24]:
            if er[i] < 0.10:
                break
            print(f"    {er[i]:6.3f}  {labels[i]}  [{kinds[i]}]")

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

    # --- strict complementarity / stationarity, via a least-squares multiplier estimate -----
    # grad f(x) via the same central-FD pattern as the constraint Jacobian above, over every
    # cost binding bp.add_cost() (and any --sym-penalty/--amom-penalty terms) added.
    g_full = np.zeros(bp.prog.num_vars())
    for c in bp.prog.GetAllCosts():
        ev = c.evaluator()
        idx = bp.prog.FindDecisionVariableIndices(c.variables())
        xloc = x[idx].copy()
        for j in range(xloc.size):
            h = eps * max(1.0, abs(xloc[j]))
            xp, xm = xloc.copy(), xloc.copy()
            xp[j] += h
            xm[j] -= h
            g_full[idx[j]] += (float(np.asarray(ev.Eval(xp)).ravel()[0])
                               - float(np.asarray(ev.Eval(xm)).ravel()[0])) / (2 * h)
    g = g_full[~fixed]

    # Solve for the multiplier vector making the Lagrangian's gradient as close to zero as
    # possible -- self-contained, no IPOPT re-solve needed. Against the ROW-NORMALISED
    # Jacobian R, not raw M: raw M's own SVD nullity (178 in the default formulation) mixes
    # genuine rank deficiency with rows that are merely badly scaled relative to each other
    # (M2 -- defect rows span ~4 orders of magnitude by construction), which is exactly why
    # the file already keeps R as the "what IPOPT's scaling gives it" lens for rank. lam here
    # is thus the IPOPT-scaled multiplier; since every row of R has max abs entry 1 by
    # construction, its magnitude alone (no further row-norm multiply needed) is already the
    # row's contribution in comparable, cross-family units -- call it "push". The raw-M
    # stationarity residual is IDENTICAL to R's (M^T @ (lam/row_norm) == R^T @ lam algebraically),
    # so solving against R costs nothing and is the numerically trustworthy version when raw
    # nullity is nonzero but row-normalised nullity (nr) is not -- exactly the case the K9/K10
    # full-rank formulation produces. g itself is left in raw units (IPOPT also independently
    # scales the objective gradient; not reproduced here), so read the residual as directional,
    # not a bit-exact match to IPOPT's own printed dual infeasibility.
    lam, _, ls_rank, _ = np.linalg.lstsq(R.T, -g, rcond=None)
    resid = R.T @ lam + g
    print(f"\nSTATIONARITY (least-squares multiplier estimate over {R.shape[0]} rows, "
          f"lstsq rank {ls_rank} of {R.shape[1]}, row-normalised):")
    print(f"  ||grad f + sum lambda_i grad c_i||_2 = {np.linalg.norm(resid):.3e}   "
          f"(inf-norm {np.abs(resid).max():.3e}) -- directionally comparable to IPOPT's own "
          f"dual infeasibility, not bit-exact (objective scaling not reproduced)")
    if nr:
        print(f"  NOTE: row-normalised nullity {nr} > 0, so this decomposition is not unique -- "
              f"fix the rank first (--sym-penalty/--sym-box/--amom-penalty) before trusting the "
              f"per-row picture below")

    # Per-row "push" = |lambda_i| directly (R's rows are already unit-normalised). Only
    # inequality rows have a sign requirement; equalities are reported for magnitude only.
    push = np.abs(lam)
    ineq = np.array([s != "eq" for s in sides])
    wrong_sign = ineq & (((np.array(sides) == "hi") & (lam < 0))
                         | ((np.array(sides) == "lo") & (lam > 0)))

    if ineq.any():
        p_ineq = push[ineq]
        scale = max(np.median(p_ineq[p_ineq > 0]) if np.any(p_ineq > 0) else 1.0, 1e-300)
        weak = ineq & (push < 1e-3 * scale)
        print(f"\nCOMPLEMENTARITY over {int(ineq.sum())} active inequality rows "
              f"(median push {scale:.3e}):")
        print(f"  weakly active (push < 1e-3 x median): {int(weak.sum())}")
        print(f"  WRONG SIGN (not a KKT candidate along this direction): "
              f"{int(wrong_sign.sum())}")

        fam_push, fam_weak, fam_wrong, fam_n2 = (defaultdict(float), defaultdict(int),
                                                  defaultdict(int), defaultdict(int))
        for lbl, side, p, w, ws, take_row in zip(labels, sides, push, weak, wrong_sign, ineq):
            if not take_row:
                continue
            fam = lbl.split('#')[0]
            fam_push[fam] += p
            fam_n2[fam] += 1
            fam_weak[fam] += int(w)
            fam_wrong[fam] += int(ws)
        print("  by call site (median push, weakly-active count, wrong-sign count):")
        for fam in sorted(fam_n2, key=lambda f: fam_push[f] / fam_n2[f]):
            print(f"    {fam_push[fam] / fam_n2[fam]:10.3e}  ({fam_n2[fam]:4d} rows)  "
                  f"weak {fam_weak[fam]:4d}  wrong-sign {fam_wrong[fam]:4d}  {fam}")

        if wrong_sign.any():
            print("  worst wrong-sign rows:")
            idxs = np.flatnonzero(wrong_sign)
            for i in idxs[np.argsort(-push[idxs])[:15]]:
                print(f"    push {push[i]:.3e}  lambda {lam[i]:+.3e}  side {sides[i]}  "
                      f"{labels[i]}")

    return 0 if nullity == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
