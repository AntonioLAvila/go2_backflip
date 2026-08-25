"""FD-Jacobian + SVD nullity check for the active-equality-constraint set at the guess.

The diagnostic that found bugs 1, 2, and 3 (see STATUS.md): stack the FD Jacobian of every
EQUALITY-type binding (lb == ub, so exact position pins and collocation defects -- not the
TIGHT boxes) at the current initial guess, SVD, and look at nullity = rows - rank. A LICQ
violation shows up as near-zero singular values; the left singular vectors for those point back
at exactly which rows are redundant.

AddBoundingBoxConstraint/AddLinearConstraint carry no description in program.py, and
DirectCollocation names each phase's own state variables "x(i)" starting from 0 independently
per phase, so raw variable names collide across phases and are useless for grouping. Monkeypatch
both methods to auto-tag each binding with its Python call site (function:line) before building
the program, purely for readable grouping -- this changes no actual constraint.

    uv run traj_opt/nullity_check.py
"""

from __future__ import annotations

import inspect
import sys
from collections import defaultdict

import numpy as np
from pydrake.solvers import MathematicalProgram

_counters: dict[str, int] = defaultdict(int)


def _tag(binding, caller):
    ev = binding.evaluator()
    if not ev.get_description():
        _counters[caller] += 1
        ev.set_description(f"{caller}#{_counters[caller]}")


def _wrap(orig):
    def wrapped(self, *a, **kw):
        f = inspect.currentframe().f_back
        caller = f"{f.f_code.co_name}:{f.f_lineno}"
        result = orig(self, *a, **kw)
        _tag(result, caller)
        return result
    return wrapped


def main() -> int:
    MathematicalProgram.AddBoundingBoxConstraint = _wrap(MathematicalProgram.AddBoundingBoxConstraint)
    MathematicalProgram.AddLinearConstraint = _wrap(MathematicalProgram.AddLinearConstraint)

    from program import BackflipProgram
    from guess import Guess
    from solve_backflip import set_guess

    bp = BackflipProgram()
    print(f"program: {bp.prog.num_vars()} vars, {len(bp.prog.GetAllConstraints())} constraints")

    g = Guess(bp.plant)
    set_guess(bp, g.build(), g.footholds())
    x0 = bp.prog.initial_guess()
    if np.any(np.isnan(x0)):
        print(f"WARNING: {np.isnan(x0).sum()} nan entries in initial guess -- filling with 0")
        x0 = np.nan_to_num(x0)

    rows, labels = [], []
    n_eq_bindings = 0
    eps = 1e-6

    for b in bp.prog.GetAllConstraints():
        ev = b.evaluator()
        lo, hi = ev.lower_bound(), ev.upper_bound()
        eq_mask = np.abs(hi - lo) < 1e-12
        if not np.any(eq_mask):
            continue
        n_eq_bindings += 1
        idx = bp.prog.FindDecisionVariableIndices(b.variables())
        xloc = x0[idx].copy()
        m = ev.Eval(xloc).size
        J = np.zeros((m, xloc.size))
        for j in range(xloc.size):
            h = eps * max(1.0, abs(xloc[j]))
            xp, xm = xloc.copy(), xloc.copy()
            xp[j] += h
            xm[j] -= h
            J[:, j] = (ev.Eval(xp) - ev.Eval(xm)) / (2 * h)
        desc = ev.get_description()
        for r in range(m):
            if not eq_mask[r]:
                continue
            full_row = np.zeros(bp.prog.num_vars())
            full_row[idx] = J[r]
            rows.append(full_row)
            labels.append(f"{desc}[{r}]")

    M = np.array(rows)
    print(f"equality bindings: {n_eq_bindings}, rows: {M.shape[0]}, cols touched: {M.shape[1]}")

    s = np.linalg.svd(M, compute_uv=False)
    tol = s.max() * max(M.shape) * np.finfo(float).eps * 100
    nullity = int((s < tol).sum())
    print(f"rank = {np.linalg.matrix_rank(M, tol=tol)}, singular values: "
          f"max={s.max():.3e} min={s.min():.3e}")
    print(f"nullity (tol={tol:.3e}) = {nullity}  out of {M.shape[0]} rows")

    if nullity > 0:
        U, s2, _ = np.linalg.svd(M, full_matrices=False)
        null_cols = np.argsort(s2)[:nullity]
        null_energy = (U[:, null_cols] ** 2).sum(axis=1)

        fam_energy, fam_count = defaultdict(float), defaultdict(int)
        for lbl, e in zip(labels, null_energy):
            fam = lbl.split("#")[0]
            fam_energy[fam] += e
            fam_count[fam] += 1

        print(f"\nnull-space energy by call site (top 20 of {len(fam_energy)}):")
        for fam, e in sorted(fam_energy.items(), key=lambda kv: -kv[1])[:20]:
            print(f"  {e:8.2f}  ({fam_count[fam]:4d} rows)  {fam}")

        print("\nsingle highest-null-energy individual rows (top 20):")
        for i in np.argsort(-null_energy)[:20]:
            print(f"  energy={null_energy[i]:.3f}  {labels[i]}")

    return 0 if nullity == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
