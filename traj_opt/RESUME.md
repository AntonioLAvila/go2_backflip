# Resume here — 2026-09-06 end of session

Read this first, then `traj_opt/CONVERGENCE.md` for detail. `STATUS.md` is the long-form
history. Nothing is uncommitted.

## State in one line

**Part I is done, shipped and merged: the audit passes 11/11.** Part II (`is_success()`) is
open on branch `kkt-convergence`, with a large structural win that did *not* produce
convergence.

## Part I — DONE (on `main`, pushed)

`traj_opt/reference/` holds the first 11/11 trajectory this project has produced. 675 samples,
1.348 s, worst audit margin 0.98x, reproducible with default flags from `backflip.npy`.

Two changes did it, now defaults:
* `nlp_scaling_max_gradient=1` — IPOPT's default scaling was putting every restart burst into
  RESTORATION on its second iteration, where `inf_pr` *grew*. That was the long-documented
  "peaks early then degrades for 16 bursts".
* `--flight-amom` — asserts the flight angular-momentum invariant.

`tools/check_tape.py` is new: the audit's physics on the 500 Hz tape rather than at knots. It
selected the shipped trajectory out of 13 otherwise-indistinguishable 11/11 candidates that
spanned **16x** on tape torque. The shipped one still misses two tape checks (+0.79 N.m over
the design envelope on 0.44% of samples; +0.51 over the hardware envelope; momentum
peak-to-peak 1.34e-3) — all better than its predecessor, all between-knot artefacts.

## Part II — OPEN (branch `kkt-convergence`, not merged)

### The one number that matters

`is_success()` needs IPOPT's overall NLP error under `tol`. At the shipped point that error
**is** the dual infeasibility — the point is feasible (2.4e-04 scaled) and complementary
(1.3e-05) but **not stationary** (3.5e+01). Everything this project ever tuned was aimed at
`max_violation`, the residual that is already small enough.

### The structural win (real, measured, committed)

`traj_opt/kkt_check.py` is new — rank of the **active** set at a solved point, equalities plus
active inequalities, raw and row-normalised. `nullity_check.py` only ever saw equalities at the
guess and could not see any of this.

    formulation                          active   raw null   scaled null   scaled cond
    original                               180      192         10          1.22e+16
    + symmetry as penalty, loose box        100      121         10          1.22e+16
    + momentum as penalty                    14       35         10          1.22e+16
    + release-knot lambda_x pin               4       27          2          7.54e+15
    + body-clearance de-duplication           2       25       **0**      **1.41e+09**

1.22e16 is past `1/eps` — numerically singular, which fully explains dual infeasibility
flooring regardless of settings. 1.41e9 is comfortably inside double precision.

Four mechanisms, all one principle — **a constraint stating what the formulation already
implies costs rank and buys nothing**:
1. symmetry boxes the solve rides (9.99e-05 against a 1e-4 box, so all 80 active);
2. **the Part I momentum constraint** — every row at null-space energy 1.00, because
   conservation is an exact consequence of the defects;
3. **the friction cone at release knots** — half the whole deficiency: `lambda_z` is pinned to
   0 and eliminated, so the two cone rows collapse to `+lambda_x <= 0` and `-lambda_x <= 0`,
   exact negatives;
4. duplicate position-only rows at pinned/stitched knots.

The shipped trajectory audits 11/11 under every one of these, so nothing load-bearing was cut.

### ...and it did not converge

Best dual infeasibility **3.12** against `tol` 1e-4, over ~25 arms: six objective forms, three
barrier strategies, three scalings, warm and cold starts, two `bound_push` decades, four L-BFGS
settings. Fixing rank and conditioning bought a factor of ~2.

Recorded losses (do not retry): flat objective (causes restoration); `bound_push` > 1e-8 (worse
both ways, 2026-09-03 setting stands); L-BFGS history 50/200/SR1 (identical to default, and
this time tested in the right regime); mesh refinement from a ringing source; `--w-rate` for
tape ringing (rings in `qd`, not `tau`).

**SNOPT re-tested** — justified, because its historical failure here was LICQ-induced and that
is what was fixed. Three arms, all fail, all move *away* from feasibility: cold `info=13` at
viol 1339; warm `info=43` "cannot satisfy the general constraints" at viol 7.04; warm+costed
`info=13` at viol 21.3 from a start of 0.58. Also 5-15x slower per solve.

**Also established:** no cold start reaches feasibility at ANY mesh size (14/26/50 knots,
either formulation, either solver). That is why this project has always needed the
restart-from-checkpoint chain.

## Overnight result — `w26r` finished, and it answers decision 2

The 26-knot restart search ran all 12 bursts and stopped. Nothing is running now.

    best: viol 0.9278, audit 6/11, worst 92.31x
    min inf_du = 1.45 at iteration 324, over 5116 normal iterations

Two readings, and they point opposite ways:

* **Size does matter.** 4934 variables reach dual infeasibility **1.45** where 6566 variables
  floor at **3.12** — a factor of ~2 from a 25% smaller problem, at a comparable distance from
  the feasible manifold (viol 0.93 against 0.58). That is the first evidence in the campaign
  that the stationarity residual is scale-dependent rather than structural.
* **It is nowhere near enough.** `tol` is 1e-4. A 25% size cut bought half an order; closing
  four more would need a problem far smaller than anything that could represent this manoeuvre.
  And the 26-knot trajectory audits **6/11** — it is not a usable trajectory, so this is a
  statement about the solver's arithmetic, not a route to a shippable result.

So decision 2 is answered: **"too big" is real but not the whole story, and shrinking is not a
path.** Treat the small-problem line of attack as closed.

## Decisions for tomorrow

1. **Stop and write up.** The structural result stands on its own and Part II is a
   well-evidenced negative. Merge `kkt-convergence` for `kkt_check.py`, the redundancy
   removals and the ledger; leave the penalties as off-by-default flags.
2. **One more question, if `w26r` reached feasibility:** ask stationarity at 4934 variables
   instead of 6566. That is the only clean test left of "too big" vs "cannot be certified".
3. **Change the objective.** Every remaining explanation points at the iterates not being near
   a stationary point of anything posed. That is a modelling question, not a solver one, and
   it is the only untried direction with real headroom.

My recommendation: (1) plus (2) only if `w26r` delivered — and treat (3) as a separate piece of
work rather than a continuation.

## Commands

    git checkout kkt-convergence            # Part II work; main has the shipped 11/11
    uv run traj_opt/kkt_check.py --sym-box 1e-2 --flight-amom 0   # rank of the active set
    uv run tools/check_tape.py                                     # between-knot checks
    uv run traj_opt/solve_backflip.py --from-checkpoint traj_opt/reference/backflip.npy

The repaired formulation is `--flight-amom 0 --sym-box 1e-2 --sym-penalty 10 --amom-penalty 1e3`.
