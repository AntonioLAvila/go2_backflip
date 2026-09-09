# Resume here — 2026-09-08 end of session

Read this first, then `traj_opt/CONVERGENCE.md` for detail (M24/M25 and K14/K15 are this
session's entries). `STATUS.md` is the long-form history. Nothing is uncommitted as documentation
— code changes below ARE uncommitted; see "Uncommitted state" at the bottom.

## State in one line

**Part I is unchanged: shipped, 11/11, on `main`.** Part II gained a real tool (strict
complementarity, and it's ruled out as the blocker) and a real new lever (`--w-vrate`) whose
first test was inconclusive because of how it was tested, not because it failed. Both are on
`kkt-convergence`, uncommitted.

## What happened this session

1. **Fixed a documentation bug**, unrelated to convergence: `CLAUDE.md` said the impulsive
   touchdown fires at "load→launch" (a liftoff). It's `schedule.IMPACT = 3`, flight→absorb —
   the actual landing. Fixed.
2. **Ruled out two more structural hypotheses** (see the previous session's exit report,
   folded into this one):
   - A larger, non-Hermite-spline program (`DirectTranscription`, hand-rolled multiple
     shooting) — not promising: `DirectTranscription` is 1st-order (forward Euler) against
     Hermite-Simpson's 3rd, so matching current accuracy needs a MUCH bigger program, which
     cuts against the one working lever Part II found (`w26r`: smaller helped, and "not nearly
     enough" even so). No other Drake trajopt class fits (both `KinematicTrajectoryOptimization`
     and `GcsTrajectoryOptimization` are force-free path planners).
   - The "DirectCollocation interpolates quaternions in flat R^4, not on SO(3)" concern —
     measured directly (`traj_opt/check_quat_interp.py`, untracked, not yet committed) against
     the shipped trajectory's actual reconstructed cubic-Hermite spline: max `|q0^2+q2^2-1|`
     between knots is **4.46e-5**, and the flat interpolant's implied pitch angle differs from a
     proper scalar-angle reconstruction by at most **2.4e-6 rad**. Sagittal symmetry has already
     collapsed the rotation to a 1-DOF circle, and at this mesh's step sizes the flat cubic
     tracks it fine. **Ruled out as a driver of anything.**
3. **`kkt_check.py` gained a strict-complementarity check.** It only ever tested rank. Extended
   to solve `lstsq(R^T, -g)` for the KKT multipliers directly (R = row-normalised active-set
   Jacobian, g = objective gradient), giving a stationarity residual comparable to IPOPT's own
   `inf_du` plus a per-row sign check.
   **Finding (K14): at the full-rank formulation, complementarity holds — cleanly.** The fit is
   exact and unique (`lstsq rank = 5756 of 5756`), both active inequalities have healthy,
   correctly-signed multipliers, zero weak, zero wrong-sign. The 0.397 residual left is entirely
   on the equality side. This independently confirms M21 by a different route and rules out one
   more candidate mechanism: it is not a degenerate-KKT-system problem any more, full stop — it
   is a well-posed linear system whose exact answer is "this point is not a critical point of
   this objective."
4. **Added `w_vrate` to `program.py:add_cost`** (default 0, opt-in via `--w-vrate`): a
   velocity-smoothness penalty, `w_rate`'s exact structure but on `v` instead of `u`. Motivated
   by two already-established findings, not a guess — see CONVERGENCE.md M25 for the argument.
   This is the concrete instance of RESUME's own standing recommendation, "change the objective
   so the good point is actually its minimizer."
5. **Ran three bounded verification chains** (5 restarts x 300 iters each, seeded bit-exact on
   the shipped 11/11 point) to test it. **Result: inconclusive, not negative** — see below for
   why the distinction matters and what to run instead.

## The verification result, and why it doesn't settle anything

None of `vopt_baseline`, `vopt_vrate`, `vopt_repaired` beat the seed (viol 0.5797) in 5 bursts —
full numbers in CONVERGENCE.md M25. Two things make this weaker evidence than it looks:

* **The search was a CHAIN, and this project has independently confirmed chains don't answer
  "is X a good idea" questions** — `restart_loop` always advances from each burst's RAW result,
  by design, so one bad burst poisons every burst after it (that's the exploration mechanism,
  and it's the right choice for the production search). For TESTING a hypothesis, it's the wrong
  instrument: `vopt_repaired`'s burst 2 (viol 53) wasn't a verdict on the formulation, it was a
  verdict on burst 1's endpoint (viol 3.5) as a starting point. A fair test of "does `--w-vrate`
  help" needs independent trials FROM THE SEED, the same `screen.sh` methodology the campaign
  already used to screen 16 solver options cheaply before committing to full searches (see
  CONVERGENCE.md "The screen").
* **The weight (0.1) was never screened.** It's `w_rate`'s own default, chosen by analogy, not
  measured. `vrate`'s burst 0 (1.04) being closer to the seed than baseline's (2.32) or
  repaired's (10.7) is a real, if weak, signal that the term pulls the right way — a proper
  sweep might find a weight where burst 0 actually wins.
* **`vopt_repaired`'s blowup (dual inf 1.4e13, worst this campaign has recorded) is very likely
  a scaling clash**, not a structural failure: `sym_penalty=10` / `amom_penalty=1e3` were
  carried over unchanged from the K6/K7 experiments, which never had `w_vrate` in the objective
  alongside them. Three penalty weights picked independently, then combined, is exactly the kind
  of thing that needs rechecking together.

## Next session: what to actually run

**Do not repeat 5x300-iteration chains as the way to test a cost-term weight.** Use the cheap
screen first:

```
# 1. Screen w_vrate ALONE (hard-box formulation, no penalty terms) at several weights, each an
#    INDEPENDENT 80-iteration burst from the seed -- not chained. ~2 min each, 5 values ~10 min
#    total, replaces one of today's ~55-minute chains with something 5x more informative.
for w in 0.01 0.03 0.1 0.3 1.0; do
  uv run traj_opt/solve_backflip.py --start-checkpoint traj_opt/reference/backflip.npy \
      --no-prepass --restarts 1 --burst-iters 80 --w-vrate $w \
      --out traj_opt/out/screen_vrate_$w.npz
done

# 2. Cheaper still, no solve at all: does adding the term even reduce the stationarity residual
#    AT THE SEED (unmoved)? Not proof a solve improves, but a term that fails this is a bad sign
#    for ~0 cost.
for w in 0.01 0.03 0.1 0.3 1.0; do
  uv run traj_opt/kkt_check.py --w-vrate $w | grep "STATIONARITY" -A1
done

# 3. Only once a weight wins the screen: retune amom_penalty/sym_penalty jointly with it (don't
#    reuse 10 / 1e3 unchanged) before re-attempting the full-rank + w_vrate combination.
```

If a proper sweep still can't find a `w_vrate` (or combination) whose short screen beats the
seed, that upgrades K15 from `INCONCLUSIVE` to a real negative result, and it would be worth
revisiting RESUME's original decision 1 from 2026-09-06: stop chasing `is_success()`-style
convergence on this formulation, the audit-passing artifact is what downstream RL actually
consumes, and the two structural wins (K9/K10 rank fix, K14 complementarity) are worth writing
up and merging on their own regardless of whether Part II's ultimate target is ever reached.

## Uncommitted state

`git status` on `kkt-convergence`: `CLAUDE.md`, `traj_opt/kkt_check.py`, `traj_opt/program.py`,
`traj_opt/solve_backflip.py` modified; `traj_opt/check_quat_interp.py` untracked (new, working,
not yet decided whether it should become a tracked diagnostic alongside `kkt_check.py`). Nothing
committed this session — ask before committing, per standing instructions. The three verification
runs also wrote `traj_opt/out/verify_{baseline,vrate,repaired}.npz` and
`traj_opt/out/checkpoints_verify_{baseline,vrate,repaired}/` (gitignored scratch, safe to
delete).

---

# Prior session (2026-09-06) — kept for context

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

### The structural win (real, measured, committed on `kkt-convergence`)

`traj_opt/kkt_check.py` — rank of the **active** set at a solved point, equalities plus active
inequalities, raw and row-normalised (and, as of 2026-09-08, the KKT multipliers themselves).
`nullity_check.py` only ever saw equalities at the guess and could not see any of this.

    formulation                          active   raw null   scaled null   scaled cond
    original                               180      192         10          1.22e+16
    + symmetry as penalty, loose box        100      121         10          1.22e+16
    + momentum as penalty                    14       35         10          1.22e+16
    + release-knot lambda_x pin               4       27          2          7.54e+15
    + body-clearance de-duplication           2       25       **0**      **1.41e+09**

1.22e16 is past `1/eps` — numerically singular, which fully explains dual infeasibility
flooring regardless of settings. 1.41e9 is comfortably inside double precision. **As of
2026-09-08, K14 additionally confirms strict complementarity holds there too** — the remaining
non-stationarity is not a KKT-system pathology of any kind, rank or complementarity; it's the
objective's minimizer being a different, worse point (M6/M9), which is what `--w-vrate` (K15) is
the first real attempt to fix.

### ...and it did not converge

Best dual infeasibility **3.12** against `tol` 1e-4, over ~25 arms: six objective forms, three
barrier strategies, three scalings, warm and cold starts, two `bound_push` decades, four L-BFGS
settings. Fixing rank and conditioning bought a factor of ~2.

Recorded losses (do not retry): flat objective (causes restoration); `bound_push` > 1e-8 (worse
both ways, 2026-09-03 setting stands); L-BFGS history 50/200/SR1 (identical to default, and
this time tested in the right regime); mesh refinement from a ringing source; `--w-rate` for
tape ringing (rings in `qd`, not `tau`); weakly-active inequalities as the complementarity
mechanism (K14, 2026-09-08).

**SNOPT re-tested** — justified, because its historical failure here was LICQ-induced and that
is what was fixed. Three arms, all fail, all move *away* from feasibility: cold `info=13` at
viol 1339; warm `info=43` "cannot satisfy the general constraints" at viol 7.04; warm+costed
`info=13` at viol 21.3 from a start of 0.58. Also 5-15x slower per solve.

**Also established:** no cold start reaches feasibility at ANY mesh size (14/26/50 knots,
either formulation, either solver). That is why this project has always needed the
restart-from-checkpoint chain.

### `w26r` — size matters, but not nearly enough

The 26-knot restart search (12 bursts): best viol 0.9278, audit 6/11, min dual infeasibility
**1.45** over 5116 normal iterations, against 3.12 at 50 knots from a comparable distance off
the feasible manifold. Smaller helps the stationarity residual by ~2x; `tol` is 1e-4, so this is
not a path on its own, and the 26-knot trajectory isn't usable anyway (6/11). Treat the
small-problem line of attack as closed.

## Commands

    git checkout kkt-convergence            # Part II work; main has the shipped 11/11
    uv run traj_opt/kkt_check.py --sym-box 1e-2 --flight-amom 0 --amom-penalty 1e3 --sym-penalty 10
        # rank AND (as of 2026-09-08) strict-complementarity of the active set
    uv run tools/check_tape.py                                     # between-knot checks
    uv run traj_opt/check_quat_interp.py     # SO(3)/flat-cubic interpolation check (untracked)
    uv run traj_opt/solve_backflip.py --from-checkpoint traj_opt/reference/backflip.npy

The repaired formulation is `--flight-amom 0 --sym-box 1e-2 --sym-penalty 10 --amom-penalty 1e3`
— add `--w-vrate W` (screen W first, see "Next session" above) to test the objective-redesign
direction.
