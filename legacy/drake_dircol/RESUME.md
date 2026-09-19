# Resume here — 2026-09-09 end of session

Read this first, then `traj_opt/CONVERGENCE.md` for detail (M26/M27/K15-resolved/K16/K17 are
this session's entries, on top of 2026-09-08's M24/M25/K14). `STATUS.md` is the long-form
history. Code changes from 2026-09-08 are committed (`1c710c7`, branch `kkt-convergence`);
everything from today (2026-09-09, doc-only) is uncommitted — see "Uncommitted state".

## State in one line

**K15 is a real, narrow win, and the shape of the win is itself the finding.** `--w-vrate 0.1`
has a genuine ~10-iteration-wide basin (`--burst-iters` roughly 76-83) where independent short
bursts from the seed land at worst-margin 0.88-0.89x (vs the shipped reference's 0.98x) and
substantially better tape torque-envelope compliance — confirmed at TWO points in the basin
(80 and 83), not one lucky checkpoint. But a 300-iteration chain from the same seed and weight
walks straight past this basin and degrades exactly like every chain before it (M27) — so the
basin exists but is not yet reliably reachable by a normal search. That's next session's job.

## What happened this session (2026-09-09), in the order it happened

1. **Ran the previous session's prescribed plan**: a zero-cost `kkt_check.py --w-vrate W`
   pre-filter (moved the wrong way with `W`, turned out not predictive — demoted, see
   CONVERGENCE.md M26), then five INDEPENDENT 80-iteration bursts from the seed across
   `w_vrate in {0.01, 0.03, 0.1, 0.3, 1.0}`. All five held 11/11 in one burst — already sharper
   than any of the previous session's three 5-burst CHAINS, none of which held 11/11 past their
   own first burst. `w_vrate=0.1` won: worst margin 0.88x vs the seed's 0.98x, and a real tape
   improvement (hardware-envelope overshoot 0.51 N.m -> 0.01 N.m, design-envelope 0.79 -> 0.27
   N.m), at the cost of a mild momentum regression (1.34e-3 -> 1.50e-3 peak-to-peak) — exactly
   the mechanism M11 predicted (`w_vrate` targets `qd` ringing, not momentum).
2. **Launched `search_vrate01`** (10 restarts x 300 iterations) to check reproducibility with
   more search. **It came back WORSE, not better**: burst 0 alone (same seed, same weight, just
   300 iterations instead of 80) landed at 0.99x, barely passing — and since IPOPT is
   deterministic from a fixed start, its first 80 iterations are bit-identical to the winning
   screen run. **IPOPT passes through the same good point around iteration 80 and then walks
   away from it over the next 220 iterations, within that single burst.** The 9 chained bursts
   after that degraded further, same pattern as every prior chain. This showed the "peaks early,
   degrades" pattern this project has documented since 2026-09-04 is not only a between-burst
   phenomenon — it happens WITHIN one continuous run too (CONVERGENCE.md M27, K17 `LOSS`).
3. **Ran a burst-length sweep** (`--burst-iters` in `{50,65,80,95,110,130,160}`) to map out
   whether 80 was a fluke. At that resolution it LOOKED like an isolated spike (65 and 95 were
   both ordinary) — worrying, since a real basin should have width. A finer sweep
   (`{70,73,76,80,83,86,90}`) resolved it properly: **76, 80, and 83 all cluster at margin
   0.88-0.89x** — a real ~10-iteration basin, not noise; the coarse sweep's neighbours simply
   fell just outside its edges. Checked the tape at a second basin point (83, not just 80): same
   pattern, smaller magnitude (design overshoot 0.79 -> 0.41 N.m, hardware 0.51 -> 0.12 N.m,
   momentum 1.34e-3 -> 1.46e-3). Confirms the improvement is a property of the basin, not one
   lucky checkpoint.

## What happened before this (2026-09-08), condensed

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

(2026-09-08's "next session" plan above is what today's session executed — kept for the
reasoning, superseded as a to-do list by what follows.)

## The open question: the basin is real but not reliably reachable

Established this session: at `w_vrate=0.1`, `--burst-iters` in roughly **76-83** lands in a
genuine, reproducible-across-two-tested-points basin (worst margin 0.88-0.89x, better tape
torque compliance than the shipped reference). Also established: a normal 300-iteration burst
from the identical start walks straight past that basin (by construction — its first 80
iterations ARE the winning run, then it keeps going and gets worse) and a 10-restart chain from
there degrades further, same as every chain before it. **So the basin exists, but "just run
`--burst-iters 80`" is not yet a search recipe** — it depends on already knowing the right
number, which was found by sweeping, not derived.

## Next session: what to actually run

1. **Test whether the basin survives from a DIFFERENT seed.** Everything so far is one seed
   (the shipped reference) — Part I's reproducibility bar was "six independent chains reach
   11/11"; this is one point in seed-space with two nearby-iteration-count confirmations, which
   is real evidence but a much lower bar. Cheapest test: take one of the OTHER audit-11/11
   candidates from Part I's `t3_b0`-selection table (STATUS.md/CONVERGENCE.md M13 lists `r1`,
   `t3_b2`, `n9`, `q2`, `u10`) as a second seed and repeat the same independent-burst,
   `w_vrate=0.1`, `--burst-iters` ~76-86 screen against it.
2. **Turn the basin into a reachable search recipe, not a located point.** Two options, in order
   of how much they cost:
   - Cheapest: run `restart_loop` with `--burst-iters` actually INSIDE the basin (e.g. 80) for
     several restarts, instead of 300 — since chaining forward from a basin point might behave
     differently than chaining from the seed itself (never tested; every restart so far started
     either at 80 iterations exactly once, or at 300). If the SECOND burst (chained from the
     80-iteration point) also lands in a similar basin, that is a real, usable two-burst recipe.
   - More informative but more code: add a callback that tracks the audit/tape-best INTERMEDIATE
     iterate during a burst (Drake's `DirectCollocation`/`MultipleShooting` supports
     `AddCompleteTrajectoryCallback`, seen in `pydrake.planning`'s own docstring), so a good
     point passed through mid-burst is captured automatically instead of requiring a correct
     guess of `--burst-iters` in advance. This is the structural fix for what M27 found; flagged
     here rather than built, since it changes `solve_backflip.py`'s solve loop and deserves
     sign-off before implementing.
3. **Only once (1) or (2) gives something reproducible enough to trust**, ship it:
   ```
   uv run tools/ship.py <checkpoint> \
       --note "w_vrate=0.1, burst-iters ~80: worst margin 0.88x vs 0.98x, tape hardware-envelope 0.01 vs 0.51 N.m"
   ```
4. Lower priority, still worth doing: a wider `w_vrate` sweep around 0.1 (0.05, 0.07, 0.15, 0.2),
   and only after the above, retuning `amom_penalty`/`sym_penalty` jointly with `w_vrate` before
   re-attempting the full-rank formulation (`vopt_repaired`'s 1.4e13 blowup from 2026-09-08 was
   very likely those weights clashing, not evidence against either piece individually).

If (1) fails — the basin turns out to be specific to this one seed — that is still worth
recording as a real, if narrower, result: `--w-vrate` demonstrably CAN produce a better basin,
even if this project doesn't yet have a reliable way to land in one on demand. That would be the
moment to revisit the original 2026-09-06 decision: stop chasing `is_success()`-style
convergence, the audit-passing artifact is what downstream RL consumes, and the structural wins
(K9/K10, K14) are worth writing up and merging regardless of Part II's ultimate target.

## Uncommitted state

`git status` on `kkt-convergence` should be clean except for today's (2026-09-09) work, since
2026-09-08's changes are committed (`1c710c7`). Today added no code changes (the `w_vrate`
machinery already existed) — only `traj_opt/CONVERGENCE.md` (M26/M27/K15-K17) and this file.
Scratch outputs from today's screens (`traj_opt/out/{screen_vrate_*,search_vrate01,
screen_burstlen_*}.npz` and their `checkpoints_*` directories) are gitignored — safe to delete,
but worth keeping until the next session's seed-robustness test is done, since
`checkpoints_screen_burstlen_80/best.npy` and `..._83/best.npy` are the two basin points that
matter.

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
