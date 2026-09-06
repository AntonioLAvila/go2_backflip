# Convergence campaign — getting to 11/11 or `is_success()`

Working ledger for the push started **2026-09-06**. `STATUS.md` stays the long-form
session log; this file is the short one: **what was tried, what was predicted, what was
measured, and what is therefore ruled out.** Its whole job is to stop me re-running a
dead end. Every experiment gets a row before it launches and a verdict after.

## The target

Stop when **either**:
- `result.is_success()` — IPOPT converges to tolerance, or
- **11/11 audit checks pass.**

Starting point (`traj_opt/reference/`, commit `3a96825`): **9/11**, worst 1.97x,
`max_violation` 0.1375.

| failing check | measured | bound | over |
|---|---|---|---|
| flight angular momentum conserved | 1.97e-3 | 1e-3 | 1.97x |
| collocation matches a tight integrator | 6.30e-3 | 5e-3 | 1.26x |

Neither is far. Both are proxies for the same residual infeasibility.

## Rules for this campaign

1. **Never loosen an audit threshold to make a check pass.** Adding a *true physical*
   constraint is fine; moving a goalpost is not. (Standing example of the wrong move:
   the friction-cone check's absolute 1e-6 N bound on ~200 N forces. Left alone.)
2. **Rank by the audit**, never by `max_violation` — they anti-correlate here
   (STATUS 2026-09-03/04).
3. **One variable per experiment**, so a verdict means something. IPOPT is
   deterministic from a fixed start + options, so two runs differing in nothing are
   the same run.
4. Record negative results with the same care as positive ones.

---

## Baseline measurements (2026-09-06)

### M1 — the entire residual violation is collocation defect, and it is in *flight*

Per-phase worst / median defect at the shipped point:

| phase | segs | worst | median |
|---|---|---|---|
| load | 11 | 1.71e-2 | 2.04e-4 |
| launch | 11 | 4.29e-2 | 1.27e-2 |
| **flight** | **49** | **1.375e-1** | **5.12e-2** |
| absorb | 15 | 9.93e-4 | 3.37e-5 |

Not one non-defect constraint is violated above 0 to printing precision.

### M2 — every large defect row is a *joint acceleration*, never a base row

Top rows by worst defect are all `d_<leg>_<joint>_joint`. The base rows (quaternion,
position, angular and linear velocity) do not appear in the top 12 at all.

    d_FR_hip    1.375e-1   load 1.6e-2  launch 4.3e-2  flight 1.4e-1  absorb 7.0e-6
    d_RL_thigh  1.072e-1   load 1.2e-2  launch 2.2e-2  flight 1.1e-1  absorb 2.9e-4
    d_RL_calf   1.002e-1   load 2.5e-3  launch 1.7e-3  flight 1.0e-1  absorb 9.9e-4

**Reading:** these rows are in rad/s^2, and flight joint accelerations are O(100), so
5e-2 is ~5e-4 *relative*. The base rows are small in absolute terms because the base
accelerates slowly, not because they are better converged. `max_violation` is an
unscaled max over rows whose natural magnitudes differ by ~4 orders — it is dominated
by the fastest-accelerating DOF by construction. This is a new reason to distrust it,
on top of the two in STATUS.

### M3 — `SetVariableScaling` is a no-op under IPOPT

Drake emits `IpoptSolver doesn't support the feature of variable scaling`.
`program.py:_scale()` (LAMBDA_SCALE on every contact force and the impulse) has
therefore **never done anything in any IPOPT solve in this repo's history**. It is
live only for SNOPT, which has never returned success here. Combined with M2, the
problem has effectively never been scaled at all.

### M4 — every restart burst is 300 iterations of IPOPT's *restoration phase*

The single most important measurement of the campaign. A burst seeded on the shipped point
reads:

    iter  objective    inf_pr    inf_du   lg(mu)
       0  2.783e+01  1.38e-01  9.17e+00    0.0
       1  2.783e+01  1.37e-01  3.57e+01   -4.4   alpha_pr 6.0e-05
       2r 2.783e+01  1.37e-01  9.99e+02   -1.9   <-- restoration, at iteration TWO
     ...
     209r 1.704e+01  1.53e+00  1.05e+03          <-- still in restoration 200 iters later

It enters restoration on the second iteration and never returns to a normal iteration for the
rest of the burst. `inf_pr` does not improve there — it *grows*, 0.138 -> 0.165 by iteration 29
and to 18 by iteration 206. Restoration minimises a different merit function (an l1 measure with
its own proximal term), so "make the constraint violation smaller" is not what it is doing.

**This reinterprets the restart loop.** Its documented behaviour — good points found early,
then 16 straight bursts of degradation, three runs in a row — is not IPOPT "wandering away from
good points". It is a random walk in restoration space, restarted from wherever the previous
walk stopped. The chaining that was supposed to explore is chaining restoration failures.

So the question the campaign has to answer is narrower than "how do we converge": **why does
the filter line search fail on the second iteration?** Everything below is an attempt at that.

### M5 — `add_cost` has no symmetry term, and its docstring says it does

The docstring promises "the symmetry cost is NOT scaled ... the symmetry term is the one the
audit actually requires, so it is deliberately outside the scaling and survives at scale=0".
There is no such term: `add_cost` adds torque, rate, time and tuck, and nothing else. So
`--cost-scale 0` gives an *identically flat* objective, not the symmetry-only pass the flag's
own help text describes. That matters because a flat objective is the documented trigger for
the degenerate-null-space stall `--proximal` was built for. Docstring corrected.

---

## Experiment ledger

Status: `RUN` in flight · `WIN` improved the audit · `NULL` no effect · `LOSS` worse ·
`DEAD` ruled out, do not retry.

### The screen

Full 20-burst searches take hours; the decisive signal turned out to be visible in one
80-iteration burst from the shipped point. `scratchpad/screen.sh` runs that, and the
readout is **min `inf_pr` reached** and **how many of the iterations were restoration**.
Screening first, then promoting only winners to real searches, is what made a
16-hypothesis sweep affordable at all.

### Screen results, 80 iterations from the shipped point (seed inf_pr 1.375e-01, 9/11 @ 1.97x)

| option | min inf_pr | resto/iters | burst end | audit |
|---|---|---|---|---|
| `nlp_scaling_method=none` | **8.28e-03** | 31/81 | 0.1375 | 9/11 @ 1.97x |
| `nlp_scaling_method=none` + `--flight-amom 1e-4` | 3.23e-02 | **0**/80 | — | — |
| `nlp_scaling_method=none` + monotone | 1.23e-01 | 0/81 | 0.1375 | 9/11 @ 1.97x |
| `bound_mult_init_method=mu-based` | 1.32e-01 | 0/81 | 0.1375 | 9/11 @ 1.97x |
| **`--flight-amom 1e-4`** | 1.37e-01 | 79/81 | 0.7831 | **10/11 @ 1.15x** |
| `mu_strategy=monotone`, `mu_init=1e-6` | 1.37e-01 | 0/81 | 0.1390 | 9/11 @ 1.97x |
| `accept_every_trial_step=yes` | 1.37e-01 | 0/81 | 0.1375 | 9/11 @ 1.97x |
| `required_infeasibility_reduction=0.1` | 1.37e-01 | 15/81 | 0.1375 | 9/11 @ 1.97x |
| `bound_push=1e-6` | 1.37e-01 | 0/81 | 0.1375 | 9/11 @ 1.97x |
| `mu_init=1e-2` monotone | 1.37e-01 | 0/81 | 0.1495 | 9/11 @ 1.88x |
| *(baseline)* | 1.37e-01 | 79/81 | 0.5033 | 9/11 @ 1.87x |
| `--cost-scale 0` | 1.37e-01 | 79/81 | 0.5034 | 9/11 @ 1.87x |
| `--cost-scale 0 --proximal 1e-3` | 1.37e-01 | 79/81 | 0.5034 | 9/11 @ 1.87x |
| `limited_memory_max_history=50` | 1.37e-01 | 79/81 | 0.5033 | 9/11 @ 1.87x |
| `least_square_init_duals=yes` | 1.37e-01 | 79/81 | 0.5033 | 9/11 @ 1.87x |
| `max_soc=10` | 1.37e-01 | 79/81 | 0.5033 | 9/11 @ 1.87x |
| `warm_start_init_point=yes` | 1.37e-01 | 65/81 | 0.3808 | 9/11 @ 1.87x |
| `bound_push=1e-4` | 1.37e-01 | 69/81 | 0.6398 | 9/11 @ 1.82x |
| `nlp_scaling_max_gradient` 1e-2 / 1e-1 / 1 / 10 | 1.38e-01 | 0-2/81 | 0.1375 | 9/11 @ 1.97x |
| `mu_init=1e-1` monotone | 1.38e-01 | 0/81 | 0.1501 | 9/11 @ 1.90x |
| `bound_push=1e-2` (IPOPT default) | 3.14e+00 | 0/81 | 0.1375 | 9/11 @ 1.97x |

**Only the scaling options move the needle.**

 **Only the scaling options move the needle.** Six of the sweep — the cost, the proximal
term, the L-BFGS history, the dual initialisation, second-order corrections — are
*bit-identical to the baseline* at 0.5033/0.5034, which says they do not change the
iteration at all once restoration owns the burst. Note also that the 80-iteration screen
does **not** resolve `nlp_scaling_max_gradient`: all four values look like the baseline
here, yet `=1` went on to produce 9/11 @ 1.25x in a full 300-iteration burst. The screen
is a filter for large effects, not a ranking.

| # | hypothesis | change | predicted | measured | verdict |
|---|---|---|---|---|---|
| H1 | The performance cost pulls IPOPT off the feasible manifold, so drop it | `--cost-scale 0`, `--proximal 1e-3` | violation falls | **identical to baseline** (0.5034 vs 0.5033) | `DEAD` |
| H2 | L-BFGS history of 6 is too short for 6566 variables | `limited_memory_max_history=50` | better steps | identical to baseline | `DEAD` |
| H3 | Bursts restart with garbage duals (`inf_du` 9.2 at iter 0) | `least_square_init_duals=yes`, `bound_mult_init_method=mu-based` | fewer wasted iterations | identical / no gain | `DEAD` |
| H4 | The filter rejects good steps (Maratos), so allow more corrections | `max_soc=10` | escapes restoration | identical to baseline | `DEAD` |
| H5 | `bound_push=1e-8` leaves 282 vars on their bounds, singular barrier | `bound_push` 1e-2 / 1e-4 / 1e-6 | escapes restoration | 1e-2 starts at inf_pr 10.7; no variant beat 1e-8 | `DEAD` — the 2026-09-03 setting stands |
| H6 | **IPOPT's gradient-based auto-scaling is distorting the problem** | `nlp_scaling_method=none` | — | **inf_pr 8.28e-03, 16x better than the seed** | `WIN` |
| H7 | Same, via a much more aggressive scale cap | `nlp_scaling_max_gradient=1` | — | full search burst 0: **9/11 at 1.25x**, AM 1.97e-3 -> 1.25e-3, integration 6.30e-3 -> 5.38e-3 | `WIN` |
| H8 | Flight angular momentum is exactly conserved by the true dynamics, so assert it | `--flight-amom 1e-4` (49 new rows) | AM check passes | LICQ clean: nullity **16 with and without**. Search pending | `RUN` |
| H9 | `nlp_scaling_method=none` alone will produce a better trajectory, not just a lower violation | full 300-iter burst | audit improves with violation | **viol 0.1375 -> 0.0012 (114x better) and the audit goes 9/11 @ 1.97x -> 8/11 @ 4.46x.** AM drift 1.97e-3 -> 4.46e-3, integration drift 6.30e-3 -> 9.92e-3, both WORSE | `LOSS` on its own — see below |

### M6 — the anti-correlation, confirmed a third time and much more sharply

`nlp_scaling_method=none` is a genuinely better *optimizer*: it takes `max_violation` from
0.1375 to 0.0012 in one burst with **zero restoration iterations**. Every physics check that
matters got worse doing it.

| | shipped | noscale, 1 burst |
|---|---|---|
| `max_violation` | 0.1375 | **0.0012** (114x better) |
| flight angular-momentum drift | 1.97e-3 | **4.46e-3** |
| flight integration drift (q) | 6.30e-3 | **9.92e-3** |
| audit | 9/11 @ 1.97x | **8/11 @ 4.46x** |

This is the same spurious discrete solution STATUS records at viol 0.0004 (7/11) and 0.0495
(9/11), reproduced with a much stronger solver — so it is a property of the *formulation*, not
of how hard the search was pushed. **A better optimizer on this formulation makes a worse
trajectory.** The formulation has to say what "good" means before the extra convergence is
worth anything, which is precisely what `--flight-amom` adds.

### M7 — truncation error, not residual infeasibility, now dominates the integration check

At the shipped point (viol 0.1375) STATUS concluded mesh refinement was pointless because
"the discretization error is an order of magnitude below the residual infeasibility". That
conclusion was correct *at that violation* and is now obsolete. At viol 1.2e-3 the defects can
account for roughly 2e-4 of position drift over the 0.66 s flight phase; the measured drift is
9.9e-3, ~50x more. The remainder is Hermite-Simpson truncation, which is what a finer mesh
actually buys.

**So the 2026-09-05 negative result on 50 -> 99 knots does not transfer.** It was measured
where refinement could not help. Once a run converges under `noscale` + `--flight-amom`,
refinement is the designated lever for the one remaining check, and 4th-order convergence
predicts ~16x on a bisection — far more than the 1.15x of margin needed.

---

## The two levers, and why they are separate

By mid-campaign the picture is that the two failing checks have **different** causes and need
different fixes, which is why nothing tried before moved both:

| failing check | cause | fix |
|---|---|---|
| collocation vs a tight integrator | IPOPT's default constraint scaling; the burst never leaves restoration | `nlp_scaling_max_gradient=1` |
| flight angular momentum | not asserted anywhere; a better-converged point makes it *worse* | `--flight-amom` |

Evidence for the split, all from bursts seeded on the shipped point:

- `nlp_scaling_max_gradient=1`, no momentum rows (run `e2`): burst 1 gives **10/11**, the
  only failure being angular momentum at 1.50e-3. **The integration check passes.**
- `--flight-amom 1e-4`, default scaling (screen `amom`): **10/11**, the only failure being
  the integration check at 5.75e-3. **The angular-momentum check passes.**

Each fixes exactly the check the other leaves. The runs that matter now are the combination.

| # | hypothesis | change | predicted | measured | verdict |
|---|---|---|---|---|---|
| H10 | Anchoring every flight knot to knot 0 is fine | `--amom-mode anchor` | — | `noscale`+anchor reached inf_pr **3.95** by iter 135 where the identical run without the rows was at **2.87e-03**. 49 rows reaching back to knot 0 put a dense block through a banded Jacobian | `LOSS` — `chain` is now the default |
| H11 | **`nlp_scaling_max_gradient=1` + `--flight-amom` gives 11/11** | both | 11/11 | runs `f4`,`k1`-`k4` | `RUN` |

## M8 — a between-knot readout, because knot checks are what got us here

**`tools/check_tape.py`** (new, tracked) measures, on the resampled 500 Hz tape rather than
at knots: torque
against the enforced halfplanes and against hardware, and flight angular-momentum drift.
Flight is detected from foot clearance, not base height — base height is well above its start
value during launch and absorb, which are contact phases where momentum is *not* conserved,
and using it reports a meaningless ~1.9 N.m.s of "drift".

| point | AM drift at knots | AM drift on the tape | tape vs design envelope |
|---|---|---|---|
| shipped reference | 1.97e-3 | 2.24e-3 | +2.33 N.m on 2.60% |
| `amom` screen (10/11) | 6.51e-4 | **1.71e-3** | +2.33 N.m on 2.60% |
| `e2` burst 0 | 1.25e-3 | 8.54e-4 | +8.61 N.m on 1.15% |
| `e1` burst 0 (viol 0.0012) | 4.46e-3 | 2.92e-3 | +5.97 N.m on 5.21% |

Two things to carry forward. **Enforcing the momentum box at knots does not buy it on the
tape** — the `amom` point holds 6.5e-4 at knots and rings to 1.7e-3 between them, which would
fail the 1e-3 bound if the bound were measured where the RL stage actually reads. And the
better-converged points have *worse* torque ringing, up to +8.6 N.m over the design envelope.
A candidate has to be judged here, not only by the audit.

It is deliberately **not** a twelfth audit check — the campaign's target is the existing
eleven, and a trajectory can pass `audit.py` and fail here. On the shipped reference it
already reports two failures the audit cannot see:

    [FAIL] torque inside the enforced design envelope  -- worst +2.3343 N.m, on 2.60% of samples
    [PASS] torque inside the HARDWARE peak            -- worst -0.4808 N.m
    [FAIL] flight angular momentum conserved          -- max drift 2.24e-03, bound 1e-03

This closes the TODO STATUS has carried since 2026-09-05 ("auditing the resampled tape, not
just the knots, is still an open TODO").
