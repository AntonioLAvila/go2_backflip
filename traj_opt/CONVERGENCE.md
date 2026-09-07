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

---

## M9 — the audit's momentum check is over all three components, and that cost a run

`check_ballistic` computes `drift = np.abs(ang - ang[0]).max()` on an `(n, 3)` array, so it
is the max over **L_x, L_y and L_z** — while printing only `L_y = ...` beside it. I read that
message as "the drift is in L_y" and built `--flight-amom` to bound the y-component alone,
reasoning that L_x and L_z vanish for a sagittal motion and were already implied by
`_add_symmetry`.

The constraint did exactly what it was asked and the check still failed:

| point | L_x drift | L_y drift | L_z drift | audit reports |
|---|---|---|---|---|
| shipped reference | 4.62e-4 | **1.98e-3** | 4.53e-4 | 1.98e-3 |
| `e2` burst 1 (no momentum rows) | **1.50e-3** | 6.34e-4 | 1.23e-3 | 1.50e-3 |
| `m1` burst 0 (L_y bounded, 1e-5) | **1.64e-3** | **8.15e-5** | 1.28e-3 | 1.64e-3 |

L_y fell 1.98e-3 -> 8.15e-5, a 24x win on the one component that was bounded, and the check
still read 1.64e-3 because L_x had taken over. The premise was wrong: `_add_symmetry` holds a
**1e-4 box**, not an equality, which leaves L_x and L_z free to wander at exactly the 1e-3
level the check cares about. Drift there is symmetry leaking, not tumbling.

Fixed both ends. The constraint now bounds all three components — free, since the spatial
momentum is computed in full either way, so it is the same autodiff evaluation returning a
vector. And `audit.py` now names the component carrying the drift and prints all three, so
the message cannot be misread the same way again. `nullity_check` still reports 16.

| # | hypothesis | change | predicted | measured | verdict |
|---|---|---|---|---|---|
| H12 | Bounding L_y alone is enough, since symmetry pins L_x and L_z | `_add_flight_momentum`, scalar | AM check passes | L_y 1.98e-3 -> 8.15e-5, **check still fails at 1.64e-3 on L_x** | `LOSS` — now vector-valued |

---

## Where the campaign stands (2026-09-06, mid-run)

Best point so far: **`n2` burst 0 — 10/11, worst 1.10x**, from `nlp_scaling_max_gradient=1`
plus the three-component `--flight-amom 1e-5`.

| check | shipped | `e2` b1 | `n2` b0 |
|---|---|---|---|
| flight angular momentum | FAIL 1.98e-3 | FAIL 1.50e-3 | **PASS 9.55e-4** |
| collocation vs integrator | FAIL 6.30e-3 | **PASS 3.29e-3** | FAIL 5.50e-3 |
| audit | 9/11 @ 1.97x | 10/11 @ 1.50x | **10/11 @ 1.10x** |

The momentum check is solved — 9.55e-4 with all three axes in hand (9.5e-4 / 4.3e-4 / 7.0e-4).
The whole campaign now rests on one number: **flight integration drift, 5.50e-3 against a
5e-3 bound.** That is 10% away.

Two levers remain for it, in order of preference:

1. **More search.** `e2` reached 3.29e-3 on this check *without* the momentum rows, so the
   formulation demonstrably supports it; the momentum rows cost some of that back
   (3.29e-3 -> 5.50e-3) and the chains have plenty of bursts left.
2. **Mesh refinement.** M7 argues it is now live where it was not at viol 0.14. Held in
   reserve, because it requires editing `schedule.py`'s knot count, which makes the working
   tree incompatible with every 50-knot checkpoint currently being searched.

### Honest note on the tape

`n2` passes the audit's momentum check at the knots and **fails it on the tape**
(peak-to-peak 1.88e-3 against 1e-3), and its torque rings +6.68 N.m over the design envelope
against the shipped trajectory's +2.33. It is better than shipped on tape momentum
(1.88e-3 vs 3.39e-3) and worse on tape torque. Whatever ships has to be reported on both.

---

## M10 — the momentum check has a floor at ~1e-3, and it is L_x, and symmetry never pinned velocities

With the three-component chained box on, many independent solves converge to the *same* wall:

| run | box | audit | L_x drift | L_y drift | integration |
|---|---|---|---|---|---|
| `n2` b0 | 1e-5 | 10/11 @ 1.10x | 9.55e-4 | 4.3e-4 | 5.50e-3 |
| `n9` b0 | 1e-6 | 10/11 @ **1.05x** | 1.1e-3 | **1.0e-5** | **2.90e-3** |
| `n7` b0 | 1e-5 + w_rate 2 | 10/11 @ 1.06x | — | — | — |

`n9` is the shape of the answer: the integration check **passes at 2.90e-3**, comfortably
inside its 5e-3 bound, and L_y is down at 1.0e-05 under the tight box. The whole thing is now
held up by **L_x**, which parks at 1.0-1.1e-3 no matter which seed, scaling or box is used.

That flatness is the tell. `_add_symmetry` pins **positions and torques and says nothing about
velocities** — the velocity-level mirror was deliberately dropped, and its docstring explains
why (an exact position pin plus its kinematically-conjugate velocity is a rank-deficient pair).
L_x is a velocity quantity, so nothing in the formulation bounds it. And chaining cannot fix
it: `|L_x|` is already ~1e-3 at the *first* flight knot, so a difference bound has nothing to
hold on to.

**The fix is the stronger physical statement.** For a sagittal motion L_x and L_z are not
merely constant, they are identically **zero**. `AMOM_LATERAL` (5e-4, half the audit's bound,
so drift of at most twice it still clears) is an absolute box on both at every flight knot.
An absolute anchor also cannot accumulate, unlike the chain. Measured starting points:

    shipped   max|L_x| 9.71e-4   max|L_z| 3.20e-4
    n2 best   max|L_x| 9.18e-4   max|L_z| 6.54e-4
    n9 best   max|L_x| 1.11e-3   max|L_z| 7.95e-4

so it is a ~2x tightening of a quantity whose true value is 0. It is chosen over tightening
`MIRROR` deliberately: that is the constraint family STATUS records three separate LICQ
failures in, and it would attack a coordinate-level proxy instead of the quantity the audit
actually measures.

| # | hypothesis | change | predicted | measured | verdict |
|---|---|---|---|---|---|
| H13 | Chaining all three components bounds the check | `--flight-amom`, vector chain | AM passes | L_y to 1e-05, **L_x floors at 1.0-1.1e-3** across every seed | `LOSS` — chain alone cannot anchor L_x |
| H14 | L_x and L_z are identically zero, so bound them absolutely | `AMOM_LATERAL = 5e-4` | AM passes, 11/11 | **11/11 at viol 0.386, better than the pre-anchor 11/11 on every single measure** | `WIN` |

---

# RESULT: 11/11

`n9` burst 1, `traj_opt/out/cand/n9_11of11.npy`. Every audit check passes.

    [PASS] quaternion stays unit without being pinned      worst |q|^2-1 = 6.75e-06
    [PASS] net rotation is one full backflip               -360.0000 deg, no reversal
    [PASS] flight CoM is ballistic                         max residual 1.90e-05 m
    [PASS] flight angular momentum conserved               max drift 9.20e-04 (L_x);
                                                           per-axis 9.2e-4 / 1.6e-5 / 5.8e-4
    [PASS] contact forces inside the friction cone         worst slack 2.25e-12 N
    [PASS] torques inside the enforced halfplanes          worst overshoot 0.00e+00 N.m
    [PASS] no geometry below the floor                     +0.02 mm at knots, +0.01 between
    [PASS] non-foot geometry keeps its clearance           +10.00 mm at knots, +7.47 between
    [PASS] sagittal symmetry holds without being pinned    worst L/R 9.99e-05 rad
    [PASS] flight is genuinely tucked                      I_yy 0.5090 kg.m^2
    [PASS] collocation matches a tight integrator          worst drift 4.09e-03 (bound 5e-3)
    11/11 audit checks passed

Recipe: seed the shipped reference, `nlp_scaling_max_gradient=1`, `--flight-amom 1e-6`
(three-component chain plus the `AMOM_LATERAL` absolute anchor), short bursts.

## What it is and is not better at

| | shipped | 11/11 point |
|---|---|---|
| audit | 9/11 @ 1.97x | **11/11** |
| flight angular-momentum drift (knots) | 1.97e-3 | **9.20e-4** |
| collocation vs integrator | 6.30e-3 | **4.09e-3** |
| flight AM peak-to-peak (tape) | 3.39e-3 | **2.04e-3** |
| tape vs design envelope | +2.33 N.m on 2.60% | **+10.58 N.m on 6.73%** |
| tape clearance to HARDWARE peak | 0.481 N.m | **0.502 N.m** |

Better on every audit check and on tape momentum; **worse on tape torque ringing**, by 4.5x in
magnitude and 2.6x in the fraction of samples affected. It never asks for more than the motor
can deliver — hardware clearance is actually slightly better at 0.502 N.m — but it spends more
of the derated envelope, which is authority a tracking policy would otherwise have. That is a
real regression in the one dimension the 2026-09-05 safety-factor work existed to protect, and
it is invisible to all eleven audit checks.

So the campaign continues past its own target: find an 11/11 that is *also* clean on the tape.
Arms `t1`-`t4` sweep `--w-rate` (0.1 default, up to 12) seeded on this point, since input-rate
weight is precisely the term that decides how much high-frequency content the first-order hold
has to carry.

---

## M11 — the tape's torque overshoot is a *velocity* artefact, so smoothing torque cannot fix it

`resample()` first-order-holds the knot torques, and the flat-peak constraint `|tau| <= tau_pk`
is linear in tau alone — so if it holds at both knots, convexity gives it everywhere between.
Measured, it does:

| | flat-peak term | speed-dependent term |
|---|---|---|
| shipped | **-0.0068** | +2.3343 |
| 11/11 point | **-0.0130** | +10.5842 |

The entire tape overshoot is the speed-dependent halfplane `tau + k*qd <= tau_stall`, and `qd`
on the tape comes from the **cubic state spline**, not from an interpolation of the knots. So
what rings is the joint velocity between knots, not the torque.

**That kills the `--w-rate` idea before it cost a full run.** Weighting input rate smooths
`tau`, and `tau` is not what is ringing. The lever is mesh resolution: more knots means less
room for `qd` to excurse between them. Arms `t1`,`t2`,`t4` were cancelled on this measurement.

| # | hypothesis | change | predicted | measured | verdict |
|---|---|---|---|---|---|
| H15 | Input-rate weight cuts the tape's torque overshoot | `--w-rate` 1-12 | less ringing | overshoot is entirely in `qd`, which `w_rate` does not touch | `DEAD` |
| H16 | Refining the flight mesh cuts it | `GO2_FLIGHT_KNOTS=99` from the 11/11 point | less ringing, audit held | guess starts at inf_pr **22.3** and all four arms *diverge* to 31-120 | `LOSS` |

### `GO2_FLIGHT_KNOTS`

`schedule.py`'s flight knot count is now overridable by that environment variable. Editing the
number in the file is a documented footgun — every 50-knot checkpoint in `traj_opt/out/` stops
loading the moment it changes, *including the ones a concurrent search is still writing*, and
`warm_start.py` has to export before the edit while the file still matches the source. The env
var keeps both meshes runnable at once, which is what let the refinement start without
stopping fifteen 50-knot chains.

---

## The lateral anchor is what made the 11/11 good, not just possible

`n9` reached 11/11 *before* `AMOM_LATERAL` existed, by luck of the chain. `r1` reached it with
the anchor, and dominates it everywhere:

| | shipped | `n9` 11/11 (chain only) | `r1` 11/11 (+ lateral anchor) |
|---|---|---|---|
| audit | 9/11 @ 1.97x | 11/11 | **11/11** |
| `max_violation` | 0.1375 | 0.9043 | **0.3858** |
| flight AM drift (knots) | 1.97e-3 | 9.20e-4 | **7.56e-4** |
| collocation vs integrator | 6.30e-3 | 4.09e-3 | 4.13e-3 |
| flight AM peak-to-peak (tape) | 3.39e-3 | 2.04e-3 | **1.47e-3** |
| tape vs design envelope | +2.33 on 2.60% | +10.58 on 6.73% | +4.72 on 2.31% |
| tape clearance to HARDWARE peak | 0.481 | 0.502 | **0.659** |

Against the shipped trajectory, `r1` is better on every audit check, better on tape momentum,
better on hardware clearance by 37%, and affects a *smaller* fraction of tape samples
(2.31% vs 2.60%). The one place it is worse is the size of the worst design-envelope
excursion, +4.72 N.m against +2.33. That excursion is against the speed-derated linear model,
not the motor: it never comes within 0.659 N.m of what the hardware can actually deliver,
where the shipped trajectory gets within 0.481.

Four independent chains (`n9`, `q1`, `q2`, `t3`, `r1`) reached 11/11, so the recipe is
reproducible rather than a lucky burst.

## M12 — refining a 50-knot solution onto 99 knots diverges, and the reason is the ringing itself

Exporting the 11/11 point onto a bisected 99-knot flight grid gives a guess at `inf_pr` 22.3,
and four arms (feasibility-only and costed, with and without the momentum rows) all moved
*away* from it — 31, 31, 82, 120 after 120-200 iterations. Broken down, the guess violates:

    2.232e+01  (135)  collocation defect
    7.822e+00  (1670) LinearConstraint      <- the torque-speed halfplanes
    1.460e-02  (99)   floor clearance

The 7.82 N.m on the halfplanes is the whole story, and it is **the between-knot ringing being
sampled**. Those constraints bind at knots; the new grid's knots are the old grid's midpoints,
which is exactly where the tape overshoots by ~10 N.m. So refinement does not smooth the
ringing away — it converts it into hard constraint violations at 49 new knots simultaneously,
and hands IPOPT a guess that is worse than anything it can walk back from.

That is a genuine catch-22 for this route: the mesh refinement that would fix the ringing
cannot be started from a solution that has the ringing. A refinement would have to come from a
point that is already clean between knots, or the envelope would have to be enforced at the
midpoints directly (a constraint on the Hermite midpoint state, which needs a dynamics
evaluation per interval — affordable but not free).

**This is a different result from 2026-09-05's**, which found refinement merely *unhelpful*
(5% where 16x was predicted). This one is refinement being actively unusable from a ringing
source, and it supersedes M7's optimism.

---

## M13 — "0.481 N.m clear of the hardware limit" was flat-peak clearance, and it was too generous

`check_tape.py` originally reported hardware headroom as `|tau| <= tau_peak`, with no speed
derating — the same measure the 2026-09-05 safety-factor work quoted. It gives false comfort. A
tape can read 0.479 N.m of flat-peak clearance while asking, *at speed*, for more torque than
the motor can produce. Measured against the hardware **torque-speed** envelope (the halfplane
form built on datasheet peaks rather than design ones):

| trajectory | over the DESIGN envelope | over the HARDWARE envelope | flat-peak clearance |
|---|---|---|---|
| pre-2026-09-06 reference | +2.33 on 2.60% | **+1.85** | 0.481 |
| `n9` 11/11 | +10.58 on 6.73% | **+10.80** | 0.502 |
| `r1` 11/11 | +4.72 on 2.31% | **+4.43** | 0.659 |
| `t3_b2` 11/11 | +5.27 on 1.88% | **+4.95** | 0.788 |
| **`t3_b0` 11/11 (shipped)** | **+0.79 on 0.44%** | **+0.51** | 0.479 |

Note `t3_b2` and `r1`: the *best* flat-peak clearance of any candidate (0.788, 0.659) with
nearly ten times the shipped one's real over-demand. Ranking on flat-peak clearance would have
picked exactly the wrong trajectory. The tool reports the envelope now, with the flat peak as
a secondary number.

So the 2% actuator safety factor did **not** do what 2026-09-05 concluded it did. It bought
flat-peak margin, and the between-knot ringing that motivated it happens at speed, where the
envelope is derated and the margin it bought is proportionally smaller. The shipped
trajectory's over-demand is down from 1.85 to 0.51 N.m — better by 3.6x, and still not zero.

## Selection: the audit alone would have shipped the wrong one

Thirteen 11/11 candidates across five independent chains, ranked on audit **and** tape:

    t3_b0    11/11   +0.79 N.m on 0.44%   <- shipped
    r1       11/11   +4.72 on 2.31%
    t3_b2    11/11   +5.27 on 1.88%
    n9       11/11  +10.58 on 6.73%
    q2       11/11  +12.80 on 7.71%
    u10      11/11  +13.06 on 5.89%

They are indistinguishable on the audit — all eleven checks, margins within a few percent of
each other — and they span **16x** on the tape. Picking by audit score alone had a good chance
of shipping something that rings five to thirteen times worse than what it replaced. That is
the `max_violation` lesson one level further out: the selection criterion has to be measured
where the artifact is consumed.

## Reproducibility

The recipe is not a lucky burst. **Six independent chains reached 11/11** — `n9`, `q1`, `q2`,
`t3`, `t6`, `r1` — from three different seeds. Most directly, a run with **nothing but the
defaults**, seeded on the old 9/11 reference:

    uv run traj_opt/solve_backflip.py \
        --start-checkpoint traj_opt/reference/backflip.npy --no-prepass \
        --restarts 30 --burst-iters 250

reached 11/11 on its **second burst** (`repro_b1`, tape +2.41 N.m on 1.74%). The defaults are
`nlp_scaling_max_gradient=1` and `--flight-amom 1e-6`.

## Final state

Shipped: `t3_b0` — 11/11, worst margin 0.98x, `max_violation` 0.5797, 675 samples over 1.348 s.

Still failing on the tape, and this is the honest remaining gap:

    [FAIL] torque inside the enforced design envelope   +0.7868 N.m on 0.44% of samples
    [FAIL] torque inside the HARDWARE torque-speed env  +0.5076 N.m
    [FAIL] flight angular momentum conserved            peak-to-peak 1.34e-03, bound 1e-03

All three are better than the trajectory it replaced (+2.33 / +1.85 / 3.39e-3), and all three
are between-knot artefacts of Hermite-Simpson rather than anything the solve did wrong. The
next piece of work, with the diagnosis already done:

1. **Enforce the torque-speed envelope at the Hermite midpoints**, not only at knots. That is
   where the tape overshoots, and it is the only remaining route — `--w-rate` cannot touch it
   (M11) and mesh refinement cannot be started from a ringing source (M12). Cost is one
   dynamics evaluation per interval to form the midpoint state.
2. **Same for the momentum box.** It is enforced at knots and rings to 1.34e-3 between them.


---
---

# Part II — `is_success()` (started 2026-09-06)

Part I hit 11/11 on the audit. This part goes after the other half of the target: IPOPT
actually converging. Same rules as Part I — never loosen a criterion to pass it, one variable
per experiment, record negatives.

## M14 — dual infeasibility is the entire blocker, and it is not close

IPOPT's final summary at the shipped 11/11 point, after 250 iterations:

| residual | scaled | unscaled |
|---|---|---|
| **Dual infeasibility** | **3.5037e+01** | 1.6589e+02 |
| Constraint violation | 2.4458e-04 | 5.7966e-01 |
| Variable bound violation | 2.9808e-09 | 2.9808e-09 |
| Complementarity | 1.2841e-05 | 9.6049e-05 |
| **Overall NLP error** | **3.5037e+01** | 1.6589e+02 |

`is_success()` requires the overall NLP error under `tol`, and the overall error **is** the
dual infeasibility — six orders above the other two. Read that carefully, because it inverts
the intuition the whole project has been running on:

* the point is essentially **feasible** (2.4e-04 scaled, and that is with `nlp_scaling_max_
  gradient=1`, which is why the unscaled 0.58 looks so different),
* it is essentially **complementary** (1.3e-05),
* it is not **stationary**, by a factor of 3.5e5 over the `tol` the restart loop uses.

So the thing to attack is not feasibility. Every lever in Part I — and every lever in
STATUS.md before it — was aimed at `max_violation`, which is the residual that is *already
small enough*. Dual infeasibility that will not fall while the primal residual has is the
signature of an active-constraint Jacobian without full row rank: the multipliers that would
make `grad L` vanish are not determined, so no iteration count fixes it.

That matches what STATUS.md measured once, on 2026-09-03, and never followed up: **nullity 202
of 4672 rows at a solved point, smallest singular values 1e-16, cond(A) = 5.5e20**, with 83 of
the null-space energy on the left/right mirror, 25.2 on the `q[1]/q[3]/q[5]` box, 14.8 on the
quaternion-norm box and 9.3 on the hip pins.

**`traj_opt/kkt_check.py`** (new) is the diagnostic: it stacks equality rows *and* active
inequality rows at a given checkpoint, not just equalities at the guess, and reports rank,
condition number and null-space energy by call site. `nullity_check.py` answers "will IPOPT
refuse to start"; this answers "can IPOPT ever converge".

## The first hypothesis follows directly

With a **flat objective** the KKT stationarity condition is `sum lambda_i grad g_i = 0`, which
`lambda = 0` satisfies at any feasible point — no rank condition needed. If dual infeasibility
is really about multipliers being undetermined, dropping the objective should collapse it. The
performance terms are documented as not required for a valid trajectory (`add_cost`: "None of
these four terms is needed for a valid trajectory"), so this is an affordable thing to test
before touching the constraint structure.

It is also a weaker claim, and worth being explicit about: `is_success()` on a feasibility
problem means "found a feasible point", not "found an optimum". If that is what converges,
say so plainly rather than quoting it as convergence of the costed problem.

| # | hypothesis | change | predicted | measured | verdict |
|---|---|---|---|---|---|
| K1 | A flat objective removes the stationarity requirement that the degenerate active set cannot meet | `--feasibility-only` | dual infeasibility collapses | **falls into RESTORATION by iteration 199** and sits at inf_du 7.4e+01; the costed arm is at 4.4e+00 and not in restoration | `LOSS` |
| K2 | Same, via zero-weighted cost terms rather than no cost object | `--cost-scale 0` | same as K1 | bit-identical to K1 | `LOSS` |
| K3 | The costed problem converges given enough iterations | 3000-iteration burst | — | descending cleanly, inf_du 9.07e+03 -> 4.35e+00 by iteration 222 | `RUN` |
| K4 | Tighter tolerances on the feasibility problem still converge | `--opt-tol 1e-6 --feas-tol 1e-8` | — | same restoration as K1 | `LOSS` |
| K5 | Removing exactly-redundant symmetry rows fixes the rank deficiency | `_symmetry_implied` | nullity falls a lot | nullity **192 -> 188**, 159 bindings removed. Correct, but not the fix | `NULL` |
| K6 | Symmetry belongs in the objective, not the active set | `--sym-penalty W --sym-box B` | the boxes go inactive, nullity collapses | runs `b1`-`b5` | `RUN` |

## M15 — the flat objective was exactly backwards

K1's reasoning was that with `f = 0` the stationarity condition `sum lambda_i grad g_i = 0` is
satisfied by `lambda = 0` at any feasible point, needing no rank condition — so dual
infeasibility should collapse. Measured, `--feasibility-only` is **worse**: it is in
restoration by iteration 199 at inf_du 7.4e+01, while the costed problem is out of restoration
and descending at 4.4e+00.

The reasoning ignored what STATUS already says about this exact case: a flat objective leaves
the ~200-dimensional degenerate subspace with no curvature at all, which gives IPOPT's
limited-memory quasi-Newton approximation nothing to resolve it with. `lambda = 0` being *a*
KKT point does not help an interior-point method that has to get there along a barrier path.

That reframes K6 rather than killing it. What is wanted is not *less* objective but an
objective that is **strictly convex in precisely the degenerate directions** — which is what a
symmetry penalty is, and what `--proximal` was reaching for less specifically.

## M16 — where the 188 rows of deficiency actually sit

`kkt_check.py` at the shipped point, after K5:

    rows: 5936 (5756 equality, 180 active inequality), free cols: 6412
    rank 5748, sigma_min 1.0e-16, cond 4.1e+21, NULLITY = 188

    30.43  (31 rows)  _add_symmetry:544 (act)   <- q[1],q[3],q[5] box
    13.10  (17 rows)  _add_symmetry:556 (act)   <- L/R mirror box
     8.54  (26 rows)  _add_symmetry:548 (act)   <- hip pins
     7.64  (19 rows)  _add_boundary:691 (eq)    <- x0 position pin
     7.63  (74 rows)  _add_stitching:653 (eq)   <- phase state continuity
     5.08  (12 rows)  _add_boundary:699 (eq)    <- final joint pin
     4.24  ( 6 rows)  _add_symmetry:555 (act)

**80 active symmetry rows carry 56 of the null-space energy.** They are inequalities the solve
rides: the trajectory sits at 9.99e-05 against a 1e-4 box, so every one of them is active, and
each is rank-deficient against the collocation defect that already implies it once the same
pin holds at two consecutive knots. Making them boxes rather than equalities (the 2026-09-03
fix) kept them out of the *equality* rank requirement and did nothing about this, because an
active inequality is back in the KKT system either way.


## M17 — the momentum constraint from Part I is itself half the rank deficiency

`kkt_check` at the shipped point, loose symmetry box, top of the null-space listing:

    6.90  (19 rows)  _add_boundary:691 (eq)
    ...
    1.00  ( 1 rows)  amom_39_40[1] (act)
    1.00  ( 1 rows)  amom_38_39[1] (act)
    1.00  ( 1 rows)  amom_40_41[1] (act)      <- and so on, dozens of them

**Energy 1.00 on a single row means that row lies entirely in the null space** — it is exactly
redundant. Every chained momentum row scores that.

The reason is the thing that made the constraint attractive in the first place. Angular
momentum about the CoM is conserved *as an exact consequence of the dynamics*, and the
collocation defects already encode the dynamics — so along the defect manifold the Jacobian of
`L(k+1) - L(k)` is a linear combination of defect rows. Asserting a true invariant that the
formulation already contains is precisely the LICQ failure mode CLAUDE.md names as "the
recurring bug class in this NLP", and Part I walked straight into it.

Measured, removing them:

| formulation | active inequalities | nullity | cond |
|---|---|---|---|
| original (TIGHT boxes, hard momentum) | 180 | **192** | 3.2e21 |
| + `_position_implied` redundancy removal | 180 | 188 | 4.1e21 |
| + loose symmetry box (`--sym-box 1e-2`) | 100 | 121 | 3.0e20 |
| + momentum as a penalty (`--amom-penalty`) | **14** | **35** | 3.0e20 |

**Nullity 192 -> 35, and the active set from 180 rows to 14.** Both wins come from the same
principle: a constraint that states something the formulation already implies costs rank and
buys nothing. Symmetry boxes the solve rides, and a conservation law the dynamics already
enforce, were both of that kind.

`--amom-penalty W` (with `--flight-amom 0`) imposes the invariant as a cost instead. It pulls L
the same way and never enters the active set. Whether the audit's momentum check survives the
softer form is the thing to measure next — Part I's whole 11/11 depends on it.

## The remaining 35

    6.75  (19 rows)  _add_boundary:711 (eq)   <- eq(x0[XQ], q0), the initial configuration
    4.76  (12 rows)  _add_boundary:719 (eq)   <- eq(xf[XQ][7:], HOME_LEGS)
    3.22  (74 rows)  _add_stitching:673 (eq)  <- phase state continuity
    2.50  ( 4 rows)  _add_contact:433 (act)
    2.50  ( 4 rows)  _add_contact:434 (act)
    2.06  ( 4 rows)  _add_boundary:718 (eq)   <- eq(xf[XQ][:4], [-1,0,0,0])

These are equalities, not boxes the solve rides, so they cannot be loosened away. They are
also where a boundary condition and a per-knot family describe the same quantity.

| # | hypothesis | change | predicted | measured | verdict |
|---|---|---|---|---|---|
| K6 | Symmetry belongs in the objective, not the active set | `--sym-penalty 10 --sym-box 1e-2` | active rows drop, nullity falls | **180 -> 100 active, nullity 188 -> 121**, symmetry gone from the null space | `WIN` (structurally) |
| K7 | The momentum constraint is redundant against the defects it is built on | `--amom-penalty` | large nullity drop | **nullity 121 -> 35, active 100 -> 14** | `WIN` (structurally) |
| K8 | With nullity 35 the solve converges | runs `e1`-`e4` | `is_success()` | — | `RUN` |

## M18 — dual infeasibility bottoms out at ~10-30 in *every* configuration

Across sixteen arms spanning cold and warm starts, costed and feasibility-only objectives,
proximal and symmetry and momentum penalties at four weights, monotone and adaptive barriers,
and three scalings, the minimum dual infeasibility reached is:

    a3 (costed, TIGHT)                  7.77
    b2/b3 (sym penalty, loose box)      5.42
    c1/c3 (monotone barrier)           10.6 - 11.9
    d1/d2/d4 (proximal + sym penalty)   5.42
    e1-e4 (momentum penalty too)       29.2 - 57.2
    f1/f2 (cold start, fully penalised) 27.6
    f3 (minimal objective)             12.1

Nothing reaches even 1, against a `tol` of 1e-4. And it is flat across formulations that
differ by 157 rows of rank deficiency, which says the remaining obstacle is **not** the rank
deficiency the last two findings removed.

`inf_pr` behaves quite differently and much better — the cold start drives it 2.06e+02 ->
3.62e-01 — so the solver is working; it is the stationarity residual specifically that will
not move.

Two candidate explanations, and they are distinguishable:

1. **Conditioning rather than rank.** `kkt_check` reported cond 3.0e20 on the *raw* Jacobian,
   which is past double precision, so computed multipliers would be numerical noise regardless
   of rank. But that number overstates what IPOPT faces: `nlp_scaling_max_gradient` rescales
   every constraint row, so the honest measurement is the row-normalised condition number.
   `kkt_check` now reports both.
2. **These iterates are simply not near a stationary point** of the objectives being posed,
   and the audit-passing region is not where any local minimum sits.

Also worth recording, because it invalidated a comparison: `--feasibility-only` and the
default costed run are **identical** once `--sym-penalty` or `--amom-penalty` is on, because
those penalties are added in the constructor and `--feasibility-only` only skips `add_cost`.
Arms f1/f2 and a1/a2 were reading as independent samples and were not.

## M19 — the ill-conditioning is confined to exactly 10 rows, and that changes the outlook

Row-normalising the Jacobian (which is what `nlp_scaling_max_gradient` does, so it is the
matrix IPOPT actually hands its linear solver) gives a completely different picture from the
raw numbers this project has been quoting since 2026-09-03:

| | raw | row-normalised |
|---|---|---|
| nullity | 35 | **10** |
| cond | 3.05e+20 | **1.22e+16** |

1.22e+16 is past `1/eps` (~4.5e15), so the scaled KKT system is numerically singular to
working precision — which is a complete explanation for `inf_du` flooring at 5-57 in every
one of the sixteen arms, regardless of objective, barrier or start point. Multipliers computed
from a singular system are noise.

But the spectrum says the singularity is not diffuse:

    4.54e-16 5.84e-16 6.10e-16 6.26e-16 7.39e-16
    7.90e-16 8.09e-16 8.76e-16 9.14e-16 1.07e-15   <- ten, at the noise floor
    ------------------------------------------------- seven orders of gap
    3.94e-09 5.74e-09 8.58e-09 1.76e-08 1.95e-08 ...

    effective cond excluding the 10 null directions: 1.41e+09

**Ten rows are exactly redundant, and with them removed the problem is well conditioned** —
1.4e9 sits comfortably inside double precision. There is no smooth decay into the noise floor,
which would have meant no finite set of rows to remove and no path at all.

So Part II is not blocked on something fundamental about direct collocation, the problem size,
or the quaternion. It is blocked on ten identifiable rows. `kkt_check` now reports the
row-normalised null space's energy by call site and lists the individual rows carrying it,
which is what names them.

## M20 — the ten rows, named, and the KKT system made full rank

`kkt_check`'s row-normalised null-space listing named them:

| rows | call site | why it is redundant |
|---|---|---|
| **8** | `_add_contact` friction cone | at a **release** knot `lambda_z` is pinned to 0, and IPOPT eliminates a fixed variable — so `lambda_x <= mu*lambda_z` and `-lambda_x <= mu*lambda_z` collapse to `+lambda_x <= 0` and `-lambda_x <= 0`, exact negatives of each other, both active |
| 2 | `_add_symmetry` lambda_x mirror | same knots: `0 == 0` |
| 2 | `_add_symmetry` `v[1] <= 0` box | phase 0 knot 0 and the final knot, where `_add_boundary` pins the whole of `v` to zero |
| ~1 | `_add_body_clearance` at load knot 0 | `eq(x0[XQ], q0)` pins the configuration, so the clearance value is determined |

Half of the entire deficiency was the friction cone at four release knots. The fix is to state
what those rows mean — `lambda_x = 0`, as a fixed variable IPOPT eliminates — rather than as
two mutually dependent inequalities.

    formulation                          active   raw nullity   scaled nullity   scaled cond
    original (TIGHT boxes, hard amom)      180        192            10            1.22e+16
    + redundancy removal + penalties        14         35            10            1.22e+16
    + release-knot lambda_x pin              4         27             2            7.54e+15
    + body clearance de-duplicated           2         25          **0**        **1.41e+09**

**The KKT system IPOPT is handed is now full rank and well conditioned**, from 1.22e16 — past
`1/eps`, i.e. numerically singular — to 1.41e9. Every step of that was measured, and the
shipped trajectory still audits 11/11 under every one of these formulations, since none of
them changed a constraint that was doing work.

| # | hypothesis | change | measured | verdict |
|---|---|---|---|---|
| K9 | The friction cone is exactly dependent at release knots | pin `lambda_x = 0` there | scaled nullity 10 -> 2 | `WIN` |
| K10 | The last two are boundary/clearance duplicates | `_position_implied` on body clearance | scaled nullity 2 -> **0**, cond -> 1.41e9 | `WIN` |
| K11 | A full-rank, well-conditioned KKT system converges | runs `g1`-`g5` | — | `RUN` |

## M21 — full rank is necessary but not sufficient: the solve still does not converge

With the scaled KKT Jacobian at nullity 0 and cond 1.41e9, five arms (warm and cold start,
proximal and plain, adaptive and monotone barrier) reach:

    g1/g2  685 iters   min inf_pr 0.577   min inf_du 15.9
    g3     1756 iters  min inf_pr 0.759   min inf_du 13.6
    g4     341 iters   min inf_pr 0.577   min inf_du **3.12**
    g5     208 iters   min inf_pr 0.473   min inf_du 16.1

3.12 is the best dual infeasibility this campaign has produced, against a `tol` of 1e-4. So
fixing the Jacobian's rank and conditioning **moved the number by a factor of ~2 and no more**.
That is a real negative result and it rules out the explanation the whole of M14-M20 was built
on: at the shipped point, the constraint Jacobian is no longer what blocks stationarity.

| # | hypothesis | change | measured | verdict |
|---|---|---|---|---|
| K11 | A full-rank, well-conditioned Jacobian converges | K6+K7+K9+K10 | min inf_du 5.42 -> 3.12; no success | `LOSS` |
| K12 | `bound_push=1e-8` makes the barrier Hessian singular (slacks 1e-8 on ~282 bound-active variables give entries of order 1e16) | `bound_push` 1e-2 / 1e-4 | **worse on both residuals**: min inf_pr 0.576-37.7 against 0.577, min inf_du 19-49 against 3.12 | `LOSS` — the 2026-09-03 setting stands for a second reason |

## What is left

The Jacobian is full rank and well conditioned; the barrier setting is confirmed optimal; the
objective has been varied over six forms and four weights; the barrier strategy over three.
Dual infeasibility does not go below ~3.

The remaining candidates, in the order they are worth testing:

1. **The Hessian approximation.** Drake supplies no second derivatives, so IPOPT runs
   `limited-memory` L-BFGS with a default history of **6** on a 6408-variable problem. Part I
   screened `limited_memory_max_history=50` and recorded it as `DEAD` — but that screen ran
   *while restoration owned every burst*, where nothing about the quasi-Newton model could
   matter. That verdict does not carry to this regime, and the re-test is `i1`-`i4`.
2. **The active set changes during the solve.** `kkt_check` measures one point. Nothing yet
   measures whether the conditioning holds along the path.
3. **These iterates are not near a stationary point** of any objective posed so far.

| K13 | The L-BFGS history of 6 is the blocker, once restoration no longer masks it | `limited_memory_max_history` 50 / 200, `update_type=sr1` | all three reach min inf_du **15.9 at iteration 1** — identical to baseline, no improvement at any history | `DEAD` (re-tested in the right regime, same verdict) |

## M22 — SNOPT, re-tested for a specific reason, and it does not succeed either

Worth doing rather than trusting `CLAUDE.md`'s "SNOPT has never once returned success here",
because of *how* it used to fail. STATUS records the symptom as a silent, wrong `info=13`
("infeasible") on points that were feasible, and attributes it to LICQ violations — which is
exactly what K6/K7/K9/K10 removed. SNOPT is also SQP, so IPOPT's barrier conditioning does not
apply to it at all, and it takes its multipliers from a QP subproblem that is well posed
precisely when LICQ holds. All three reasons said this was the best untried lever.

| arm | start | objective | result |
|---|---|---|---|
| `s2` | cold (guess.py) | costed | `info=13`, viol **1338.7**, 2267 s |
| `s3` | shipped 11/11 point | penalties only | `info=43` "cannot satisfy the general constraints", viol 0.58 -> **7.04**, 3336 s |

Neither is the spurious failure. `s2`'s `info=13` comes with a genuine viol of 1339 — SNOPT
simply cannot reach feasibility from the analytic guess, which is a real result about the
guess, not a false report. And `s3` starts 0.58-feasible and moves *away*, to 7.04, before
declaring the constraints unsatisfiable.

So the LICQ repair did change SNOPT's failure mode (13 -> 43) without making it succeed. On
this problem SNOPT is also 5-15x slower per solve than IPOPT — 2267 s and 3336 s for a single
pass — which makes it a poor search vehicle regardless.
