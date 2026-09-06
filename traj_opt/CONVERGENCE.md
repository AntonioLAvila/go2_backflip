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
| H16 | Refining the flight mesh cuts it | `GO2_FLIGHT_KNOTS=99` from the 11/11 point | less ringing, audit held | runs `w1`,`w2` | `RUN` |

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
