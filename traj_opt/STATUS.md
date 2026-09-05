# Backflip trajectory optimization — status

Last updated 2026-09-04. Not yet converged
(`is_success()==False`), and it may never need to be — see next steps. Best
confirmed result: IPOPT, **10/11 audit checks passing, worst failure 1.61x
over its threshold**, violation 0.1287, on the **50**-knot-flight problem with
the 10 mm body-clearance margin, shipped as `out/backflip.npz` and reproducible
from `out/checkpoints_backflip_k50/best_10of11.npy` with `--from-checkpoint`.
The collocation-vs-integrator check, which had failed in every session this file
records, now passes; only flight angular momentum is left.

**The headline number changed on 2026-09-04 from constraint violation to the
audit**, and the shipped trajectory changed twice that day — first to the 26-knot
`burst_38`, then to the 50-knot solve above, which is better on the violation *as
well*, the first time the two criteria have agreed. The note that follows
describes the first of those changes. `burst_38`'s violation was 0.4259 —
8.6x the 0.0495 of the point it replaced — and it is better than that point on
every physics measure there is: half the flight angular-momentum drift, 2.7x
less integration drift, an order of magnitude less CoM residual, and no pitch
reversal at all. The 0.0495 point is kept at `best_0495.npy`. Do not rank
candidates on `max_violation`; the 2026-09-04 and 2026-09-03 sections are why.
The previous best (0.0376, 6/8, no clearance margin) is kept as
`out/backflip_noclearance_0376.npz` — it is *better converged* but its rear
knees scrape the floor and its head touches down before its feet do.

> **Read the 2026-09-04 section FIRST**, then the 2026-09-03 one. Together they say the same
> thing twice: `max_violation` does not measure whether this trajectory is any good, and
> ranking on it has been actively costing the search. Two numbers make the case — a point at
> 124x lower violation that audits 7/11 (2026-09-03), and a point at 8.6x *higher* violation
> that beats the shipped one on every physics check (2026-09-04).
>
> **On the 2026-09-03 section:** It supersedes the two below on one point that
> matters before any further solving: the remaining violation is *entirely* collocation
> defect, the mesh is not short, and two of the reasons the solver could not converge were
> bugs in how IPOPT was being driven (`bound_push` wrecking every restart's seed; Drake's
> `adaptive` barrier default silently disabling `mu_init`). It also records one structural
> change that was measured and rejected -- do not re-try fixed-variable symmetry pins.
>
> **Then read the 2026-09-01 session B section**, then the one below it.
>
> **Read the 2026-09-01 section next.** It supersedes the 2026-08-25 one: the
> floor clipping and the missing tuck are fixed and measured (−71.6 mm → −0.2 mm,
> I_yy 0.668 → 0.509), a second defect of the same family (a stance foot pinned
> in place but free to move through the floor at 1.4 m/s) was found and fixed,
> and three separate levers were tried and found not to work. The 2026-08-25
> section's "in progress" checklist is done.

> **2026-08-20 note:** everything in this repo (`tools/`, `traj_opt/`, this
> file, and the `go2_mjcf` submodule fix) is now committed and pushed,
> including to the submodule's own fork remote — the "uncommitted working
> tree, one accident from data loss" state flagged in the previous note is
> resolved. See git log for the commit.
## 2026-09-05: warm_start.py never worked, and it was three re-timing bugs

`warm_start.py` has been in the repo since 2026-08-23 with a docstring promising that a phase
whose knot count is unchanged "round-trips through this essentially exactly". It did not. Fed
the shipped 50-knot point and asked for the same 50 knots back, it returned a guess at
**violation 321** that audits **7/11**, against the source's own 0.1287 and 10/11.

That is why the 2026-08-23 session concluded mesh refinement "isn't a free lunch" and why the
Files table still says warm_start is the "wrong tool for 14->20 (resampling an under-resolved
source gave violation 586)". The source's resolution was never the problem. **Every warm start
this repo has ever attempted was being silently re-timed**, and the effect is the same whether
the source is under-resolved or excellent.

Three bugs, all the same shape -- knots ending up at times other than the ones their states
were computed for:

1. **`set_guess` threw the time grid away.** It passed the guess to
   `dc.SetInitialTrajectory(u_traj, x_traj)`, which derives ONE uniform `h = duration/(N-1)`
   and then samples both trajectories at `i*h`. A solved grid is never uniform:
   `AddEqualTimeIntervalsConstraints` is a chain of adjacent equalities `h_k == h_k+1`, each
   satisfied to ~1e-4, with **nothing bounding the accumulated drift**. The shipped flight
   phase runs `h` from 0.011469 to 0.013467 -- 17% off uniform across 49 intervals.
   Substituting uniform `h` into the shipped point and changing nothing else takes it from
   **violation 0.1287 to 152.3**. Fixed by setting `dc.state(k)`, `dc.input(k)` and
   `dc.time_step(k)` per knot directly. guess.py's analytic `t` is uniform, so the cold-start
   path is unaffected either way -- which is exactly why this hid for two weeks.
2. **`export` resampled onto `np.linspace`**, a uniform grid, for the same reason. Now it
   interpolates the source grid against index fraction, which reproduces it exactly when the
   knot count is unchanged and preserves its shape when it grows.
3. **`export` sampled Drake's reconstruction spline.** The knots of a solved phase are a
   genuine trajectory -- `audit.check_integration` confirms it at 3.2e-3 -- but the cubic
   through them is only an interpolant, and it is not the trajectory between knots. Sampling
   it broke sagittal symmetry from 2.8e-4 to 3.1e-2 rad. States now come from re-shooting the
   true dynamics through the source's own first-order-held generalized force.

After all three: the round trip is **exact**. 0.128651, 10/11, worst 1.61x -- the shipped
point's own numbers, to the digit.

### Refine on a 2n-1 grid, not an arbitrary one

Growing the mesh exposed a property of this trajectory worth recording: **the flight torque
profile oscillates at the knot scale**, swinging ~13 N.m between adjacent knots (u at flight
knots 36/37: 11.8 then 24.7). The states are smooth and pass the integration check; the input
is not smooth. So resampling the input onto a grid that does not contain the old breakpoints
distorts it badly, and the distortion is worst at the start and end of flight where the tuck
ramp moves the legs fastest.

A bisected grid (`n_new == 2n-1`) contains every old knot, so the first-order hold through the
resampled samples **is** the source's force, exactly. The difference is not subtle:

| refinement | grid contains source knots | guess violation | flight integration drift |
|---|---|---|---|
| 50 -> 76 | no | 587 | 6.6e-1 |
| 50 -> 99 | yes (bisection) | 18.0 | 7.0e-2 |

Per-interval re-shooting (restarting the shot at each source knot instead of shooting across
the phase) is in for correctness -- it pins every original knot to its solved state -- but it
is worth noting it did *not* move the number on its own (17.4 -> 18.0). The residual at 99 is
the torque-speed halfplanes (2.38 N.m over) and the collocation defects on alternating
segments at each end of flight, i.e. it is all input-resampling error, not state error.

Nullity at 50 knots is **16 / 5756 rows**, in line with the 15-16 recorded before, so none of
this introduced an LICQ degeneracy.

### TUCK_RAMP was a fraction of knots; it needed to be a fraction of intervals

Refining exposed an off-by-one that only shows up when the mesh changes. `TUCK_RAMP` was
`ceil(0.23 * n_knots)`, but what has to stay fixed as the mesh changes is the *span of the
phase* the hard-tuck window covers, and a ramp of `r` knots spans `r/(n-1)` of the phase, not
`r/n`. Bisecting 50 -> 99 gave 23, a window of 0.2347..0.7653 against the source's
0.2449..0.7551 -- so the first and last hard-tucked knots landed half an interval outside where
the source solution had actually ramped to, and those two knots produced the two largest
collocation defects in the entire guess (18.02 at segments 74/75, 9.90 at 22/23, each pair
straddling exactly one of them).

`round(TUCK_FRAC * (n - 1))` with `TUCK_FRAC = 12/49` gives 24 at 99 knots -- which is 2*12,
the bisection of the source window, exactly -- and reproduces every value the knot form ever
produced: 6 at 26, 12 at 50, 18 at 76. It cannot affect `nullity_check`: both uses of
`TUCK_RAMP` are a bounding box and a cost, and nullity only examines exact equalities.

Effect on the 50 -> 99 guess, with nothing else changed:

| | violation | flight L drift | flight integration drift | L/R symmetry |
|---|---|---|---|---|
| knot-based ramp (23) | 18.02 | 1.76e-2 | 7.0e-2 | 7.24e-2 rad |
| interval-based ramp (24) | 10.77 | 6.15e-3 | 1.6e-2 | 2.06e-3 rad |

Also fixed: `_shoot` clipped the source-interval index to `len(ts)-2`, so the new grid's final
time -- which lands exactly on the last source knot -- was shot across the whole last interval
instead of being pinned, leaving that one knot 4.3e-2 off while all 49 others were exact. It is
the knot the flight->absorb stitching reads. Pinning it does not change the violation (the
4.3e-2 is the source's own end-of-flight drift: integrating from the second-to-last knot lands
that far from the solved last knot either way), but the guess is now exactly the source
solution wherever the two meshes coincide, which is the property that makes the round trip
checkable at all.

Knot-to-knot consistency of the 99-knot guess across flight, integrating each knot forward to
the next through the guess's own force: **median 5.4e-5**, max 4.3e-2 at that final segment.

## 2026-09-04 (later): 50 flight knots — 10/11, the best trajectory this project has produced

Refining the flight mesh works. `out/backflip.npz` is now a **50-flight-knot** solve, violation
**0.1287**, **10 of 11 audit checks passing**, with the one failure — flight angular momentum —
**1.61x** over its threshold. Reproduce with `--from-checkpoint
out/checkpoints_backflip_k50/best_10of11.npy`.

The collocation-vs-integrator check, which has failed in every session this file records,
**passes**: worst in-phase drift 3.44e-03 against a 5e-3 bound, with flight itself at 3.2e-03,
down from 1.8e-02 at 26 knots.

| check | 26 knots (`burst_38`) | 50 knots (shipped) |
|---|---|---|
| flight angular momentum | 3.44e-3 (3.44x) | **1.61e-3 (1.61x)** |
| collocation vs integrator | 1.81e-2 (3.62x) | **3.44e-3 (PASSES)** |
| flight CoM ballistic | 4.27e-05 | **1.43e-05** |
| max rotation per interval | 14.1 deg | **9.1 deg** |
| `max_violation` | 0.4259 | **0.1287** |
| audit | 9/11, worst 3.62x | **10/11, worst 1.61x** |

Better on every line, `max_violation` included — the first time in this file that ranking on the
audit and ranking on the violation have agreed.

### `TUCK_RAMP` was a fixed knot count, and that is why 50 knots stalled at first

`guess.py` ramps the tuck over the first and last 20% of flight *time*, and its comment requires
`TUCK_RAMP` to be at least that as a fraction of flight *knots*. `TUCK_RAMP` was the literal
integer **6** — which is 0.23 at the 26 knots it was written for, but 0.17 at 36 and **0.12** at
50. Above 26 knots the guess therefore starts outside the hard tuck window at exactly the knots
that window constrains.

The symptom was unmistakable once looked for: the first 50-knot attempt sat at `inf_pr` 2.05e2
for 45 iterations with step sizes of ~1e-5, while the 36-knot run next to it was moving. It is
now `ceil(0.23 * PHASES[FLIGHT].n_knots)`, which reproduces 6 exactly at 26 knots and gives 12
at 50. **Any future mesh change must keep this a fraction.**

### Three meshes do not give a convergence order — they are three different solutions

The plan was to run 36 and 50 together and read the observed order off the pair, to decide
whether the wall was mesh resolution or the quaternion representation. That does not work, and
the numbers say why:

| flight knots | intervals | flight drift | angular momentum | audit |
|---|---|---|---|---|
| 26 | 25 | 1.8e-2 | 3.44e-3 | 9/11, 3.62x |
| 36 | 35 | **1.9e-2** | 1.41e-3 | 9/11, 3.78x |
| 50 | 49 | **3.2e-3** | 1.61e-3 | 10/11, 1.61x |

Non-monotonic in *both* error columns: 36 knots is no better than 26 on drift, and it is better
than 50 on angular momentum. Each run is a cold start that lands in its own local solution, so
the difference between two rows mixes the mesh effect with the basin effect and cannot separate
them. A real order study has to refine the mesh *around a single trajectory* — interpolate a
converged solution onto a finer mesh and re-converge it — not solve three independent problems.
Do not quote an order from this table.

### Cost, and the LICQ check at the new size

6566 decision variables against 4934 at 26 knots (+33%), 3954 constraints. Each restart burst of
300 iterations takes ~10 minutes rather than ~3. `nullity_check.py` reports **16** at 50 knots
against 13 at 26 — the same `_add_boundary` / `_add_stitching` families, no new degeneracy class,
and IPOPT never raised `TOO_FEW_DOF`.

### The trajectory is now tracked in the repo

`traj_opt/out/` is scratch and gitignored, which was fine while the numbers were the deliverable
and wrong now that a trajectory is. A solve is not bit-reproducible across machines anyway —
IPOPT's linear algebra is threaded and hardware-dependent — so the best result has to live in the
repo as an artifact rather than as a recipe.

`traj_opt/reference/` holds `backflip.npz` (500 Hz, MuJoCo convention), `backflip.npy` (the
decision-variable vector that reproduces it) and `manifest.json` (mesh, commit, violation, and
every check's margin). Together ~330 KB. Promote with `uv run tools/ship.py <checkpoint>`, which
re-audits the point and **refuses a regression** on the same `(checks failed, worst overrun)` key
`restart_loop` ranks by — verified: a perturbed vector auditing 2/11 is rejected and the shipped
files are left untouched. `--force` plus a `--note` overrides, and the note goes in the manifest.

Downstream stages should read `traj_opt/reference/backflip.npz`. `traj_opt/out/backflip.npz`
stays a working file that any solve overwrites.

### What is left

One check. Flight angular momentum, 1.61e-3 against a 1e-3 bound. In flight there is no contact
and joint torques are internal forces, so this quantity is conserved *exactly* as a matter of
physics — the residual is pure transcription error and nothing else. Options, in the order I
would try them:

- **More flight knots still.** Cheapest to try, and the only lever with direct evidence behind
  it. But the table above shows the per-run scatter is comparable to the gap being closed, so a
  single 64- or 76-knot run will not be conclusive on its own.
- **A proper mesh-refinement study around the shipped solution**, per the section above. This is
  what would actually tell us whether more knots converges or asymptotes.
- **The quaternion representation.** `DirectCollocation` cubic-interpolates the state as a flat
  vector, including the four quaternion components, which is not interpolation on SO(3). At 50
  knots the base still turns 9.1 deg per interval. If refinement asymptotes, this is why.

## 2026-09-04 session: scoring on the audit found a better trajectory in checkpoints we already had

The 2026-09-03 session ended on the finding that `max_violation` is the wrong objective: a
point at 0.0004 (124x below the shipped 0.0495) audited 7/11 against the shipped point's 9/11.
The next step it wrote down was to change `restart_loop`'s selection rule to score bursts on
`audit.run` instead. That is done, and it paid off immediately — not from new solving, but
from re-ranking checkpoints the old rule had already discarded.

### The rule

`audit.report` now carries a **margin** — measured/threshold, so a check passes iff margin < 1
— and `restart_loop` ranks bursts on **`(checks failed, worst overrun, violation)`**,
lexicographic. `best_viol.npy` still keeps the lowest-violation point separately so runs stay
comparable with every number quoted in the sections below.

Deliberately not a weighted sum. A scalar over these eleven checks would be adding radians to
metres to N.m.s to kg.m^2: it ranks perfectly well, but the number it produces measures
nothing, and a headline figure nobody can interpret is worse than two that can. "9/11, worst
3.6x over threshold" says what is wrong and how badly. (A `score()` scalar was tried first and
removed the same day for exactly this reason; the ordering of every candidate below is
unchanged by the swap.)

The chain still runs on the raw result of every burst regardless of rank. Selection and
exploration are separate; filtering what gets chained would collapse the search back to a
deterministic fixed point (the same reason reverting to the best-seen point never worked).

### What the two rules pick out of the same 43 checkpoints

Re-ranking `out/checkpoints_backflip_clr_b/` — the run that produced the shipped trajectory:

| by violation (old rule) | viol | audit | | by audit (new rule) | viol | audit |
|---|---|---|---|---|---|---|
| `best_0495` | 0.0495 | 9/11, worst 9.80x | | `burst_38` | 0.4259 | **9/11, worst 3.62x** |
| `burst_14` | 0.0776 | 8/11, worst 121.70x | | `burst_24` | 0.1135 | 9/11, worst 5.30x |
| `burst_18` | 0.0812 | 7/11, worst 104.60x | | `burst_29` | 2.7513 | 9/11, worst 7.86x |

The two orderings share nothing below the top. And the old rule's *second* choice — 0.0776,
the next-lowest violation in the whole run — fails three checks with one of them **121x** over
its threshold, against 9.80x on a single check for the point it did keep. Below about 0.1, `max_violation` carries almost no information about whether the
trajectory is physically sound.

### `burst_38` beats the shipped trajectory on every physics measure

|  | shipped `best_0495` | `burst_38` |
|---|---|---|
| `max_violation` | **0.0495** | 0.4259 |
| flight angular-momentum drift | 7.13e-3 | **3.44e-3** |
| in-phase integration drift (q) | 4.90e-2 | **1.81e-2** |
| flight CoM ballistic residual | 3.71e-4 | **4.27e-5** |
| worst pitch reversal | 0.0489 deg | **0.0000 deg** |
| worst check, over threshold | 9.8x | **3.6x** |

It is worse on exactly one number, the one the old rule ranked on. `burst_38` is now shipped
as `out/backflip.npz` and kept as `out/checkpoints_backflip_clr_b/best_audit_b38.npy`; the
0.0495 point stays at `best_0495.npy`.

MuJoCo open-loop replay agrees, for what an unstable open-loop tape is worth: `burst_38`
reaches its first MuJoCo contact at t=0.002 with 5.4e-05 of base error behind it, where the
0.0495 point is already in contact at t=0.0 — and the final joint error is 1.425 rad against
2.354.

### The endpoint-only integration check was hiding half the error

Checking this raised a fair objection: `burst_38` has **32** collocation segments over 1e-2 of
defect against the shipped point's **12**, yet a smaller drift. That is exactly what
mid-phase error cancelling over a phase would look like, and the check could not tell, because
it only ever compared the *end* of each phase.

`check_integration` now samples every knot and takes the worst. The cancellation turned out to
be in the shipped point, not the candidate:

| flight drift (q) | end-of-phase only | worst over knots |
|---|---|---|
| `best_0495` | 3.2e-2 | **4.9e-2** |
| `burst_38` | 1.7e-2 | 1.8e-2 |

So the shipped trajectory's flight drift was 56% larger than the audit had been reporting —
about 10x its 5e-3 threshold, not 6x — and `burst_38`'s is genuinely uniform along the phase.
Every earlier "worst end-of-phase drift" number in the sections below is a lower bound on what
this check now reports; do not compare them across the change.

### All eleven checks on the shipped trajectory

`burst_38`, violation 0.4259. Margin is measured/threshold, so a check passes below 1.0.

| # | check | measured | threshold | margin |
|---|---|---|---|---|
| 1 | quaternion stays unit | 3.94e-05 | 1e-4 | 0.39x |
| 2 | net rotation is one backflip | -359.9979 deg, reversal 0.00000 deg | 1e-3 rad | 0.37x |
| 3 | flight CoM ballistic | 4.27e-05 m | 1e-3 | 0.04x |
| **4** | **flight angular momentum conserved** | **3.44e-03** | 1e-3 | **3.44x** |
| 5 | friction cone | 8.63e-07 N | 1e-6 | 0.86x |
| 6 | torque-speed halfplanes | 0.00 N.m | 1e-6 | 0.00x |
| 7 | no geometry below floor | +0.08 mm (never penetrates) | 2 mm | 0.04x |
| 8 | non-foot clearance margin | 7.58 mm (base, absorb) | 5 mm | 0.49x |
| 9 | sagittal symmetry | 3.78e-04 rad | 1e-3 | 0.38x |
| 10 | flight genuinely tucked | I_yy 0.5053 kg.m^2 | 0.55 | 0.92x |
| **11** | **collocation vs tight integrator** | **1.81e-02 (q)** | 5e-3 | **3.62x** |

Nine of the eleven are comfortable; six are an order of magnitude or more inside their bound.

### Still true, and still the two open failures

Both surviving audit failures are the same two as before — flight angular momentum and
integration drift — and both still live almost entirely in flight. Check 11 breaks down per
phase as load 2.0e-04, launch 3.0e-03, **flight 1.8e-02**, absorb 4.7e-05: flight carries
~100x the drift of the phases either side of it, and the other three are already an order of
magnitude inside the threshold, so adding knots anywhere but flight buys nothing.

Check 4 is the cleaner argument of the two. In flight there is no contact, and joint torques
are internal forces, so angular momentum about the CoM is conserved **exactly** as a matter of
physics. Any drift there is not the solver getting the mechanics wrong — it is purely the cubic
failing to represent the true trajectory between collocation points. It is an unusually direct
measurement of transcription error, and the only things that reduce it are a finer flight mesh
or a higher-order transcription. Every candidate examined
has 100% of its violation in collocation defects; no physical constraint (friction cone,
torque envelope, floor, clearance) is violated by more than 1e-6 in any of them.

### A warm start was never actually warm

Found while watching the first audit-ranked search start. `main()` always ran a feasibility
pass and then a costed pass before `restart_loop`, including when seeded with
`--start-checkpoint`. Both are long continuous solves — the exact thing the burst structure
exists to avoid, and the thing this problem is documented (2026-08-22) to handle badly.

Seeded with `burst_38` (worst check **3.6x** over), those two passes handed the restart loop
a point whose worst check was **12.6x** over. The search then spent its first bursts climbing back to where it had started. The
first attempt showed the same thing on the violation: IPOPT took the seed's 0.4259 into
restoration and was at 3.13 when the run was killed.

`--no-prepass` adds the cost and enters the loop on the seed itself, through the existing
bit-exact `result_from_vector`. Measured: `restart loop starting from viol=0.4259 audit=9/11
worst 3.62x over`.

It is deliberately **not** the default. The unseeded path has to run those passes — there is
nothing to preserve when the start point is `guess.py` — and every result in this file came
through them, so changing the default would silently change what a bare `--restarts` run
means. Use it whenever `--start-checkpoint` is given.

Two smaller fixes went in alongside, both found by exercising the new code rather than by any
search: `rank()` guarded the audit but not `max_violation`, which sits beside it and throws on
the same degeneracies (Drake refuses an all-zero quaternion), so one bad burst could kill a
40-burst run; and Python was block-buffering stdout to the redirected log, so the per-burst
rankings arrived in 8 KB clumps minutes behind the solve.

### The restart search is not the lever, and reseeding it is not a second sample

Two negative results, both measured, both costing a couple of hours of core time.

**Restarts do not close this gap.** `flip-m1` (26 knots, seeded from `burst_38`) found its best
at **burst 1** — 15.645 — and seventeen further bursts produced nothing better (closest 15.972,
the rest far worse). Its two failing checks sit at 2.0x and 4.6x their thresholds; the seed's
were 3.4x and 3.6x. A search that trades one failing check against the other, by tens of
percent, over forty bursts, is not the mechanism that closes a 3.6x gap.

And there is a sharper reason it cannot be. **In flight, angular momentum about the CoM is
conserved exactly** — joint torques are internal forces and cannot change it. So that check's
drift is not physics the solver got wrong; it is a direct measurement of transcription error,
of how badly the cubic represents the flight dynamics. No amount of searching within a fixed
mesh reduces it. Only a finer flight mesh or a better transcription does.

**Seeding a new run from another run's checkpoint is not an independent sample.** `flip-m2`
was launched from `flip-m1`'s best on the theory that multithreaded `spral` would make the two
chains diverge. It reproduced m1's chain bit-for-bit, offset by two bursts:

```
m1 bursts 2-10 : 52.255 48.146 39.806 23.666 30.801 15.972 28.420 771.143 49.122
m2 bursts 0-8  : 52.255 48.146 39.806 23.666 30.801 15.972 28.420 771.143 49.122
```

IPOPT is fully deterministic in this configuration. The "the search is stochastic in practice"
note used to justify side-by-side runs (see `checkpoint_dir`'s docstring) **does not hold** —
a concurrent run with the same seed and options is a duplicate, not a sample. To get a second
sample something must differ: `--burst-iters`, a tolerance, `--cost-scale`.

`m1`'s best (viol 0.1960) is kept at `out/checkpoints_backflip_m1/best.npy` but is **not**
shipped: its angular-momentum drift is better than `burst_38`'s (2.03e-3 vs 3.44e-3) and its
integration drift is worse (2.31e-2 vs 1.81e-2). On the ranking that matters its worst check is
4.61x over against `burst_38`'s 3.62x, so it loses — where the scalar had called it a 2.5%
win. That reversal is the clearest argument against the scalar.

**Last session's 34-knot checkpoints cannot seed anything.** `checkpoints_backflip_h1` has the
right variable count (5478) but evaluates at violations of 445-594 under the current
formulation — it was produced under different bounds *and* a different `h_max`, so the vectors
do not mean the same thing. A 34-knot run has to be cold-started.

### Next

- **Finish the two searches now running** (`flip-m1`, 26 knots seeded from `burst_38`;
  `flip-m34`, 34 knots from the analytic guess). Early: m1's best came at burst 1,
  beating its seed — the first improvement the rule has *driven* rather than found in
  checkpoints already on disk. m34 is at 8-of-11 after two bursts, but it is cold-started
  against m1's warm start, so the two are not yet comparable.
- **Re-run the seeded search with `--no-prepass`.** `flip-m1` was launched before that flag
  existed and threw away its warm start; whatever it reaches is a floor, not a fair number.
- **Then judge 34 knots.** It halved both failing checks at a fixed violation level
  (2026-09-03); it only hurt because the search was free to run past the good point on a
  criterion that could not see the damage. That criterion is now gone.

## 2026-09-03 session: the restart loop was throwing away every point it saved

Goal for the session: drive the remaining constraint violations to zero. The violation went
from 0.0376 to 0.0004 -- 94x -- and **the trajectory got worse**, which is the finding the rest
of this entry exists to explain. Two real bugs in how IPOPT was being driven were fixed along
the way and those stand on their own. `out/backflip.npz` is unchanged.

**Read "THE RESULT THAT MATTERS" first.**

### Where the violation actually is: all of it is collocation defect

Grouping every binding at the shipped point (`best_0495.npy`) by constraint family and taking
the worst residual in each:

| family | worst residual |
|---|---|
| collocation defects | **3.05e-02** |
| coupling (`input == B u + sum J^T lambda`) | 6.41e-04 |
| impact (impulsive touchdown) | 1.50e-04 |
| no-slip (stance foot vertical velocity) | 1.85e-04 |
| foot pins / floor clearance / friction cone / torque envelope | <= 6.8e-05 |

**Every physical constraint is already satisfied to better than 1e-3.** The 0.0495 headline
number is entirely the transcription's own defects, which is why the two failing audit checks
are both flight-side: they are not separate problems, they are that one number seen twice.

### The mesh was never the problem -- 26 knots was the wrong thing to blame

Per flight segment, the defect residual next to what a tight-tolerance integrator does over
that same single interval:

| flight segment | defect (v) | one-interval integration error (v) | (q) |
|---|---|---|---|
| 0 | 3.43e-02 | 6.14e-02 | 2.14e-03 |
| 5 | 3.05e-02 | 1.31e-02 | 7.39e-04 |
| **16** | **5.60e-04** | **9.93e-05** | **2.97e-06** |
| 24 | 3.09e-03 | 4.10e-02 | 2.09e-03 |

The local error tracks the defect residual roughly 1:1, and segments 15-17 -- the ones that
happen to be converged -- integrate to 1e-5 in q. That is what Hermite-Simpson truncation
actually costs at h = 24 ms here, and it is ~1000x below the audit's 5e-3 threshold. So the
2026-09-01 negative result on 26 -> 32 knots was right to be reverted but was read backwards:
growing the mesh failed because the mesh was never short, it just made a harder NLP. **Knot
count is not a lever on either remaining audit failure. Convergence is the only lever.**

### Bug 4: IPOPT's default `bound_push` was destroying the start of every restart burst

`restart_loop` chains forward by `SetInitialGuess(x)` and re-solving. IPOPT moves any variable
sitting within `bound_push` of a bound into the interior *before iteration 0*, and at the
default 0.01 that move is not small here:

| `bound_push` | viol at iteration 0 | variables moved | max abs move |
|---|---|---|---|
| **1e-2 (default)** | **1.68e+01** | 282 | 4.54e-01 |
| 1e-3 | 1.67e+00 | 210 | 4.54e-02 |
| 1e-4 | 1.57e-01 | 192 | 4.52e-03 |
| 1e-6 | 4.95e-02 | 59 | 1.88e-05 |
| 1e-8 | **4.9465e-02 (exact)** | 0 | 0 |

The damage is linear in the setting, ~1675x it. 282 variables sit on bounds at any good point
here -- launch torques against their +-45 N.m limit above all -- so **every burst in every
restart search this project has run re-entered IPOPT from a point ~340x worse than the one it
had just saved to disk.** That is almost certainly what "IPOPT reliably wanders away from good
points once it finds them" always was. Fixed in `ipopt_options`: `bound_push`, `bound_frac`,
`slack_bound_push`, `slack_bound_frac` all 1e-8.

Fixing it alone does not converge the problem (a 300-iteration feasibility pass from the
shipped point still drifts 0.0495 -> 0.1293), because it trades one problem for another: at
1e-8 from a bound the barrier multipliers `mu/(x - x_L)` are ~1e7, and IPOPT reports dual
infeasibility in the thousands *on a zero-objective problem*. The complete warm start needs a
small `mu` as well, which leads directly to:

### Bug 5: Drake defaults IPOPT to `mu_strategy = adaptive`, silently disabling `mu_init`

Three chains launched with different barrier options produced **bit-identical iterates**, which
looked at first like Drake dropping the options. It is not dropping them: under `adaptive`,
`mu_init` is unused and re-setting `adaptive` is a no-op, so both null results have the same
one cause. Forcing `mu_strategy=monotone` makes `mu_init` take effect immediately and visibly
(`lg(mu)` reads -6.0 at iteration 1 for `mu_init=1e-6`). Worth knowing before spending another
run on a barrier option: **on this Drake build `mu_init` does nothing unless `monotone` is set
alongside it.** `--ipopt-opt KEY=VALUE` (repeatable) now exists on `solve_backflip.py` so a
barrier setting can be tested without editing the file.

### The active set at a solved point is rank-deficient -- nullity 202, cond 5e20

`nullity_check.py` has only ever tested the *initial guess*, and only *exact equalities*. That
is the right check for "will IPOPT refuse to start" and it cannot see what stalls the solve:
once the optimizer drives the iterate onto a TIGHT box, that box is an active inequality and is
back in the KKT system, rank-deficient against the defects that already imply it. Measured at
`best_0495`, stacking equalities plus genuinely-active inequalities:

- **nullity 202 of 4672 rows**, smallest singular values at 1e-16, cond(A) = 5.5e20
- 83 of the null-space energy on the left/right thigh/calf mirror (97 active rows)
- 25.2 on the `q[1]/q[3]/q[5]` TIGHT box, 9.3 on the hip pins, 14.8 on the quaternion-norm box

This is what IPOPT's behaviour looks like from the inside: Newton steps of norm 6e5 cut back to
`alpha ~ 1e-9`, and dual infeasibility of 1e3-1e8 on a problem whose objective is identically
zero.

### Two structural changes, both measured and both ultimately rejected

**Rejected: pinning the sagittally-zero DOFs as fixed variables.** `q[1]`, `q[3]`, `q[5]` and
the four hips are exactly zero for a sagittal motion, so `AddBoundingBoxConstraint(0, 0, .)`
looks strictly better than a TIGHT box -- IPOPT eliminates a fixed variable outright (it is
already doing this for 154 of them, which is why it reports 4780 variables for a 4934-variable
program). It is much worse. Removing the positions as unknowns leaves their own collocation
defect rows over-determined in velocity alone, and the guess-level equality nullity goes
**13 -> 224**. Reverted. Do not re-try this.

**Tried, then reverted: the left/right mirror as a safety net rather than a pin.** `MIRROR`
was set to 1e-2. It is back at `TIGHT` -- see "THE RESULT THAT MATTERS" above; the counting
argument below is sound and the violation improved a lot, but the trajectory got worse. The count is the argument: linearised about a symmetric point the
problem splits into symmetric and antisymmetric halves, and per joint pair per phase the
antisymmetric half carries 12 mirror rows against 22 defect rows on 24 unknowns -- over-
determined by 10, which is exactly the observed structure. "Nothing is lost, because symmetry is already structural" was the prediction --
symmetric HOME start, exact per-knot torque mirror, exactly mirrored contact forces and
impulses, no asymmetric term in the mechanism -- and it was **wrong**: the solve parks 9.9e-3
of hip splay on whatever bound it is given. A prediction about what the dynamics will carry is
not a mechanism. (The old bound was not holding exactly either -- the shipped trajectory's
worst L/R mismatch is 2.73e-04 rad against a 1e-4 box -- but 2.7e-04 is not 9.9e-3.)

`nullity_check.py` reports **13** with the mirror change in place (unchanged from the same
diagnostic on the pre-change program, so this adds no LICQ risk at the guess).

### Diagnostics fixed along the way

- **`nullity_check.py` now eliminates fixed variables the way IPOPT does** -- zero their
  columns, drop the rows left empty -- and reports how many. Without this it counts rows IPOPT
  never sees; the first version of the rejected change above read 312 instead of 12. It also
  means the historic "15/4424" and today's "13/4268" are different accountings, not a change in
  the program.
- **`audit.py` gains a tenth check: sagittal symmetry**, measured on all four legs plus the
  zeroed DOFs, and required to hold at a tenth of `MIRROR`. Loosening a bound without measuring
  what it was protecting is how the grazing knee got shipped; this is that lesson applied in
  advance. Passes at 2.73e-04 rad on the shipped trajectory.
- **`--proximal W`** adds `W*||x - seed||^2` (normalised per variable). A pure feasibility pass
  leaves the objective flat across the ~200-dimensional degenerate subspace, which gives the
  limited-memory quasi-Newton approximation nothing to resolve it with; a proximal term makes
  the reduced Hessian the identity there. On its own, with adaptive mu, it does not hold the
  point (two chains wandered to inf_pr 0.21 and 0.35); paired with `monotone` + small `mu_init`
  it holds ~57 iterations before IPOPT drops into its restoration phase.
- **Launch long runs as systemd user services**, not `setsid nohup`: `systemd-run --user
  --collect --unit=flip-x -p StandardOutput=file:... `. `setsid` was *still* losing runs to
  session teardown (that has now cost this project five searches); a transient unit does not
  die with the session and `systemctl --user list-units 'flip-*'` shows what is alive.

### THE RESULT THAT MATTERS: a 124x lower violation audits WORSE

Everything below was chasing the wrong number, and the run that finally proved it is the most
useful thing this session produced.

With the degeneracy reduced and `bound_push` fixed, the searches got dramatically better at the
thing they were being scored on. The 400-iteration feasibility pass from the analytic guess
reached **viol 0.0025** against an all-time best of 0.0376 that used to take 40 restarts and
hours; growing flight to 34 knots then let a restart chain reach **viol 0.0004**, which is 94x
below that all-time best and 124x below the shipped 0.0495.

That 0.0004 point audits **7/11**. The shipped 0.0495 point audits **9/11**.

| audit check | shipped, viol 0.0495 | "best", viol 0.0004 |
|---|---|---|
| net rotation | PASS | **FAIL** -- 0.131 deg pitch reversal |
| flight angular momentum | FAIL 7.13e-03 | FAIL 6.35e-03 |
| sagittal symmetry | PASS 2.7e-04 | **FAIL** 1.0e-02 |
| collocation vs integrator (flight) | FAIL 3.2e-02 | **FAIL 1.66e-01** |
| | **9/11** | **7/11** |

The integration drift is 5x WORSE at 80x lower violation, and the pitch now reverses. That
combination has one explanation: **the solve is finding a spurious discrete solution.** The
defects are satisfied almost exactly AT the collocation points while the cubic rings between
them -- which is precisely what Hermite-Simpson admits and precisely what `check_integration`
exists to catch. More knots gave the ringing more room, and a looser feasible set let the
optimizer reach it.

**So `max_violation` is not the objective for this project, and optimising it hard is actively
dangerous.** The audit is the scoreboard. Two of its checks -- integration drift and the
"without being pinned" pair -- are the only things standing between a low violation number and
a trajectory that is not physical. Everything a future session does to the formulation has to
be scored on the audit, on a checkpoint, before it is believed.

### What was reverted, and why each thing looked right first

All of these lowered the violation and none survived the audit:

| change | violation | audit |
|---|---|---|
| `MIRROR` / `SAGITTAL_BOX` 1e-4 -> 1e-2 | 400-iter pass ~69 -> **0.0025** | symmetry FAILS: solve parks 9.9e-3 of hip splay on the bound |
| stance `h_max` +40% | active-set nullity 64 -> 38 | launch integration drift 1.8e-3 -> **1.8e-2** |
| flight 26 -> 34 knots | 0.0025 -> 0.0028, enabled the 0.0004 chain | enabled the spurious point above |
| `QUAT_BOX` 1e-4 -> 1e-2 | neutral | neutral (measured 8.5e-6..5e-5) -- reverted for consistency |

The sagittal bound sweep, all at 400 iterations with everything else held, is worth keeping
because it shows how strong the pull toward the wrong answer was:

| `SAGITTAL_BOX` / `MIRROR` | feasibility pass |
|---|---|
| 1e-4 (TIGHT, shipped) | stuck at ~69 |
| 3e-4 | 0.0541 |
| 1e-3 | 0.1137 |
| 1e-2 | **0.0025** |

And no cost recovers the symmetry a loose box gives away. `SYM_COST` at 10 leaves 9.7e-3 of
wander, at 1000 it leaves 7.2e-3, and the objective RISES during the pass as feasibility
improves -- so the lateral DOFs are not the free null direction they look like, they are slack
the solver spends absorbing defect residual. Named, the offender is a hip (0.4-0.6 deg of
splay), not the base or the quaternion. Two-stage (solve loose, re-impose TIGHT warm-started
from the loose point) does not rescue it either: from 0.0164 the tight pins go to 0.19-3.06.

### Mesh: flight refinement is available again, stance refinement is not

Recorded because the 2026-09-01 note said the opposite and was reasoning from the old
formulation. Growing flight no longer blows up -- 34 knots solves to 0.0028 where 32 used to
stall at 2.13 -- and on a checkpoint at comparable violation it does what the per-segment
measurement predicted:

| check (both at viol ~0.02) | flight 26 | flight 34 |
|---|---|---|
| flight angular momentum drift | 1.26e-02 | **1.25e-03** |
| flight integration drift, q | 4.7e-02 | **9.4e-03** |

So flight refinement is a real lever on the two failing checks **at a fixed violation level** --
it is only when the search is then allowed to chase the violation down to 0.0004 that it turns
into ringing. A future attempt should grow flight AND stop the search early on the audit, not
on `max_violation`.

Stance refinement is the opposite and should not be retried: launch 12 -> 16/18/20 knots gives
0.22 / 9.39 / 0.28 against 0.0028. Every extra flight knot adds a defect row and nothing else,
while every extra stance knot also adds a foot pin, a no-slip row, a friction cone and a
coupling constraint. 40 flight knots is also too many (0.628).

### And any objective at all costs feasibility

The costed pass at the weights that have always been used walks straight off the manifold: it
takes 0.0025 to **0.7349** at cost 11.5, and the restart loop then starts 300x worse than the
point it was handed. `--cost-scale` now scales the four performance terms; at `--cost-scale 0`
the pass keeps feasibility. None of the four is needed for a valid trajectory -- the tuck is
guaranteed by the hard `FLIGHT_TUCK` window, not by `w_tuck` -- so they are affordable only
once feasibility is banked. A proximal term (`--proximal`) has the same problem for the same
reason and is kept only as a diagnostic.

### Where this leaves it

`out/backflip.npz` is **unchanged**: viol 0.0495, **9/11**, still the best trajectory the
project has. The formulation is back where it started apart from the two solver bugs, the
no-slip de-duplication, and much better diagnostics. What genuinely improved is the
understanding of what to optimise, and the tooling that can tell.

Next, in order:

1. **Score on the audit, not on `max_violation`.** `restart_loop` keeps the lowest-violation
   burst; on the evidence above that is the wrong selection rule. It should run `audit.run` on
   each burst and keep the best-auditing one, or at minimum reject any burst whose integration
   drift got worse.
2. **Flight 34 knots, with that selection rule.** It halves both failing checks at a fixed
   violation level; it only hurt because the search was free to run past the good point.
3. The two solver findings below (`bound_push`, `mu_strategy`) are unconditional and stay.

## 2026-09-01 session B: clearing the floor by a margin, not by a hair

User replayed the shipped trajectory and reported it looked much better, but that the head
came close to collision on landing and the rear knees collided "just a little". Both were
real, both were the same bug, and the bug was in a **bound**, not in the solver.

### The defect: `>= 0` is not "does not collide"

`_add_body_clearance` required every one of the 27 witness spheres to satisfy `p_z(q) >= 0`,
and its docstring defended the zero: a stance foot's witness sphere IS the contact, so any
positive margin there would contradict the foot pin. That reasoning is correct for the foot
and wrong for the other 25 spheres, which then had no reason to keep any gap — and the
optimizer spent every millimetre of gap it was allowed to, exactly where the user saw it:

| | where | when | clearance |
|---|---|---|---|
| rear knee | `RL/RR_calf_upper` | t = 0.30–0.31, launch | **−0.2 mm**, three knots running |
| head | `head_sphere` | t = 1.078, touchdown | **+0.1 mm**, *before* the feet load |

The knee rides the floor while the base pitches back through −40°; the head gets to the
ground before the feet do. Neither is solver error — both figures are stable across
checkpoints of very different violation, which is the same tell that identified the no-slip
defect earlier: **a number that ignores solver accuracy is a missing constraint.**

### The fix

`BODY_CLEARANCE = 0.010` in `program.py`. The bound is now per-sphere: exactly 0 for the two
foot spheres (they are the contact, and their swing clearance is `FOOT_CLEARANCE` already),
10 mm for the other 25. The exemption is found **by geometry** (matching `K.P_ANKLE` and
`K.R_FOOT`), not by row index, so regenerating `COLLISION_SPHERES` with
`tools/clearance_points.py` cannot silently move it.

10 mm is ~50× the between-knot reconstruction error (0.05 mm) and ~4× the worst knot
violation IPOPT leaves at the tolerance it converges to here, so it survives both — and it
is margin the hardware stage needs against tracking error anyway.

`nullity_check.py` is **unchanged at 15/4424** (verified against a stashed baseline, not
assumed): only an inequality bound moved, so this adds no LICQ risk.

### The matching audit blind spot, also fixed

The audit could only see penetration *below zero*, so "grazes at 0.1 mm" passed — the same
class of blindness that let the stale file report 4/6 while the head was 72 mm underground.
Both checkers now report the closest **non-foot** approach as a first-class number:
`audit.py` gains a ninth check, and `check_npz.py` reports it for the replayed file, where it
flags the old trajectory `TOO CLOSE  −0.2 mm (RL_calf_upper)`. Without this, a later rerun
could regress the clearance and still report a clean audit.

### Result: 0.0495 at 7/9 — the clearance is free

Four searches were run on the tightened problem (all `is_success()==False`, as always here):

| chain | seed | budget | best viol | note |
|---|---|---|---|---|
| warm | `checkpoints/best.npy` (0.0376) | 12 × 300 | 0.0855 | chain blew up after restart 2 (viol 8–124) |
| A | analytic | 40 × 300 | 0.0590 | stopped by user at restart 30 |
| **B** | analytic | **40 × 200** | **0.0495** | **shipped** |
| C | analytic | 70 × 200 | — | stopped by user inside the feasibility pass |

**B vs the previous shipped best**, and the point of the whole exercise:

| audit check | previous (0.0376) | B (0.0495) |
|---|---|---|
| closest non-foot approach | **−0.2 mm** | **+9.7 mm** (+9.99 at knots, +9.72 between) |
| head at touchdown | +0.1 mm | clear — off the close-approach list entirely |
| net rotation | −359.9998° | −359.9996° |
| flight CoM ballistic | 4.98e-05 | 3.71e-04 (passes) |
| flight angular momentum | FAIL 2.24e-02 | FAIL **7.13e-03** — 3× better |
| friction cone | pass | pass |
| torque envelope | pass | pass |
| floor penetration | +0.06 / +0.24 mm | +0.08 / +0.06 mm |
| tuck (peak I_yy) | 0.5091 | **0.5091** — identical |
| collocation vs integrator | FAIL 2.9e-02 / 2.2e-01 | FAIL 3.2e-02 / 2.8e-01 |
| | 6/8 | **7/9** |

B fails only the same two checks the previous best fails, and beats it on angular momentum.
**The 10 mm margin cost essentially nothing dynamically** — the large degradation seen in the
first, half-budget runs (flight drift 2.7e-01, ballistic failing) was under-convergence, not
the margin fighting the physics. Worth remembering the next time a constraint "looks
expensive" after one short search.

### Two things worth carrying forward

1. **B found its best on restart 39 of 40**, having sat at 0.0776 since restart 14. The
   search was still improving when the budget ran out, so on this problem *budget*, not the
   formulation, was the binding constraint. Chain C (70 × 200, same recipe) exists to test
   that and was stopped before it produced anything — it is the obvious thing to rerun.
2. **Long runs keep dying to session teardown** (this has now cost three searches, including
   the one that made the stale file). Launch them detached:
   `setsid nohup env PYTHONUNBUFFERED=1 uv run traj_opt/solve_backflip.py ... &` — they
   reparent to `systemd --user` and survive the editor closing. `PYTHONUNBUFFERED=1` also
   makes `[restart N]` lines appear live instead of only at exit, which is why an earlier
   chain looked silent for hours while it was in fact fine.

## 2026-09-01 session: the geometry fix works — the shipped .npz was just stale

User replayed the trajectory and reported the robot barely tucks. It doesn't: **the file
being replayed (`out/backflip.npz`) is byte-identical to `out/backflip_prefloor.npz`**, the
pre-geometry-fix trajectory from 2026-08-23. The 2026-08-25 solve that would have replaced
it (`out/solve_geom.log`, now `solve_geom_partial1.log`) was killed after ~5 of 20 restarts
and never got as far as writing an `.npz`. Nothing regressed; the new formulation had simply
never produced an output file.

**New: `tools/check_npz.py`** measures the saved `.npz` directly in MuJoCo — exact lowest
point of every collision geom (box/sphere/capsule/cylinder), I_yy about the CoM, net base
pitch — so "does the thing I'm replaying tuck / clip the floor" is one command instead of an
inference from the solver's own residuals. It reproduces the 2026-08-25 table exactly on the
stale file (head −71.6 mm, rear thigh −68.6 mm, I_yy 0.668), which is what validates it.

**New: `solve_backflip.py --from-checkpoint PATH.npy [--out PATH.npz]`** wraps a restart-loop
checkpoint through the existing `result_from_vector()` and runs the whole
audit/resample/save pipeline on it, so a good point can be replayed and audited *while* the
run that produced it is still going. `--out` also picks the checkpoint directory
(`checkpoint_dir()`), which is what lets two independent restart searches run side by side.

**The killed run's partial best (viol 3.04 — barely feasible) already audits 6/8**, and that
is the real result of this session: the formulation change did what it was supposed to.

| check | stale `backflip.npz` (viol 0.0283) | geom partial best (viol 3.04) |
|---|---|---|
| net rotation | PASS | PASS (−359.9999°) |
| CoM ballistic | PASS | PASS (0.14 mm) |
| angular momentum | FAIL 0.76% | FAIL 2.0e-2 drift |
| friction cone | PASS | PASS |
| torque envelope | PASS | PASS |
| floor penetration | **−71.6 mm** | **+0.07 mm** |
| tuck (peak I_yy, held knots) | **0.668** | **0.508** |
| collocation vs integrator | FAIL | FAIL (0.06 q / 0.73 v) |

Kept as `out/backflip_geom_partial.npz` — replayable (`uv run traj_opt/replay.py --npz …`)
but **not** a validated trajectory at viol 3.

One genuinely new defect this exposed: the resampled 500 Hz output dips **4.2 mm** below the
floor at the rear foot around t = 0.21 s even though every *knot* is clean (audit: +0.07 mm).
Floor clearance binds at knots only, and the cubic reconstruction sags between them. Small,
and it will shrink as the solve tightens, but a per-witness-point margin on the non-stance
knots is the fix if it survives a converged solve.

### Then a second defect of the same family: the stance foot was never held still

`check_npz.py` on that same checkpoint showed the rear foot at −4.6 mm mid-interval while
every knot was clean (+0.08 mm). Same figure (−4.2 mm) on the viol=3.0 checkpoint: **a
number that does not move with the violation level is a missing constraint, not solver
inaccuracy.**

It was. `_add_contact` pinned the foot's *position* at knots and nothing else, so a stance
foot could sit exactly on the floor at every knot while moving through it — measured on the
solved checkpoint at **1.375 m/s** (front feet at the load→launch seam), 0.41 m/s at
takeoff, median 0.0025 elsewhere; the analytic guess does 0.58 m/s through the launch. The
Hermite cubic between two pinned knots then bulges by `h·|v|/4`, which is the ±4.4 mm.

Fixed by boxing the **vertical velocity of the foot sphere's centre** — exactly `d/dt` of
the pin already imposed — at every stance knot (`NO_SLIP = 1e-3` m/s, capping the bulge at
5 µm). Two things worth not re-deriving:

- **Vertical only, and the centre, not the material contact point.** The material point's
  velocity is the no-slip condition of a *rolling* sphere, whose contact patch travels along
  the floor; the foothold pin says it doesn't. Imposing both forces `omega_y = 0` at every
  stance knot — no calf pitch, i.e. no push-off. The first version of this constraint had
  exactly that bug. Vertically the two agree (`v_mat_z = c_z`), which is also why this is
  consistent with the touchdown impulse's own no-slip rows.
- **Boxed, not exact**, for the usual LICQ reason: it *is* the pin's derivative, so the two
  are dependent by construction. `nullity_check.py` unchanged at **15/4424**.

`audit.check_floor` now samples between knots as well as at them — a knots-only check can
only ever confirm what the solver already reported, which is how a 4.4 mm floor excursion
passed an audit written to catch floor excursions.

**Known and NOT fixed: the pin models a foot that spins in place rather than rolls.** With
the lowest point pinned to a fixed foothold, a pitching calf slides its material contact
point at `R·omega_y` (~0.7 m/s at takeoff) while the friction force is applied at that same
sliding point, with nothing tying the force's direction to the slip. The honest model is a
foothold that translates with the roll. That is a real formulation change (the foothold
stops being a constant), worth doing before hardware, not before a first converged
reference.

### Open question #1 answered: flight carries the integration drift, load/absorb do not

`audit.check_integration` now reports **per phase**, not just the worst across all four. On
the best corrected-formulation checkpoint (viol 0.0376):

| phase | q drift | v drift |
|---|---|---|
| load | 1.5e-04 | 1.1e-02 |
| launch | 3.1e-03 | 5.5e-02 |
| **flight** | **2.9e-02** | **2.2e-01** |
| absorb | 6.5e-04 | 6.8e-04 |

Flight carries roughly 10x any other phase. **The standing suspicion was backwards**: the
next-steps list ranked "extend single-shooting to `load`/`absorb`" highly because those two
are still on the plain kinematic guess and relatively coarse — they turn out to be the two
*cleanest* phases. Extending single-shooting to them is therefore not the lever it was
ranked as. Growing the flight mesh was the obvious follow-up — and it does not work either;
see the negative results below.

**Two independent 40-restart searches** on the corrected formulation (the search is
stochastic in practice, so a second concurrent run is a second sample, not a duplicate):

- A: `--burst-iters 300` → `out/backflip.npz`, `out/checkpoints/`, log `out/solve_noslip_a.log`
- B: `--burst-iters 200` → `out/backflip_b.npz`, `out/checkpoints_backflip_b/`, log
  `out/solve_noslip_b.log`

The pre-no-slip runs they replaced (best 0.146 / 0.412, both stalled with no improvement for
~2.5 h) are kept in `out/checkpoints_prenoslip_{a,b}/` with their logs, and are the reference
for whether the new constraint costs feasibility. Both searches write every burst to disk, so
a killed run still leaves its best point behind — check those directories with
`--from-checkpoint` before launching anything new.

### Result: 0.0376 at 26 knots, 6/8 — the best trajectory the project has produced

Run A completed all 40 restarts at **viol 0.0376** and wrote `out/backflip.npz`. Run B
completed at 0.0510 and never beat its own burst-2 point in the 38 bursts after it.

| audit check | pre-geometry best (0.0283) | now (0.0376) |
|---|---|---|
| net rotation | PASS | PASS (−359.9998°) |
| CoM ballistic | PASS 0.21 mm | PASS **0.05 mm** |
| angular momentum | FAIL 0.76% | FAIL **0.56%** |
| friction cone | PASS | PASS |
| torque envelope | PASS | PASS |
| floor penetration | **−71.6 mm** | **−0.06 mm** knots / **−0.24 mm** between |
| tuck (peak I_yy held) | **0.668** | **0.509** |
| collocation vs integrator | FAIL 0.037/0.34 | FAIL **0.029/0.219** |

Better on every metric than the old best, on a strictly *smaller* feasible set (floor
clearance + hard tuck + no-slip all added since). Confirmed independently on the written
`.npz` by `tools/check_npz.py`: worst penetration 0.2 mm, held I_yy 0.509.

### Three negative results, all worth not repeating

1. **`--start-checkpoint` continuation didn't help here.** A fresh 40-restart chain seeded
   from the 0.0376 point (different `--burst-iters`, so a different chain) ran to completion
   at **0.0688** and never beat its own seed. Consistent with the older finding that good
   points are transient under continued iteration — a *new* chain from a good point is no
   more likely to stay near it than any other chain.
2. **Flight 26→32 knots is worse, not better.** Despite the per-phase result above pointing
   at flight, growing it stalled at **2.13** after 26 bursts — ~50x worse than 26 knots at
   the same stage, with no improvement over the last hour of it. `h_max` was tightened
   0.032→0.026 in the same step so the new knots couldn't just bunch up; nullity was clean
   (16/4796). **Reverted to 26 knots**, which stays the shipped configuration.
3. **Warm-starting that 32-knot problem from the 0.0376 26-knot solution didn't rescue it**
   either: iteration-0 infeasibility 418 vs the analytic guess's 360, and it stalled at
   **2.35**. This was the one untested case the previous session flagged (warm-starting from
   a genuinely smooth, well-resolved source) — it now has an answer, and it is the same
   answer as every other mesh transition: `guess.py`'s single-shooting is the better start.

So the flight mesh is not simply under-resolved — 26 knots is where this transcription's
feasibility and its discretization error balance, and pushing either side of that loses.
The remaining two failures are small and shrinking, and the next lever is not knot count.

## 2026-08-25 session: the trajectory clips the floor and never tucks (IN PROGRESS)

User reviewed the solved trajectory visually and reported three things: the heels clip the
ground on launch, the robot barely tucks after takeoff, and the legs sit near full extension
for much of the flip. All three reproduce numerically, and all three come from the same root
cause: **the NLP's only geometric knowledge of the robot is four point-feet.** Nothing else
about its shape exists in the program, so nothing stops the rest of it from sweeping through
the floor, and nothing rewards folding up.

### Measured on the shipped `out/backflip.npz` (26-knot, viol 0.0283, "4/6 passing")

Lowest point of every collision geom, over the trajectory:

| geom | min z | when |
|---|---|---|
| `head_sphere` | **−71.6 mm** | t = 1.128 s (just after touchdown, still 25° from upright) |
| `RL/RR_thigh_col` | **−68.6 mm** | t = 0.280 s (mid-launch) |
| `RL/RR_calf_upper` | −65.8 mm | t = 0.280 s |
| `RL/RR` foot sphere | −27.6 mm | t = 0.282 s |

Flight inertia and what it costs:

| pose | I_yy (kg·m²) | ω = L/I | time for 4.36 rad | CoM rise needed |
|---|---|---|---|---|
| solved trajectory, mid-flight | **0.659** | 6.41 | 0.680 s | **0.567 m** |
| just standing (HOME) | 0.484 | 8.74 | 0.499 s | 0.305 m |
| `TUCK_LEGS` (2.2, −2.7) | 0.452 | 9.35 | 0.466 s | 0.267 m |

The solved flight pose is a **worse** rotational configuration than simply standing there.
Both knees ride the *straightest* edge of `TUCK_BOX` for essentially the whole flight
(`calf_rear` = −0.838 vs the box's −0.83776 bound; `calf_front` = −1.686 vs −1.685983), which
is the direct evidence for "legs near full extension": `TUCK_BOX` only asserts *not
self-colliding*, and near-full extension satisfies that. Nothing in the cost (torque² +
torque-rate² + time) rewards tucking — actively folding the legs *costs* torque — so the
optimizer paid for the 46% larger inertia with a 0.57 m CoM rise instead. **Tucking is the
single biggest feasibility lever left: it roughly halves the jump the launch has to produce.**

### Root causes, and a fourth bug found on the way

1. **No floor for anything but the feet.** `_add_clearance` only bounds swing-foot P_FOOT
   height and (flight only) base z. Load, launch and absorb have no torso/head constraint at
   all, which is why the head punches through on landing.
2. **The foot is a sphere, not a point.** `P_FOOT` is the bottom of the 22 mm foot sphere
   *only when the calf is vertical*, and the calf is never vertical — 51.6° at HOME, past 90°
   at launch. Pinning that body-fixed point to z = 0 buries the sphere by R·(1−cos tilt).
3. **`STAND_BASE_HEIGHT` inherits that error**: the "corrected" 0.2800479196045126 rests
   P_FOOT on the floor and therefore the foot *sphere* **8.3 mm under it** — so the reference
   starts and ends already penetrating. Correct value: **0.2883725003026**.
4. **The audit cannot see any of this.** It passed 4/6 while the robot was 69 mm through the
   floor. A floor-penetration check has to be added, or this class of error stays invisible.

### Landed this session

Commit 1 (`1c5fd3e`, diagnosis + inert groundwork):

- `tools/clearance_points.py` (new): generates sphere-swept witness points bounding every
  collision geom in `go2.xml`, grouped per body kind, and **verifies** them against MuJoCo's
  own geom poses over 4000 random sagittal poses (conservative to 1e-16 m). Same idiom as
  `tools/tuck_box.py`.
- `constants.py`: `R_FOOT`, `P_ANKLE`, `P_FOOT` re-derived from them (numerically unchanged),
  the generated `COLLISION_SPHERES` table, and a `FLIGHT_TUCK` window.

Commit 2 (`d9df54c`, the actual fix — all four defects):

- **Sphere feet.** `_Kin.pos()` returns the foot sphere's lowest point; `_Kin.jac()` takes the
  q-dependent *material* point at the contact, so the friction force no longer acts 22 mm above
  where it should (~2.5 N·m of spurious knee moment). `STAND_BASE_HEIGHT` corrected to
  0.2883725003026 in the same commit — verified: the guess now puts all four feet at exactly
  z = 0.
- **Floor clearance.** New `_add_body_clearance()`: `p_z(q) ≥ r` for 27 witness points at every
  knot of every phase (base + `FL_*` + `RL_*`; the right legs are redundant under the enforced
  mirror). Lower bound is exactly 0 — a stance foot's own witness sphere *is* the contact, so
  any positive margin would contradict the pin.
- **Real tuck.** `FLIGHT_TUCK` held on the middle flight knots (`TUCK_RAMP = 6` free at each
  end to fold in and extend out), strictly inside `TUCK_BOX` so they never conflict, plus a
  `w_tuck` cost pulling toward `TUCK_LEGS` so the solver settles inside the window instead of
  riding an edge.
- **Audit.** `check_floor` (all four legs, so the left-leg-only symmetry assumption cannot leak
  silently) and `check_tuck` (peak flight I_yy, fails above 0.55). 6 checks → 8.
- `guess.py` follows the same contact convention, and its flight `RAMP` dropped 0.3 → 0.2 to
  match `TUCK_RAMP` as a fraction of the flight knots.

Verified before solving: `nullity_check.py` = **15/4424**, unchanged from the documented 13 at
that tolerance — no new LICQ pathology. Program builds at 4934 vars / **2982** constraints
(+80 = 66 floor + 14 tuck). Guess violation 88.3, in the normal range for the plain analytic
guess at this size (83.1 before), with worst floor slack exactly 0.00 mm.

### Open

- A 20-restart solve on the new formulation is running (`out/solve_geom.log`). **The previous
  violation figures are not comparable across this change** — the feasible set is strictly
  smaller now, and the tuck should change the whole shape of the answer (a ~0.27 m CoM rise
  instead of ~0.57 m). Judge the result on the 8 audit checks, not on beating 0.0283.
- The pre-change artifacts are kept for comparison: `out/backflip_prefloor.npz` and
  `out/checkpoints_prefloor/`.
- Still open from before, and untouched here: flight angular-momentum drift (0.76%) and the
  collocation-vs-tight-integrator drift that plateaued at 26 knots (~0.34 rad/s in v). The
  suspects recorded last session — `load`/`absorb` still on the plain non-single-shot guess at
  12/16 knots — are unchanged.

## 2026-08-23 session: audit was half-wrong, and mesh refinement isn't a free lunch

**Re-audited the best checkpoint independently** (`traj_opt/out/checkpoints/best.npy`
from the production run left going at the end of the previous session — it had been
killed, presumably by session teardown, after only 11 of its planned 20 restarts, but
`best.npy` was already there: **max_violation 0.0298**, essentially the same quality as
the previous session's 0.0308 find, on a completely independent restart trajectory. This
is a real confirmation that restart-from-checkpoint reliably finds this quality of point,
not a one-off.) Wrapping a raw checkpoint vector back into a proper
`MathematicalProgramResult` for inspection (no pydrake binding exposes
`set_decision_variable_index` directly) needs an actual zero-iteration IPOPT call with
`bound_push`/`bound_frac` disabled — confirmed bit-exact (`GetSolution` reproduces the
input vector exactly) — promoted to `solve_backflip.result_from_vector()`.

**Two of the audit's five failures were the audit's own bugs, not the trajectory's:**

- **Torque envelope "35 N·m overshoot" was checking the wrong bound.** Directly evaluated
  the *actual* NLP constraint (`torque_speed_halfplanes`, box + both halfplanes) against
  the checkpoint: exactly zero overshoot, everywhere. The audit was checking
  `torque_speed_bound()` instead — documented (`go2_backflip/constants.py`,
  `tools/check_envelope.py`) to *intentionally* diverge from the halfplanes in the
  regenerating quadrant. Fixed `audit.check_envelope` to check the halfplanes (what's
  actually enforced), not the bound. STATUS.md's own next-steps list flagged this as
  worth checking; it was real.
- **"Monotone=False, max step 22°" was reporting solver noise as a defect.** Isolated the
  exact reversal: 0.00021° at the flight→absorb impact boundary (state continuity there
  is an exact equality, but the checkpoint is only accepted to ~0.03 constraint
  violation, not floating-point-exact) — six orders of magnitude below the reported "22°
  step" figure, which is actually the single largest (monotone, real) per-knot rotation
  during the fastest part of the tumble, not a reversal at all. Loosened the
  monotonicity/magnitude thresholds in `audit.check_rotation` to be judged against the
  solve's own tolerance (~1e-3) rather than a hardcoded 1e-6 that nothing but a true
  `is_success()` result could ever pass.

Net: **3/6 audit checks now pass** (rotation, friction cone, torque envelope) on the same
checkpoint that scored 1/6 before. The three genuine remaining failures — CoM not exactly
ballistic (6mm residual vs 1mm threshold), flight angular momentum drifting ~38-44%, and
collocation-vs-tight-integrator drift (up to 4.6 rad/s in v by end of phase) — all
independently implicate the same thing: **14 knots is too coarse to resolve flight.**

**Grew flight 14→20 knots** (`schedule.py`; tightened `h_max` 0.055→0.040 so the extra
knots can't just bunch up and reproduce the old coarse spacing — 20 knots over the same
~0.55-0.67s flight leaves ~0.035s average h, comfortably inside the new bound).
`nullity_check.py` re-run at the new size: **13/4052** equality rows, same thin/diffuse
pattern as before (was 12/3680) — no new LICQ redundancy introduced by the resize.

**Built mesh-refinement warm-starting** (`traj_opt/warm_start.py`): export resamples a
solved phase's *continuous* reconstruction (`ReconstructStateTrajectory`, first-order-hold
of u/lambda — the same interpolation the transcription itself assumes) at a new knot
count, so an unchanged-knot-count phase round-trips near-exactly and a grown phase gets
new interior points off the same curve, not new information invented. `solve_backflip.py`
gained `result_from_vector()` (above) and `--warm-start PATH.npz` to seed from this
instead of `guess.py`'s analytic guess.

**Then found it's the wrong tool for THIS transition, and why.** Warm-starting the new
20-knot flight from the 14-knot checkpoint gave an *initial* (iteration-0, before any
solver step) unscaled violation of **586** — worse than even the pre-single-shooting
kinematic-only guess ever was. Root cause, confirmed by direct dynamics evaluation
(`plant.EvalTimeDerivatives` at resampled knots vs. finite-differencing the resampled
velocity): the 14-knot checkpoint's per-component cubic reconstruction, in between its
own (coarse) knots, implies joint accelerations up to ~1700 rad/s² that don't match the
*true* forward dynamics at those same resampled points — the checkpoint was only ever
weakly consistent with the ODE in Hermite-Simpson's coarse, *integral* (Simpson's-rule)
sense at 14 points, not smoothly consistent in between them. This is the exact same
pathology `audit.check_integration` already caught independently (worst v-drift 4.6 rad/s
replaying this same checkpoint through a tight integrator) — refining the mesh makes a
previously-hidden discretization error visible instead of fixing it for free. **Plain
`guess.py` at 20 knots (no warm start) starts at violation 83** — worse than the 14-knot
single-shot guess's ~6-30ish historically, but far better than the 586 from resampling,
and uses the same already-validated single-shooting machinery (which is already
knot-count-generic — no code changes needed to run it at any flight `n_knots`). **Using
the plain analytic guess, not warm_start.py, for this transition.** `warm_start.py` is
kept as real, working infrastructure (not deleted) — it's the right tool once a mesh
already has a *smooth*, well-resolved solution to hand up to a finer one (e.g. a
follow-up 20→26 step, after 20 itself is in good shape), just not for jumping straight
from an under-resolved 14-knot source.

**Launched a production solve at 20 knots**: `--iters 400 --feas-tol 1e-4 --opt-tol 1e-2
--restarts 30 --burst-iters 300`, plain analytic guess, via the harness's own
`run_in_background` tracking this time (not a bare `nohup` + separate watcher — that
combination was exactly what left the previous session unable to confirm its production
run's fate). Old 14-knot checkpoints backed up to
`traj_opt/out/checkpoints_flight14_backup/` before this run's restart loop could
overwrite `traj_opt/out/checkpoints/`.

**Result: this is the best trajectory the project has produced.** All 30 restarts ran
to completion (~2.6 hours), best violation **0.0310** (restart 11) — essentially the
same constraint-violation *level* as the 14-knot best (0.0298/0.0308), but at 20 knots
that level now comes with the discretization errors actually fixed, exactly as
predicted:

| audit check | 14 knots | 20 knots |
|---|---|---|
| net rotation | PASS (-359.9999°) | PASS (-359.9991°) |
| CoM ballistic | **FAIL**, 6.1mm residual | **PASS**, 0.34mm residual |
| angular momentum conserved | **FAIL**, 38-44% drift | FAIL, but **1.16%** drift (~30x better) |
| friction cone | PASS | PASS |
| torque envelope (halfplanes) | PASS | PASS |
| collocation vs. tight integrator | **FAIL**, 0.64-1.14 (q) / 4.6-8.6 (v) | FAIL, but **0.036 (q) / 0.33 (v)** (~15-25x better) |

**4/6 audit checks now pass**, and the two still-failing ones are the same
discretization signature, just an order of magnitude smaller — not a new problem, the
same one continuing to shrink with mesh resolution, right where it should.

**Pushed to 26 knots next**, and re-tested the warm-start-vs-plain-guess question now
that a genuinely smooth, well-resolved (not jerky) 20-knot solution exists to warm-start
from: warm-starting from it gave iteration-0 violation **228** — much better than
resampling the jerky 14-knot source (586), confirming the smoothness theory, but still
*worse* than the plain analytic single-shot guess at 26 knots, which starts at **83.1** —
essentially identical to its own 20-knot number, since it re-simulates the continuous
dynamics from scratch at whatever knot count is asked and is therefore dynamically
self-consistent by construction regardless of sampling density, whereas any
resampling-based warm start (even from a good source) necessarily reconstructs from a
*discrete*, only-approximately-consistent trajectory. **Conclusion: for this problem,
`guess.py`'s single-shooting is the better warm start at every knot count tried so far,
full stop** — `warm_start.py`'s mesh-refinement resampling has not yet found a regime
where it beats it, though it remains available if a future need arises (e.g. warm-
starting after a *non*-knot-count formulation change, where the analytic guess would
need to change too). Launched the same restart pipeline at 26 knots with the plain
guess; nullity re-checked clean first (13/4424, same benign pattern).

**Result: another real improvement, best result of the project so far.** All 30
restarts completed (~2h50m), best violation **0.0283** (restart 28) — slightly better
than the 20-knot best (0.0310), still 4/6 audit checks passing:

| audit check | 14 knots | 20 knots | 26 knots |
|---|---|---|---|
| net rotation | PASS | PASS | PASS |
| CoM ballistic | FAIL, 6.1mm | PASS, 0.34mm | PASS, 0.21mm |
| angular momentum conserved | FAIL, 38-44% | FAIL, 1.16% | FAIL, **0.76%** |
| friction cone | PASS | PASS | PASS |
| torque envelope (halfplanes) | PASS | PASS | PASS |
| collocation vs. tight integrator | FAIL, 0.64-1.14(q)/4.6-8.6(v) | FAIL, 0.036(q)/0.33(v) | FAIL, 0.037(q)/**0.34(v)** |

Angular momentum kept improving with the extra knots (1.16%→0.76%), but the
integration-drift number **plateaued** rather than continuing to shrink (0.33→0.34 in
v, essentially flat) — the first sign in this progression that flight knot count alone
may not be the only thing left to fix for that specific check. Worth investigating
before growing the mesh further (26→32 or beyond): candidates are the `load`/`absorb`
phases (still analytic-guess-only, never single-shot, and still relatively coarse at
12/16 knots) or the impact map itself, rather than assuming flight is still the
bottleneck. Stopping the mesh-growth ladder here for this session — 26 knots is the
shipped configuration (`schedule.py`).

## 2026-08-20 session: bug 3 (a second instance of bug 2's mechanism)

Picked up from the interrupted SNOPT-on-shrunk-problem test (see attempt table
below): it finished, and it's a **negative result** — `info=13` on both the
feasibility and optimal passes (706s), same failure code as the pre-bug-fix
baseline. A second independent run earlier in the session gave `info=13` /
`info=43`. SNOPT has never once returned success in this project.

That repeated `info=13` on a problem that had already had bugs 1+2 fixed was
itself the signal: re-ran the FD-Jacobian/SVD check (method in the section
below) against the shrunk problem's guess and found real nullity again — 335
out of 4112 equality rows, 88% of it concentrated in `_add_symmetry`. Two real
bugs, found and fixed in `program.py`:

- **v[4]/q[5], bug 1's mechanism, missed the first time.** `v[4]` is the base's
  world-frame translational y-velocity, exactly conjugate to `q[5]` (base
  y-position) — `qdot=v` exactly, no rotation involved, same as the hip joints
  bug 1 already fixed. Pinning both `q[5]=0` and `v[4]=0` is the same
  redundant-with-the-collocation-defect pattern bug 1 described, just never
  applied to this one variable pair. Fix: drop the `v[4]` pin, same as bug 1.
  This alone cut nullity 335 → 281, 1-for-1 with the 54 rows removed.
- **quat_x/quat_z/base-y and hip position pins, bug 2's mechanism, missed the
  first time.** These are pinned to the same constant at *every* knot of
  *every* phase — bug 2's exact mechanism (a position equality repeated across
  consecutive knots is implied by that state's own collocation defect), just
  never applied to these two families; bug 2's fix only covered foot-pin,
  quaternion-norm, and leg-mirror. Fix: `TIGHT`-box them the same way.
  `program.py:_add_symmetry`. This cut nullity 281 → **16** (of 3680 rows) —
  the remaining 16 is thin and diffuse across boundary/stitching/contact rows,
  no single dominant family, and is roughly the same order of magnitude as the
  ~50 residual nullity bug 2's own fix left behind on the full-size problem
  (never itself chased down, evidently benign there).

**The diagnostic script is now a real file**, not a one-off scratch script:
`traj_opt/nullity_check.py`. Run it after any constraint-family change to
`program.py` — it monkeypatches `AddBoundingBoxConstraint`/`AddLinearConstraint`
to auto-tag each binding by call site (function:line) purely for readable
grouping, builds the guess the same way `solve_backflip.py` does, and reports
nullity plus which call sites dominate it.

## 2026-08-22 session: single-shooting the launch/flight guess

**Retried both solvers with bug 3 fixed** (still the shrunk 14-knot-flight
problem): SNOPT `info=13` then `info=43`, cost 5.09 on the optimal pass (real
progress vs. the ~24-29 cost pre-fix, but still not success) and a much longer
feasibility pass (1695s vs ~130-200s before). IPOPT: max-iter exceeded,
unscaled constraint violation 36.5 — same order as before the fix, and the
same "dips low, then wanders" pattern under closer inspection (`inf_pr`
oscillates through the 7-90 range across the run, not a clean dip-then-rise).
Bug 3 was real and worth fixing (it's what the nullity check demanded), but
it did not change either solver's qualitative failure mode. **Ruled out the
linear solver lever too**: this Drake IPOPT build only actually supports
`spral` (confirmed by trying to set `linear_solver=ma57` — Drake's own error
message lists `spral`/`custom` as the only valid settings, despite `strings`
on `libdrake.so` turning up `ma27`/`ma57`/etc. as recognized-but-unlinked
option names). Step 3 below is closed, not just deprioritized.

**What actually moved the needle: single-shooting launch/flight (step 4).**
Every attempt's very first solve is an unweighted, zero-cost feasibility
pass — and it had never once succeeded, at up to 800-3000 iterations, across
every combination of bug fixes and solver tried. That rules out a cost/
feasibility tradeoff and points at the guess itself not being dynamically
close enough to reach. Implemented in `guess.py`: after the existing
kinematic path produces a `u(t)`/`lambda(t)` choice per knot (unchanged),
launch and flight are now **re-simulated** forward from their start state
using that same `u`/`lambda` fed through the identical
`applied_generalized_force` port DirectCollocation itself uses — the result
satisfies the true dynamics exactly, by construction, rather than only
balancing the manipulator equation pointwise per knot. (One bug on the way:
chaining flight's simulation from launch's actual ending *velocity* while
`u(t)` had been derived assuming a different, self-consistent velocity from
flight's own kinematic path caused the two to fight and the whole trajectory
to blow up — z fell to -0.5 m, x to -1.4 m. Fixed by chaining position only
(via z0/x0) and simulating from flight's own self-consistent `v(0)`, matching
what `u(t)` was actually derived against.)

Result: nullity actually improved slightly (16 → 12), and IPOPT's best
unscaled constraint violation dropped **36.5 → 5.7** (800 iters) — a ~6x
improvement, with the last ~30 iterations of that run monotonically
decreasing (13.4 → 5.7), unlike every prior attempt's noise. Tried 3000
iterations expecting it to keep converging: it didn't — ended at 8.6, and the
trajectory in the 2900-3000 range oscillates in a 6-13 band with no net
progress. So the guess fix gave a real, one-time improvement in the best
reachable quality, but more IPOPT iterations alone don't close the remaining
gap; it circles a region instead of converging into it.

Tried SNOPT with this same new guess, expecting a proper SQP step might
handle the near-feasible region differently than IPOPT's L-BFGS: it didn't —
`info=44` on feasibility (594.7s, a code not seen before in this project),
then `info=13` with cost **51.5** on the optimal pass, markedly *worse* than
the 5.09 SNOPT got from the *old* kinematic-only guess. So the new guess
helps IPOPT and hurts SNOPT — plausibly because the simulated trajectory's
larger velocities (flight `|v|` up to 16 rad/s or m/s, vs a much gentler
kinematic profile) make for a worse-conditioned linearization point for an
active-set method, even though it's a strictly more dynamically-accurate
point. **Best result to date, across both sessions: IPOPT + the new guess,
unscaled constraint violation 5.7, still not converged.**

## 2026-08-22 session, continued: restarting from checkpoints is the real lever

Ruled out `mu_strategy=adaptive` next: it genuinely changes IPOPT's early
iterations (verified directly — a 30-iteration side-by-side shows completely
different `inf_pr` trajectories) but converges to the *exact same* final
result as `monotone` by iteration 800 (confirmed twice, independently). Dead
end, not worth revisiting.

**What worked:** instead of one long continuous solve, re-solve in short
bursts (300 iterations), warm-starting each burst from the previous burst's
raw result (not reverting to the best-seen point — reverting to an identical
checkpoint with identical options just reproduces the identical result,
since IPOPT is deterministic given the same start and options; confirmed by
accident when 16 consecutive "revert to best" restarts all returned the
exact same viol=3.603). Chaining forward through bursts, even through
temporary regressions, explores materially different territory each burst.
Two independent runs of this both stumbled onto excellent points:
**unscaled constraint violation 0.055** (run 1) and **0.078** (run 2, at
"restart 2") — both far better than anything a continuous run ever found,
and better than the historical best (0.068, on the full 26-knot problem).
Runs don't reproduce each other bit-for-bit despite identical code and
starting guess, most likely because `spral` (the linear solver) is
multi-threaded and floating-point reduction order isn't deterministic across
threads — small numerical noise compounds over hundreds of nonlinear
iterations into different trajectories. Restarting is therefore closer to a
stochastic multi-start than a deterministic refinement, and needs the good
checkpoints saved immediately or they're lost (learned the hard way — the
0.055 result from run 1 was never saved and could not be recovered).

**Chasing the run-2 checkpoint (`r2`, viol=0.078) further**: a long
continuous solve from `r2` dipped to an even better **0.0308** partway
through (iteration ~1764) — the best result of the entire project, over 2x
better than the previous historical best — but then wandered back up to 2.0
by iteration 3000. Recapturing that dip by capping `max_iter` at the right
point reproduced it exactly (IPOPT *is* deterministic run-to-run from a
fixed starting point and options — only different starting points or option
changes introduce the threading noise above). Also tried a pure
zero-cost feasibility pass from that same dip, hoping to close the last gap
without a cost gradient fighting it: made things *worse* (0.81), same
"wanders away" pattern. **The pattern holds everywhere: good points are
transient under continued iteration, regardless of whether cost is present.
The only reliable way found to capture one is to cap iterations at the
right spot and save immediately** — there is no known way yet to make IPOPT
run *past* one of these points without losing it.

**Found and fixed a real, previously-latent bug** while trying to audit the
0.0308 checkpoint: `solve_backflip.py:extract()` called
`result.GetSolution(bp.lam[p])` on a 3D array of decision variables, which
`GetSolution` doesn't support (only 1D/2D) — this would have crashed on the
very first `is_success()==True` this project ever produced, since `main()`
only calls `extract()` past that gate. Never triggered before because no
solve had reached success. Fixed the same way `set_guess()` already handles
it elsewhere: flatten to 2D for `GetSolution`, reshape back.

**Reframing what "done" means here:** IPOPT's `is_success()` requires both
primal feasibility *and* dual optimality (cost-gradient stationarity). The
0.0308 checkpoint has excellent primal feasibility but enormous dual
infeasibility (~1.5e6) — it is nowhere near a certified local optimum, just
very close to a *feasible* point. Since the actual purpose of this
trajectory is an RL-imitation reference, not a publishable optimality
certificate, a small primal violation may already be good enough to use even
without `is_success()==True`.

**Ran the full audit against the 0.0308 checkpoint** (bypassing the
`is_success()` gate). 1 of 6 checks pass outright, but the picture is more
nuanced than a flat fail:

- **Net rotation: -359.9993° vs a -360° target** — off by 0.0007°. The audit
  marks this FAIL only because its tolerance is 1e-6 rad; this is, in every
  practical sense, an exact backflip.
- **Contact forces inside the friction cone: PASS**, cleanly (cone slack
  1.7e-7 N).
- **Flight CoM ballistic**: FAIL, 1.24 cm residual vs a 1 mm audit threshold
  — plausibly just the 14-knot flight discretization being coarse, not a
  correctness problem.
- **Flight angular momentum conserved**: FAIL, ~11.5% drift (L_y=-4.33,
  drift 0.499 N.m.s) — a real, if moderate, discretization artifact.
- **Torque envelope**: FAIL, 39 N.m worst overshoot — but the audit checks
  against `torque_speed_bound()`, the true non-smooth envelope, not
  `torque_speed_halfplanes()`, the linear approximation actually enforced in
  `program.py`. These two are known and *documented* to diverge in the
  regenerating quadrant (`tools/check_envelope.py`), so at least part of
  this "overshoot" is expected behavior, not necessarily a violated NLP
  constraint — worth rechecking against the halfplane bound directly before
  treating this as a real problem.
- **Collocation matches a tight integrator**: FAIL, badly — 1.14 (q) / 8.65
  (v) worst end-of-phase drift when a phase's reconstructed input is
  replayed through a tight-tolerance simulator. Individual knot-level
  collocation defects are all small (consistent with the 0.03 aggregate NLP
  violation), but they accumulate across a phase into a much larger
  cumulative drift — classic small-per-step / large-cumulative-error, and
  the most likely explanation is exactly what step 1 below already
  proposes: the flight phase's coarse 14-knot discretization.

Net read: **this checkpoint gets the actual maneuver essentially right**
(near-exact rotation, real contact forces) but is not yet something to trust
as a hardware/RL reference — the integration-drift and momentum-conservation
failures both point at the same fix (more flight knots), which was already
next on the list before any of today's work.

Also wrote `traj_opt/out/backflip.npz` from this checkpoint for inspection
(747 samples @ 500 Hz) — **do not treat this as a validated trajectory**,
it's a snapshot of a `NOT SOLVED` result with 5/6 audit failures, kept only
for `replay.py`/`mj_divergence.py` spot-checking. `traj_opt/out/` is now
gitignored so a file like this is never accidentally committed.

**The restart loop is now a real, committed feature**, not scratch scripts:
`solve_backflip.py --restarts N --burst-iters K`. After the normal
feasibility/optimal passes, if not already solved, it re-solves in `K`-
iteration bursts (default 300), chaining forward from each burst's raw
result, tracking and immediately disk-saving (`traj_opt/out/checkpoints/`)
whichever burst had the lowest `max_violation()` — a new helper that walks
`prog.GetAllConstraints()` directly rather than parsing IPOPT's printed
summary, so it works mid-loop and for SNOPT too. If nothing ever reaches
`is_success()`, `main()` now still runs the full `extract`/`audit`/`resample`/
`save` pipeline on the best checkpoint found, rather than exiting with
nothing (matching what was done by hand above) — always prints
`NOT SOLVED (best viol=...)` first so this is never mistaken for a real
success.

**A production run using this feature was started and left running in the
background as this session wrapped up**: `--solver ipopt --iters 400
--feas-tol 1e-4 --opt-tol 1e-2 --restarts 20 --burst-iters 300`, aiming to
reproduce or beat the 0.0308 result. It is a detached background process
(will keep running after this session ends) and writes every checkpoint to
`traj_opt/out/checkpoints/*.npy` plus a final `traj_opt/out/backflip.npz`
regardless of whether anyone is watching — **check those paths first** next
session before launching anything new; the run may already have finished
(or found something better) by then. Its console log lives in this
session's scratchpad, which may not survive — the `traj_opt/out/` files are
the durable record.

## What's real and confirmed

**Two structural NLP bugs found and fixed**, both confirmed with a
finite-difference Jacobian rank check (not guesswork) — see method below.

### Bug 1 — conjugate position/velocity pins (fixed, in `program.py`)

`_add_symmetry` originally pinned a quaternion component (`q[1]=qx`) *and* its
kinematically-conjugate angular velocity (`v[0]=wx`) to 0 simultaneously, same
for `q[3]/v[2]` and every hip joint's `(q, qdot)` pair. Since `qdot = f(q)·v`
for the base and `qdot = v` exactly for a revolute joint, the pair is
redundant with the collocation defect's own row for that state — the active
equality Jacobian goes rank-deficient at a point that is *actually feasible*.
SNOPT silently reported the whole problem infeasible (`info=13`) rather than
surfacing this. IPOPT instead threw `TOO_FEW_DOF` outright.

Confirmed with an isolated single-phase (12-knot stance-hold) test harness:
adding `wx=0` alongside `qx=0` broke a problem that solved cleanly (SNOPT
`info=1`) with either alone. **Fix:** drop the redundant velocity-level pin,
keep only the position-level one (`program.py:_add_symmetry`, now only pins
`q[1],q[3],q[5]` and `q[7+j]` for hips; velocity is left to follow from the
dynamics).

### Bug 2 — position equalities repeated across knots (fixed, `TIGHT` in `program.py`)

Separately: any position-level equality held at *every* knot of a phase
(foot-pin `pos(q)==pf`, quaternion unit-norm `q0²+q2²=1`, thigh/calf mirror
`q_FL==q_FR`) is *also* redundant with the collocation defect once it holds at
two consecutive knots — same mechanism, different variables. Verified by
computing the actual FD Jacobian of every equality-type constraint at the
initial guess and taking its SVD: nullity dropped from **764 → 50** by
converting exactly these three families from hard `==` to a `±1e-4` box
(`TIGHT` constant, `program.py`). u-mirror/hip-antisymmetry and the no-contact
coupling identity were tested the same way and found to be *already*
independent — left as exact equalities.

**Method, if this needs re-deriving or extending to a different formulation:**
build every equality-type binding's local Jacobian via forward-difference at
the current guess, stack into one big sparse-ish matrix, `np.linalg.svd`, and
look at `nullity = rows - rank`. To find *which* rows are responsible, take
the left singular vectors `U[:, i]` for the near-zero singular values and see
which row labels dominate each one (`argsort(-abs(U[:,i]))`). This is exact
and fast (SVD of ~5000×5000 is seconds), much faster than guessing.

### Also fixed along the way (real, but secondary)

- `HOME_BASE_HEIGHT` (0.27, the MJCF keyframe) sits the feet 1 cm into the
  floor once you use the actual foot-sphere geometry. TO uses
  `K.STAND_BASE_HEIGHT = 0.2800479196045126` instead; the keyframe is
  untouched.
- `guess.py`'s per-knot contact wrench solve originally fixed `lam` at a
  uniform `mg/n_contacts` and solved only for `u` — since `B` has zero rows on
  the 6 base DOFs, this left the base-row equilibrium unsolved whenever the
  CoM wasn't centered over the feet, producing a "static" guess with several
  rad/s² of spurious acceleration. Fixed by solving `[u; lam]` jointly
  (`guess.py:_wrench`).
- The `launch`-phase rigid-pivot kinematic guess wanted **negative** (tension)
  contact forces at points — the prescribed rotation was more aggressive than
  gravity + push-only contact can realize. `_wrench` now clips `lam` to the
  friction cone and re-closes only the actuated (leg) rows, accepting a base-
  row residual there instead of an unphysical guess.
- The flight-phase tuck weight was a clipped triangle (`min(t,1-t)/0.3`) —
  C0 but not C1, giving a genuine finite-difference acceleration spike
  (−179 rad/s²) at the kink regardless of grid resolution. Replaced with a
  smoothstep ramp.
- Flight's leg trajectory reset to `HOME_LEG` at t=0 regardless of what launch
  actually ended on (front tucked, rear extended) — a real position
  discontinuity at the phase seam. `_flight_path` now continues from launch's
  actual endpoint.
- The `TUCK_BOX` self-collision guard (`tools/tuck_box.py`) was applied at
  *every* phase in `program.py`, not just `flight` as the plan intended —
  it conflicted with the launch pose's necessarily-extended rear leg. Now
  scoped to `FLIGHT` only.
- Added generous, physically-loose bounds on previously-unbounded DOFs (base
  x/z, base vx/vz, foothold x/y, contact force and impulse upper bounds) —
  IPOPT's own exit message calls out unbounded variables as a known cause of
  poor interior-point behavior.
- `guess.py` didn't zero the departing feet's `lam_z` at a phase's last knot
  to match `program.py`'s "release" constraint (load→launch drops FL/FR,
  launch→flight drops RL/RR) — the guess disagreed with a *fixed* variable's
  required value by up to 34 N. Fixed in `guess.py:build()`.

## What's NOT yet true

The full backflip has **not converged** with either solver after ~5 hours of
attempts across two sessions:

| Attempt | Solver | Knots (flight) | Result |
|---|---|---|---|
| baseline | SNOPT | 26 | `info=13` (this was bug 1, undiagnosed) |
| bug 1 fixed | SNOPT | 26 | still `info=13`, but real SQP progress (686s vs instant bail-out) |
| bugs 1+2 fixed, `TIGHT=1e-7` | IPOPT | 26 | 3000 iters, unscaled constraint violation 0.135 |
| + `TIGHT=1e-4`, bounds, guess fix | IPOPT | 26 | 3000 iters, unscaled constraint violation 0.068 |
| shrunk (prior session) | IPOPT | 14, loose tol | 800 iters, unscaled constraint violation **31.5** — *worse*, not better |
| shrunk, bugs 1+2 only | SNOPT | 14, loose tol | `info=13` (127.7s) then `info=13` (706.0s) — infeasible |
| shrunk, bugs 1+2 only, earlier run | SNOPT | 14, loose tol | `info=13` (208.1s) then `info=43` (1289.4s) — infeasible |
| bugs 1-3 fixed, old guess | SNOPT | 14, loose tol | `info=13` then `info=13`, cost 5.09 (2078s) — infeasible, but real progress |
| bugs 1-3 fixed, old guess | IPOPT | 14, loose tol | 800 iters, unscaled constraint violation 36.5 — same oscillating pattern as before |
| bugs 1-3 fixed, **new (single-shot) guess** | IPOPT | 14, loose tol | 800 iters, unscaled constraint violation **5.7** — best result yet |
| bugs 1-3 fixed, new guess, more iters | IPOPT | 14, loose tol | 3000 iters, unscaled constraint violation 8.6 — *worse*, oscillates 6-13 with no net progress |
| bugs 1-3 fixed, new guess | SNOPT | 14, loose tol | `info=44` (594.7s) then `info=13`, cost 51.5 (2064s) — *worse* than SNOPT's old-guess result |

The shrink experiment gave a **negative result worth remembering** (IPOPT
diverging from a promising start on the pre-bug-3 formulation) that in
hindsight was really bug 3 showing up on a different problem size. With bugs
1-3 all fixed, the real remaining bottleneck showed up cleanly: every
attempt's first solve is an unweighted, zero-cost feasibility pass, and it
had never once succeeded — at up to 3000 iterations, on either solver, before
*or* after the guess fix below. That rules out a cost-vs-feasibility
tradeoff and points at guess quality/optimizer capability as the remaining
gap, not the constraint formulation.

**Single-shooting launch/flight (`guess.py`, see its module docstring) is a
real, confirmed improvement for IPOPT** (36.5 → 5.7, ~6x) but doesn't fully
close the gap, and *hurts* SNOPT (5.09 → 51.5) — plausibly because the
simulated trajectory's larger, more realistic velocities make for a worse
linearization point for an active-set method even though they're more
dynamically accurate. **Best result to date: IPOPT + the new guess, unscaled
constraint violation 5.7, not converged.** More IPOPT iterations alone don't
help (3000 iters gave 8.6, oscillating) — the bottleneck now looks like
genuine optimizer capability (limited-memory BFGS curvature quality) rather
than formulation or guess quality.

**Ruled out, not just deprioritized:** IPOPT's linear solver. This Drake
build only actually has `spral` linked — confirmed by trying to set
`linear_solver=ma57` directly; Drake's own error lists `spral`/`custom` as
the only valid settings, despite `ma27`/`ma57`/etc. appearing as recognized
option-name strings in `libdrake.so`.

## Concrete next steps, in order of expected payoff

All four items below replace the previous list, every entry of which has now been
answered: (1) which phase carries the drift — flight, not `load`/`absorb`; (2) 26→32
knots — worse, reverted; (3) `warm_start.py` from a well-resolved source — still loses
to the analytic guess; (4) nullity after each change — done, 15/4424 and 16/4796.

1. **The two remaining audit failures are both flight-side and both small**
   (angular momentum 0.56%, integration drift 0.029 q / 0.219 v). Knot count is
   spent as a lever. What has never been tried: the transcription itself — the
   first-order hold on *generalized force* means the contact term at a collocation
   point is the average of the endpoints' `J^T lambda` rather than
   `J(q_col)^T lambda_col`, which `audit.check_integration` prices but nothing
   fixes. Flight is contact-free, so there it reduces to the FOH on torque alone.
2. **The foot rolls; the pin says it spins in place.** See the 2026-08-31 section —
   a pitching calf slides its material contact point at `R*omega_y` (~0.7 m/s at
   takeoff) with nothing tying the friction force's direction to that slip. Making
   the foothold translate with the roll is the honest model and the last known
   *modelling* error, as opposed to discretization error, in the stance phases.
3. **A converged `is_success()` is still out of reach and may not be worth chasing** —
   the dual infeasibility at these points is ~1e6, i.e. nowhere near a certified local
   optimum, while the primal violation is 0.0376 and every physical check but two
   passes. For an RL-imitation reference, primal feasibility is what matters.
4. Run `traj_opt/nullity_check.py` after any `program.py` or `schedule.py` change —
   cheap, and it is what found bugs 1-3.

## Files

| Path | Role | State |
|---|---|---|
| `src/go2_backflip/constants.py` | shared constants, Drake↔MuJoCo mapping | done, `verify_parity.py` passes 6/6 |
| `tools/check_envelope.py` | unit check for the linear torque-speed envelope | done, passes |
| `tools/tuck_box.py` | self-collision-free sagittal joint box (flight only) | done |
| `traj_opt/schedule.py` | phase table — flight **26 knots** (grown 14→20→26; 32 tried 2026-09-01 and reverted, much worse), `h_max` 0.055→0.040→0.032 | — |
| `tools/check_npz.py` | geometric review of a saved `.npz`: exact lowest point of every collision geom, closest **non-foot** approach, flight I_yy, net pitch — measures the file that gets replayed, not the solver's residuals | done; found the stale output, the between-knot floor bulge, and the grazing knee/head |
| `traj_opt/program.py` | the NLP: constraints, all fixed bugs live here; `BODY_CLEARANCE` = 10 mm for non-foot witness spheres, 0 for the two foot spheres | builds cleanly, nullity **15/4424** at the 26-knot size (unchanged by the clearance margin — it moved an inequality bound only), not yet solved |
| `traj_opt/nullity_check.py` | FD-Jacobian/SVD LICQ diagnostic — found bugs 1-3; now eliminates fixed variables the way IPOPT does before taking the rank | done, rerun after any constraint-family change. **13** at the 26-knot size. Only ever checks the GUESS and only exact equalities — for what stalls a solve, check the active set at the solved point instead (2026-09-03: nullity 202, cond 5e20) |
| `traj_opt/guess.py` | analytic initial guess, single-shooting launch/flight, knot-count-generic (no changes needed to run at any flight size) | done, best IPOPT result yet at 14 knots (violation 5.7 pre-restart, 0.0298-0.0308 post-restart); at 20 knots starts at violation 83 pre-solve |
| `traj_opt/warm_start.py` | mesh-refinement warm-start: export a solved phase's continuous reconstruction resampled at a new knot count, `--warm-start` flag on `solve_backflip.py` | done, but wrong tool for 14→20 (resampling an under-resolved source gave violation 586, worse than the plain analytic guess's 83) — keep for refining an already-smooth solution (e.g. a later 20→26) |
| `traj_opt/solve_backflip.py` | CLI, `--solver ipopt\|snopt`, `--feas-tol`, `--opt-tol`, `--restarts`/`--burst-iters`, `--warm-start`, `--ipopt-opt KEY=VALUE`, `--proximal W` | `extract()` 3D-`GetSolution` bug fixed; restart-from-checkpoint is a real feature; `result_from_vector()` added for inspecting raw checkpoints without a fresh solve; `bound_push` fixed at 1e-8 (2026-09-03 -- the default was costing every restart burst its seed) |
| `traj_opt/audit.py` | post-solve physics audit, **10 checks** | tenth check added 2026-09-03: sagittal symmetry, measured on all four legs, because `MIRROR` loosened the bound that used to assert it. Ninth check: closest non-foot approach vs `BODY_CLEARANCE`, because penetration-below-zero could not see a geom grazing the floor at 0.1 mm; torque-envelope check fixed to test `torque_speed_halfplanes` (was testing the wrong, intentionally-divergent bound); rotation-check tolerances loosened to the solve's own scale (were tighter than any non-`is_success()` result could ever pass) |
| `traj_opt/replay.py`, `traj_opt/mj_divergence.py` | meshcat playback, MuJoCo open-loop divergence (TODO-10) | untested end-to-end, no solve has reached `is_success()` yet |

Run with:
```
PYTHONPATH=/opt/drake/lib/python3.12/site-packages:tools:traj_opt \
    .venv/bin/python traj_opt/solve_backflip.py --iters 3000 [--solver ipopt|snopt] [--feas-tol X] [--opt-tol X]
```
