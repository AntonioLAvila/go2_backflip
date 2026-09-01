# Backflip trajectory optimization — status

Last updated 2026-09-01. Not yet converged (`is_success()==False`), and it may
never need to be — see next steps. Best confirmed result: IPOPT, unscaled
constraint violation **0.0376**, on the 26-knot-flight problem, audited at
**6/8 checks passing**, shipped as `out/backflip.npz` and reproducible from
`out/checkpoints/best.npy` with `--from-checkpoint`.

> **Read the 2026-09-01 section FIRST.** It supersedes the 2026-08-25 one: the
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
| `tools/check_npz.py` | geometric review of a saved `.npz`: exact lowest point of every collision geom, flight I_yy, net pitch — measures the file that gets replayed, not the solver's residuals | done; found the stale output and the between-knot floor bulge |
| `traj_opt/program.py` | the NLP: constraints, all three fixed bugs live here | builds cleanly, nullity 13/4424 at the 26-knot size (was 12/3680 at 14 knots), not yet solved |
| `traj_opt/nullity_check.py` | FD-Jacobian/SVD LICQ diagnostic — found bugs 1-3 | done, rerun after any constraint-family change |
| `traj_opt/guess.py` | analytic initial guess, single-shooting launch/flight, knot-count-generic (no changes needed to run at any flight size) | done, best IPOPT result yet at 14 knots (violation 5.7 pre-restart, 0.0298-0.0308 post-restart); at 20 knots starts at violation 83 pre-solve |
| `traj_opt/warm_start.py` | mesh-refinement warm-start: export a solved phase's continuous reconstruction resampled at a new knot count, `--warm-start` flag on `solve_backflip.py` | done, but wrong tool for 14→20 (resampling an under-resolved source gave violation 586, worse than the plain analytic guess's 83) — keep for refining an already-smooth solution (e.g. a later 20→26) |
| `traj_opt/solve_backflip.py` | CLI, `--solver ipopt\|snopt`, `--feas-tol`, `--opt-tol`, `--restarts`/`--burst-iters`, `--warm-start` | `extract()` 3D-`GetSolution` bug fixed; restart-from-checkpoint is a real feature; `result_from_vector()` added for inspecting raw checkpoints without a fresh solve |
| `traj_opt/audit.py` | post-solve physics audit | torque-envelope check fixed to test `torque_speed_halfplanes` (was testing the wrong, intentionally-divergent bound); rotation-check tolerances loosened to the solve's own scale (were tighter than any non-`is_success()` result could ever pass) |
| `traj_opt/replay.py`, `traj_opt/mj_divergence.py` | meshcat playback, MuJoCo open-loop divergence (TODO-10) | untested end-to-end, no solve has reached `is_success()` yet |

Run with:
```
PYTHONPATH=/opt/drake/lib/python3.12/site-packages:tools:traj_opt \
    .venv/bin/python traj_opt/solve_backflip.py --iters 3000 [--solver ipopt|snopt] [--feas-tol X] [--opt-tol X]
```
