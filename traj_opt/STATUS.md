# Backflip trajectory optimization — status

Last updated 2026-08-22. Not yet converged (`is_success()==False`). This is a
resumption point, not a finished pipeline. Best result to date: IPOPT,
unscaled constraint violation **0.0308** on the shrunk (14-knot flight)
problem, found via restart-from-checkpoint (not a single continuous solve) —
see "restarting from checkpoints is the real lever" below. Near-exact net
rotation and clean contact forces; real remaining gaps in flight momentum
conservation and integration accuracy that point at more flight knots.

> **2026-08-20 note:** everything in this repo (`tools/`, `traj_opt/`, this
> file, and the `go2_mjcf` submodule fix) is now committed and pushed,
> including to the submodule's own fork remote — the "uncommitted working
> tree, one accident from data loss" state flagged in the previous note is
> resolved. See git log for the commit.

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

1. **Make restart-from-checkpoint a real, reusable feature of
   `solve_backflip.py`**, not scratch scripts in `/tmp`. It's the single
   biggest lever found in this project (5.7 → 0.0308). Needs: a loop of
   short bursts (a few hundred iterations each) with a `max_violation()`
   helper (already written in the scratch scripts — walks
   `prog.GetAllConstraints()`, no need for IPOPT's own printed summary),
   chaining forward from each burst's raw result (**not** reverting to the
   best-seen point between bursts — reverting to an identical checkpoint
   with identical options just reproduces the identical result, since IPOPT
   is deterministic given the same start and options), and saving every
   burst's decision-variable vector to disk immediately so a good point is
   never lost the way run 1's 0.055 result was.
2. **Grow the flight phase's knot count back up** (14→20→26), warm-starting
   from the 0.0308 checkpoint (once saved via 1) rather than `guess.py`'s
   analytic guess. This directly targets the audit's two real remaining
   failures (integration drift, angular momentum conservation) — both look
   like coarse-discretization artifacts on a checkpoint that already gets
   the maneuver essentially right.
3. **Re-check the torque-envelope audit failure against
   `torque_speed_halfplanes()` directly**, not `torque_speed_bound()` — the
   two are known to diverge in the regenerating quadrant, so some (maybe
   all) of the reported 39 N.m overshoot may not reflect a violated NLP
   constraint at all.
4. **Extend single-shooting to `load`/`absorb`** for full internal
   consistency, though these are currently static holds and already provably
   exact fixed points of their own equilibrium `u`/`lambda` — expected low
   payoff, do this only after 1-2.
5. Run `traj_opt/nullity_check.py` again after any further `program.py` or
   `guess.py` change — cheap (~1 min), and it's what found bugs 1-3.

## Files

| Path | Role | State |
|---|---|---|
| `tools/go2_constants.py` | shared constants, Drake↔MuJoCo mapping | done, `verify_parity.py` passes 6/6 |
| `tools/check_envelope.py` | unit check for the linear torque-speed envelope | done, passes |
| `tools/tuck_box.py` | self-collision-free sagittal joint box (flight only) | done |
| `traj_opt/schedule.py` | phase table — flight currently **14 knots** (shrunk for the diagnostic above, not yet grown back) | — |
| `traj_opt/program.py` | the NLP: constraints, all three fixed bugs live here | builds cleanly, nullity 12/3680 (was 335), not yet solved |
| `traj_opt/nullity_check.py` | FD-Jacobian/SVD LICQ diagnostic — found bugs 1-3 | done, rerun after any constraint-family change |
| `traj_opt/guess.py` | analytic initial guess, now single-shooting launch/flight | done, best IPOPT result yet (violation 5.7) |
| `traj_opt/solve_backflip.py` | CLI, supports `--solver ipopt\|snopt`, `--feas-tol`, `--opt-tol` | `extract()` 3D-`GetSolution` bug fixed (never triggered before, no solve had reached success); restart-from-checkpoint (next steps step 1) not yet built in, only in scratch scripts |
| `traj_opt/audit.py` | post-solve physics audit | untested end-to-end (never reached, no solve has succeeded) |
| `traj_opt/replay.py`, `traj_opt/mj_divergence.py` | meshcat playback, MuJoCo open-loop divergence (TODO-10) | untested end-to-end, same reason |

Run with:
```
PYTHONPATH=/opt/drake/lib/python3.12/site-packages:tools:traj_opt \
    .venv/bin/python traj_opt/solve_backflip.py --iters 3000 [--solver ipopt|snopt] [--feas-tol X] [--opt-tol X]
```
