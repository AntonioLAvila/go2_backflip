# Backflip trajectory optimization — status

Last updated 2026-08-20. Not yet converged. This is a resumption point, not a
finished pipeline.

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

**Not yet done:** re-solve with this fix in place. The shrunk-problem SNOPT/
IPOPT attempts in the table below all predate it — none of those numbers
reflect the current `program.py`. This is the next thing to try, and there's
real reason to expect it matters: SNOPT is LICQ-sensitive by construction
(that's the entire reason bugs 1-3 were all found via SNOPT's silent
`info=13` while IPOPT threw explicit errors), and SNOPT has literally never
been tried on a `program.py` with all three bugs fixed. Try SNOPT again first,
on the shrunk (14-knot flight) problem for speed, before going back to IPOPT
tuning (steps 3-4 below).

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

The shrink experiment gave a **negative result worth remembering**: IPOPT on
the smaller/coarser flight discretization got promisingly close early
(`inf_pr` down to ~8–14 by iteration 20–30) then **diverged away from that**,
ending up markedly worse (constraint violation 31.5) than the full-size run's
0.068 — not "scale was the whole problem." And both SNOPT runs on this same
shrunk problem came back infeasible, which is what triggered the bug 3
diagnosis above (same `info=13` signature bugs 1 and 2 were originally found
through). **Nothing in this table reflects bug 3's fix yet** — every row here
predates it.

## Concrete next steps, in order of expected payoff

1. **Retry SNOPT on the shrunk (14-knot flight) problem with bug 3 fixed.**
   `PYTHONPATH=... .venv/bin/python traj_opt/solve_backflip.py --solver snopt
   --iters 800 --feas-tol 1e-4 --opt-tol 1e-2`. SNOPT has never been run
   against a `program.py` with all three LICQ bugs fixed — every prior SNOPT
   attempt still had at least one of them live. Fast (~15-25 min), and this
   time there's a real mechanism-level reason to expect a different result,
   not just a re-roll.
2. Run `traj_opt/nullity_check.py` again first if `program.py` changes at all
   before that — cheap (~1 min), and it's what found bugs 1-3.
3. **If SNOPT still fails, try IPOPT on the bug-3-fixed shrunk problem** before
   concluding SNOPT just doesn't suit this problem — the 31.5 constraint
   violation in the table above was measured on a formulation with a real
   LICQ defect, so it isn't a clean read on IPOPT either.
4. **If both solvers do better on the small problem, grow it back up**
   (16→20→26 flight knots), warm-starting each size from the previous solve.
5. **Check IPOPT's linear solver** (`spral`, Drake's vendored default; MA27/
   MA57 usually more robust if available) and **reconsider the `launch`/
   `flight` guess** (single-shooting instead of the analytic kinematic path)
   — both still apply if 1-4 don't fully resolve it, per the previous
   session's reasoning.

## Files

| Path | Role | State |
|---|---|---|
| `tools/go2_constants.py` | shared constants, Drake↔MuJoCo mapping | done, `verify_parity.py` passes 6/6 |
| `tools/check_envelope.py` | unit check for the linear torque-speed envelope | done, passes |
| `tools/tuck_box.py` | self-collision-free sagittal joint box (flight only) | done |
| `traj_opt/schedule.py` | phase table — flight currently **14 knots** (shrunk for the diagnostic above, not yet grown back) | — |
| `traj_opt/program.py` | the NLP: constraints, all three fixed bugs live here | builds cleanly, nullity 16/3680 (was 335), not yet solved |
| `traj_opt/nullity_check.py` | FD-Jacobian/SVD LICQ diagnostic — found bugs 1-3 | done, rerun after any constraint-family change |
| `traj_opt/guess.py` | analytic initial guess | done, all known inconsistencies fixed |
| `traj_opt/solve_backflip.py` | CLI, supports `--solver ipopt\|snopt`, `--feas-tol`, `--opt-tol` | runs, doesn't converge yet |
| `traj_opt/audit.py` | post-solve physics audit | untested end-to-end (never reached, no solve has succeeded) |
| `traj_opt/replay.py`, `traj_opt/mj_divergence.py` | meshcat playback, MuJoCo open-loop divergence (TODO-10) | untested end-to-end, same reason |

Run with:
```
PYTHONPATH=/opt/drake/lib/python3.12/site-packages:tools:traj_opt \
    .venv/bin/python traj_opt/solve_backflip.py --iters 3000 [--solver ipopt|snopt] [--feas-tol X] [--opt-tol X]
```
