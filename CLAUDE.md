# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Stage 1 of a three-stage pipeline: **Drake full-order trajectory optimization → mjlab RL
imitation → Unitree Go2 hardware**. This repo builds the first stage — a dynamically-feasible
`t, q, v, tau` backflip reference trajectory for the Go2, produced by a Drake
`DirectCollocation` trajectory optimization — plus the shared MJCF model and the Drake↔MuJoCo
parity checks both later stages depend on. Nothing downstream (RL training, hardware) lives in
this repo yet.

The full original design (contact schedule rationale, constraint derivations, sizing
calculations) is in the plan this was built from; the day-to-day numeric state of the
optimization (current best result, what's converged, what isn't) is **not** in this file — see
`traj_opt/STATUS.md`, which is the living log and is updated every session. Read it before
assuming anything about where the optimization currently stands.

## Environment

The project is managed by **uv** (`pyproject.toml` + `uv.lock`, Python 3.12, venv at `.venv`).
Drake and MuJoCo are ordinary locked dependencies — there is no `/opt/drake` system install to
put on `PYTHONPATH`, and no `PYTHONPATH` is needed at all. Every script runs the same way:

```
uv run traj_opt/<script>.py
uv run tools/<script>.py
```

The shared constants live in the installed package `go2_backflip` (`src/go2_backflip/`), which
`uv sync` installs in **editable** mode, so edits to it take effect with no reinstall. Modules
inside `traj_opt/` still import each other flat (`from program import ...`); that works because
Python puts a script's own directory on `sys.path`, which is why only the cross-directory
constants import needed to become a package.

## Commands

```bash
# Prove Drake and MuJoCo agree on go2.xml (structure, mass, mass matrix, inverse dynamics,
# open-loop torque tape). Checks A-E must pass; F is report-only (contact is expected to differ).
uv run tools/verify_parity.py

# Solve the backflip. Writes traj_opt/out/backflip.npz (MuJoCo convention) even on failure,
# for inspection. Key flags: --solver {ipopt,snopt} (default ipopt), --iters, --feas-tol,
# --opt-tol, --feasibility-only, --restarts N --burst-iters K (short-burst restart search --
# see "Restart-from-checkpoint" below, the single biggest lever found for this problem).
uv run traj_opt/solve_backflip.py --restarts 20 --burst-iters 300

# Diagnose "solver reports infeasible on a problem that looks feasible" -- see LICQ note below.
uv run traj_opt/nullity_check.py

# Physics audit of a solved trajectory (rotation, ballistic flight, friction cone, torque
# envelope, collocation-vs-integration drift) -- runs automatically at the end of solve_backflip.py.

# Meshcat playback / MuJoCo open-loop divergence check of traj_opt/out/backflip.npz.
# replay.py records a meshcat animation (one frame per 500 Hz knot) and holds the server
# open, so playback speed is a timeScale slider in the browser's "Animations" panel rather
# than a CLI flag. --live restores the old real-time streaming pass.
uv run traj_opt/replay.py [--fps 500] [--no-hold]
uv run traj_opt/replay.py --live [--speed 0.25] [--loops 3]
uv run traj_opt/mj_divergence.py

# Regenerate the self-collision-free sagittal tuck box (src/go2_backflip/constants.py:TUCK_BOX) and
# sanity-check the linear torque-speed envelope against the true (non-smooth) one:
uv run tools/tuck_box.py
uv run tools/check_envelope.py
```

No test suite exists; `verify_parity.py`, `check_envelope.py`, and the audit in
`solve_backflip.py` are the correctness checks for this codebase.

## Architecture

**`go2_mjcf/`** (git submodule) is the single ground-truth MJCF, parsed independently by Drake
and MuJoCo. **`src/go2_backflip/constants.py`** is the single source of truth for everything derived
from it — actuator limits, poses, joint index layout, and the Drake↔MuJoCo state conversion —
specifically so the Drake TO and the (future) mjlab RL config cannot drift apart. Two
conversion details worth knowing before touching either engine: the quaternion/position blocks
are swapped (`[quat, xyz]` in Drake vs `[xyz, quat]` in MuJoCo), and angular velocity is
expressed in the **world** frame in Drake but the **body** frame in MuJoCo — a difference that
only bites once the base is tilted, i.e. exactly during a flip. `torque_speed_bound()` (true,
non-smooth envelope) and `torque_speed_halfplanes()` (its linear NLP-safe relaxation) read the
same constants so they can't diverge except in the one place they're meant to (the regenerating
quadrant).

**`traj_opt/program.py`** builds `BackflipProgram`: one shared `MathematicalProgram` holding
four `pydrake.planning.DirectCollocation` phases (`traj_opt/schedule.py` has the phase table —
load/launch/flight/absorb, contact sets, knot counts). The trick that makes contact forces work
as decision variables is the input port choice: `applied_spatial_force` is abstract-valued and
can't be a DirectCollocation input, but `applied_generalized_force` is vector-valued (18) and
already excludes gravity/damping (those are force elements inside `EvalTimeDerivatives`), so a
per-knot constraint `port(k) == B@u_k + sum_i J_i(q_k)^T lambda_i_k` ties ordinary `u`/`lambda`
decision variables to it. Phases are glued by state-continuity constraints, except the
load→launch touchdown (`schedule.IMPACT`), which instead gets a full impulsive-contact equation
(mass matrix times velocity jump equals the sum of foot impulses, no-slip enforced post-impact).
Sagittal symmetry (hip pins, left/right leg mirroring, `quat_x=quat_z=0`) is enforced hard,
which is also what turns "one full backflip" into a single terminal equality: with those
components zeroed the base quaternion is `[cos(θ/2), 0, sin(θ/2), 0]`, so `-2π` lands exactly on
`[-1,0,0,0]`, numerically distinct from identity.

**The recurring bug class in this NLP is LICQ (constraint-qualification) violations**, not
modeling errors: an *exact* equality that turns out to be implied by another already-imposed
constraint (most often a state's own collocation defect, once a position pin holds at two
consecutive knots, or a position pin paired with its exact kinematically-conjugate velocity)
makes the active-constraint Jacobian rank-deficient at a point that is genuinely feasible.
SNOPT's symptom is a silent, wrong `info=13` ("infeasible") on a point that isn't; IPOPT's is an
outright `TOO_FEW_DOF` error. This has recurred three times so far, always inside
`_add_symmetry`; the fix is always either dropping the redundant velocity-level pin or replacing
an exact position equality with a tight (`TIGHT = 1e-4`) box. **`traj_opt/nullity_check.py`** is
the diagnostic — an FD-Jacobian/SVD rank check over every equality binding at the initial guess
— and should be the first thing run whenever a solver reports infeasibility on a problem that
looks right, before any other debugging.

**`traj_opt/guess.py`** builds the analytic initial guess: hand-authored kinematic paths per
phase, converted to `(u, lambda)` per knot via the manipulator equation, then (for
launch/flight, where the hand-authored path's implied accelerations are least trustworthy)
**re-simulated forward** through the exact same `applied_generalized_force` port
DirectCollocation itself uses, so the guess satisfies the true dynamics exactly rather than only
balancing them pointwise per knot. This single-shooting step was a large, measured improvement
to solver performance — see `traj_opt/STATUS.md` for the numbers.

**`traj_opt/solve_backflip.py`** is the CLI: builds the program, sets the guess, solves
(feasibility pass, then a costed pass), optionally runs a **restart-from-checkpoint** loop
(`restart_loop()`), extracts/resamples the result onto a uniform 500 Hz grid, and writes
`traj_opt/out/backflip.npz` in MuJoCo convention (even on non-success, for inspection — never
treat that output as validated without checking `result.is_success()`/the audit). The restart
loop exists because continuous IPOPT runs on this problem reliably wander away from good points
once they find them; short bursts that always **chain forward** from each burst's raw result
(never revert to the best-seen point — IPOPT is deterministic given identical start/options, so
reverting just reproduces the same result forever) explore new territory instead. Every burst's
raw decision-variable vector is saved to `traj_opt/out/checkpoints/` immediately, since a good
point is one burst away from being overwritten by a worse one.

**`traj_opt/audit.py`** is the physics check independent of the solver's own reported residuals:
net rotation, ballistic flight (CoM parabola + angular momentum conservation), friction-cone
compliance, torque-envelope compliance, and — specific to this transcription — re-simulating
each phase with a tight-tolerance integrator to price the error introduced by Hermite-Simpson's
first-order-hold of the *generalized force* (so the contact term at collocation points is the
average of endpoint `J^T lambda`, not `J(q_col)^T lambda_col`; contact constraints themselves
bind at knots only).

Solver preference: IPOPT over SNOPT for this contact-rich problem (SNOPT has never once
returned success here). Both are still runnable via `--solver`.
