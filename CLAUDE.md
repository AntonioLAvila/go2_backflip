# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Stage 1 of a three-stage pipeline: **trajectory optimization → mjlab RL imitation (or a
deterministic tracking controller) → Unitree Go2 hardware**. This repo produces a dynamically
feasible backflip reference for the Go2 — `t, qpos, qvel, qacc, ctrl`, the contact schedule and
the planned ground forces — plus the shared MJCF model, the constants both later stages depend
on, and the checks that prove the reference against MuJoCo. Nothing downstream lives here yet.

`traj_opt/STATUS.md` is the living log: current numbers, what was tried, what is open. Read it
before assuming where things stand.

**History you should know about.** Until 2026-09-18 the optimizer was a Drake
`DirectCollocation` program on the full 18-DOF model with sagittal symmetry imposed by
constraints. It never once returned solver success; three weeks of work on it (LICQ repairs,
restart-from-checkpoint searches, a 16-option IPOPT screen) is archived, with its complete logs,
in `legacy/drake_dircol/`. It is not maintained and its scripts are not expected to run from
there. The lesson it paid for, and the reason the current formulation looks the way it does:
**impose a symmetry by parametrisation, never by constraint** — every pin on a coordinate the
dynamics already propagate is a rank-deficient row waiting to happen.

## Environment

Managed by **uv** (`pyproject.toml` + `uv.lock`, Python 3.12, venv at `.venv`). CasADi, MuJoCo,
Drake and mjlab are ordinary locked dependencies; no `PYTHONPATH` is needed.

```
uv run traj_opt/<script>.py
uv run tools/<script>.py
```

The shared package `go2_backflip` (`src/go2_backflip/`) is installed editable. Scripts inside
`traj_opt/` import each other flat (`import flip`), which works because Python puts a script's
own directory on `sys.path`.

## Commands

```bash
# Solve from a cold start (~40 s, of which ~6 s is IPOPT), audit the 500 Hz tape against MuJoCo,
# roll it out closed-loop in MuJoCo. Writes traj_opt/out/backflip.npz. Every float field of
# flip.Config is a flag (--torque-sf, --body-clearance, --mu, --w-du, ...).
uv run traj_opt/solve.py [--quiet]
uv run traj_opt/solve.py --ship --note "why"      # also promote to traj_opt/reference/
uv run traj_opt/solve.py --refine 2               # re-solve on a 2x mesh, warm-started

# The audit alone, on any tape (default: the shipped reference). MuJoCo-only physics checks.
uv run traj_opt/validate.py [npz]

# THE acceptance test: feedforward + joint PD through MuJoCo's own contact model with hardware
# torque-speed clipping. Exit 0 iff it lands and stands.
uv run tools/mj_track.py [npz] [--kp 60 --kd 2] [--view]
uv run tools/mj_track.py --sweep                  # one perturbation at a time
uv run tools/mj_track.py --monte-carlo 500        # all at once, randomised, parallel

# Prove the CasADi sagittal model IS go2.xml restricted to its symmetry plane (~1e-12).
uv run tools/verify_sagittal.py
# Prove Drake and MuJoCo agree on go2.xml (Drake is now only used for meshcat replay).
uv run tools/verify_parity.py

# Hand-off to mjlab's motion-tracking task: 50 fps clip with standing holds on both ends.
uv run tools/export_mjlab.py

# Meshcat playback (Animations panel has the timeScale slider) / open-loop divergence.
uv run traj_opt/replay.py --npz traj_opt/reference/backflip.npz
uv run traj_opt/mj_divergence.py --npz traj_opt/reference/backflip.npz

# Regenerate model-derived constants (self-collision tuck box, floor-clearance witness spheres).
uv run tools/tuck_box.py
uv run tools/clearance_points.py
uv run tools/check_envelope.py
```

No unit-test suite exists; `verify_sagittal.py`, `validate.py` and `mj_track.py` are the
correctness checks, in that order of dependence.

`traj_opt/reference/` is tracked: `backflip.npz` (500 Hz, MuJoCo convention),
`backflip_mjlab.npz` (the RL clip) and `manifest.json` (full `Config`, IPOPT status, every audit
margin, the closed-loop result). A solve is a deterministic cold start, so the manifest's config
is a *recipe* — but IPOPT/MUMPS are not bit-reproducible across machines, so the npz is still
the artifact. `--ship` refuses unless IPOPT succeeded, the audit is clean, and the closed-loop
rollout lands with no stray contact. `traj_opt/out/` is scratch and gitignored.

## Architecture

**`go2_mjcf/`** (git submodule) is the single ground-truth MJCF.
**`src/go2_backflip/constants.py`** is the single source of truth for everything derived from it
— actuator limits, poses, joint layout, the torque-speed envelope, witness spheres, and the
Drake↔MuJoCo state conversion (quaternion/position blocks swapped; angular velocity world-frame
in Drake, body-frame in MuJoCo).

**`src/go2_backflip/sagittal.py`** is the model the optimizer uses: the Go2 restricted to its
mirror-symmetric manifold, 7 coordinates `[x, z, theta, thigh_f, calf_f, thigh_r, calf_r]`, as
CasADi expressions. Nothing is hand-derived: it reads MuJoCo's *compiled* `go2.xml` (tree,
masses, inertias, armature, damping), assembles the Lagrangian of all 13 bodies symbolically,
and lets CasADi differentiate. The mirrored hip-abduction coordinates are carried symbolically
and evaluated at zero, which yields `hip()` — the torque needed to *hold* the hips at zero under
load. That is a real limit a hand-written planar model would miss: the legacy reference ran its
hips to 21.6 of 23.7 N·m (the shipped one peaks at 7.5). Conventions: `R_y(+theta)` tips the nose down so a backflip is `theta → -2π`; forces
are **per foot**, torques **per motor**, and the factor 2 for the mirrored pair is applied inside
`sagittal.py` and nowhere else. `embed_qpos/qvel/ctrl` lift to MuJoCo's full state.

**`traj_opt/flip.py`** is the NLP: four phases (load / launch / flight / absorb, same schedule
rationale as always — front feet supply the pitch-up moment and leave first), each transcribed by
Hermite–Simpson in **separated form**: every node, knot *and* midpoint, has its own
`(q, v, a, u, λ)` and satisfies the equations of motion there, so nothing is first-order-held.
Three design decisions carry the whole thing, and each exists to keep the constraint Jacobian
full rank:

* **Stance is an ODE, not a DAE.** The rolling-sphere foot constraint is imposed at the
  *acceleration* level at every node with Baumgarte feedback
  (`g̈ + 2αġ + α²(g − g0) = 0`). With the equations of motion that is a square nonsingular system
  for `(a, λ)`; the flow stays on the contact manifold because it starts there (HOME at rest;
  inherited; or the impact map's `J v⁺ = 0`). **Do not add position or velocity foot pins on
  top** — they are implied by the defects and are exactly the LICQ failure that sank the legacy
  formulation. The audit measures the resulting drift instead (tens of microns).
* **Boundary conditions pin only as many things as the manifold has freedoms.** Terminal:
  `theta`, four leg angles, base velocity (3), base acceleration (3). Base height and joint rates
  follow from the contact constraint. Initial `a_base = 0` makes the start a static equilibrium
  so a standing hold can be spliced on.
* **A constraint already fixed by a neighbouring phase is skipped at the shared knot**
  (swing-foot clearance at a lift-off/touchdown knot; the friction cone where `λ` is pinned to
  zero for release).

The foot is a rolling sphere: `foot()` is holonomic in the plane (`x_centre − R·calf_pitch`,
`z_centre − R`) and its Jacobian is exactly the *material* contact point's, so one `J` serves the
constraint and the `Jᵀλ` term. Touchdown is an inelastic impact (`MΔv = 2JᵀΛ`, `Jv⁺ = 0`, impulse
in the friction cone). Swing-foot clearance ramps in over a fixed **fraction** of the phase —
anything counted in nodes makes the optimum move with the mesh.

IPOPT gets exact gradients, Jacobians and Hessians from CasADi (`expand=True`), which is why a
crude piecewise-linear key-pose guess (`initial_guess`) converges in ~50 iterations where the
L-BFGS legacy solver never did. Variables and constraint rows are hand-scaled in `_NLP.var/con`.

**Local optima are real.** A cold start on a 2× mesh lands in a different, worse solution
(cost 1.40 vs 0.65). Refine with `--refine`, which warm-starts through `tape.regrid`; a warm
start also switches IPOPT to a low monotone barrier. Expect the cost to rise ~15–20% on
refinement: torque is allowed to jump across a contact switch and the post-lift-off retraction
transient sharpens as the mesh resolves it. Forcing torque continuity or a hard slew limit was
tried and doubles the cost or goes locally infeasible — see `STATUS.md`.

**`traj_opt/tape.py`** resamples a solution. Inside an interval the solution *is* a polynomial
(cubic Hermite for `q`, `v`; quadratic for `a`, `u`, `λ`), so the tape is that polynomial
evaluated, not a fit. `regrid` is the same evaluation onto another mesh.

**`traj_opt/validate.py`** is the audit, run on the 500 Hz tape (between knots, where a
transcription hides things) and asked of **MuJoCo alone** wherever possible so the CasADi model
cannot vouch for itself: `mj_inverse` residual against `ctrl + Σ Jᵀ grf`, design and hardware
torque-speed halfplanes, limits, unilateral/cone/no-slip contact, `mj_geomDistance` floor
clearance of every non-foot geom, ballistic CoM and conserved angular momentum in flight, and
per-phase re-integration under a tight adaptive integrator.

**`tools/mj_track.py`** is what "ready for the next stage" means operationally. The audit says
the tape is consistent with rigid-body physics; this says a dumb joint PD + feedforward, through
MuJoCo's soft contact and the motor's real torque-speed clipping, actually lands it. Use
`--monte-carlo` as the figure of merit when changing `Config` — the legacy reference landed
78.5 % of trials; the current one lands 100 %.

**Actuator limits come in two flavours.** `PEAK_TORQUE`/`hardware_torque_limits()` are datasheet
peaks and must keep matching `go2.xml`'s `forcerange`; `mj_track` clips against them (with the
torque-speed halfplanes) because that is what the motor does. The optimizer uses
`Config.torque_sf × NOMINAL_TORQUE` (default 0.90) and `speed_sf` (0.90) so a tracking controller
inherits headroom. Use `torque_speed_halfplanes()`-style linear envelopes for anything on a tape;
`torque_speed_bound()` under-rates a back-driven motor (`tools/check_envelope.py`).
`constants.TORQUE_SF`/`DESIGN_TORQUE` are legacy (0.98) and used only by `legacy/`.

Drake is no longer in the optimization path — only `replay.py` (meshcat) and
`verify_parity.py` use it.
