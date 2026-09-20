# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Stages 1 and 2 of a three-stage pipeline: **trajectory optimization → mjlab RL imitation (or a
deterministic tracking controller) → Unitree Go2 hardware**. Stage 1 (`traj_opt/`, `tools/`)
produces a dynamically feasible backflip reference for the Go2 — `t, qpos, qvel, qacc, ctrl`, the
contact schedule and the planned ground forces — plus the shared MJCF model, the constants both
later stages depend on, and the checks that prove the reference against MuJoCo. Stage 2
(`src/go2_backflip/rl/`) is the mjlab motion-tracking environment that imitates it; it is built
and checked but not yet trained. Nothing for hardware lives here yet.

`traj_opt/STATUS.md` is the living log: current numbers, what was tried, what is open. Read it
before assuming where things stand.

**History you should know about.** Until 2026-09-18 the optimizer was a Drake
`DirectCollocation` program on the full 18-DOF model with sagittal symmetry imposed by
constraints. It never once returned solver success. It was deleted on 2026-09-19; its code and
three weeks of logs (LICQ repairs, restart-from-checkpoint searches, a 16-option IPOPT screen)
are in git history before commit `30f0d0a` if ever needed. The lesson it paid for, and the
reason the current formulation looks the way it does: **impose a symmetry by parametrisation,
never by constraint** — every pin on a coordinate the dynamics already propagate is a
rank-deficient row waiting to happen.

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

# Stage 2 (mjlab). The tasks are registered through the `mjlab.tasks` entry point in
# pyproject.toml, so mjlab's own CLIs see them (after `uv sync` if the entry point changed).
uv run list-envs
uv run tools/rl_env_check.py                  # build the env, print managers, one zero-action step
uv run tools/rl_env_check.py --replay [--view] [--action-noise 0.2]   # zero policy = mj_track's controller at 50 Hz; exit 0 iff it lands
uv run play Mjlab-Tracking-Flat-Unitree-Go2-Backflip --agent zero --no-terminations --motion-file traj_opt/reference/backflip_mjlab.npz
uv run train Mjlab-Tracking-Flat-Unitree-Go2-Backflip --env.scene.num-envs 4096 --agent.max-iterations 10000
```

No unit-test suite exists; `verify_sagittal.py`, `validate.py`, `mj_track.py` and
`rl_env_check.py --replay` are the correctness checks, in that order of dependence.

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
load. That is a real limit a hand-written planar model would miss: the old reference ran its
hips to 21.6 of 23.7 N·m (the shipped one peaks at 7.5).
Conventions: `R_y(+theta)` tips the nose down so a backflip is `theta → -2π`; forces
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
  top** — they are implied by the defects and are exactly the LICQ failure that sank the old
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
old L-BFGS solve never did. Variables and constraint rows are hand-scaled in `_NLP.var/con`.

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
`--monte-carlo` as the figure of merit when changing `Config` — the old reference landed
78.5 % of trials; the current one lands 100 %.

**Actuator limits come in two flavours.** `PEAK_TORQUE`/`hardware_torque_limits()` are datasheet
peaks and must keep matching `go2.xml`'s `forcerange`; `mj_track` clips against them (with the
torque-speed halfplanes) because that is what the motor does. The optimizer uses
`Config.torque_sf × NOMINAL_TORQUE` (default 0.90) and `speed_sf` (0.90) so a tracking controller
inherits headroom. Use `torque_speed_halfplanes(peak)` for anything on a tape;
`torque_speed_bound()` under-rates a back-driven motor (`tools/check_envelope.py`).

Drake is no longer in the optimization path — only `replay.py` (meshcat) and
`verify_parity.py` use it.

## Stage 2: the mjlab environment (`src/go2_backflip/rl/`)

mjlab's tracking task (`mjlab.tasks.tracking`, a BeyondMimic re-implementation) on the Go2, with
the actuator driven the way a Unitree low-level command is: **PD position target + velocity
target + feedforward torque at 50 Hz** (the DDS rate). mjlab ships no Go2, so everything
robot-specific is here and reads `constants.py`:

* **`go2_robot.py`** — the `EntityCfg`. `spec_fn` loads `go2.xml` and deletes its XML `<motor>`
  actuators *and keyframes* (mjlab adds its own motors and `init_state` key; leaving the keys in
  fails compile). Actuators are `DcMotorActuatorCfg` (kp 60, kd 2): the only mjlab actuator that
  honours a feedforward torque, and its linear torque-speed clip is exactly the hardware envelope
  once `saturation_effort = peak / (1 − CORNER_SPEED_FRAC)`. Armature/damping stay the XML's.
  `soft_joint_pos_limit_factor` **must be 1.0** — the reference runs the calves to 0.02 rad from
  the hard limit and any smaller factor clips it on reset and penalises it.
* **`actions.py`** — `JointPositionFeedforwardAction`: position target `q_ref[t] + scale·action`
  (a residual on the reference; zero action *is* the `mj_track` controller), plus `dq_ref[t]` and
  `ctrl_ff[t]` from the npz, all through mjlab's shared command-delay buffer. Keep the term key
  `"joint_pos"` and the `JointPositionAction` base: mjlab's ONNX export checks both. Frame
  bookkeeping: after a reset the robot is at frame `s` while `time_steps` reads `s+1`, so
  everything indexed at `time_steps` is the frame to reach by the end of the coming step.
* **`mdp.py`** — `motion_finished` (a `time_out=True` truncation at the last frame; without it
  the command wraps and teleports mid-episode), `gravity_offset` (gravity DR emulated as a
  per-episode constant `m_b·Δg` wrench on every body, because mjlab cannot batch `opt.gravity`;
  never add another wrench event on the robot, it would overwrite it), `reference_contact`.
* **`env_cfg.py`** — the walking-policy-like actor (joint pos/vel, gyro, projected gravity, last
  action: 5-frame history, noise, 0–1-step delay; plus the next reference frame and the base
  orientation error; no base linear velocity / position error in the default task), all the DR
  (`pseudo_inertia`, friction, joint friction/damping/armature, `pd_gains`, `effort_limits`,
  encoder bias, gravity, velocity pushes, 0–6 ms command delay), physics 2 ms × decimation 10.
  `cone="elliptic", impratio=100` is load-bearing: pyramidal breaks the launch.
* **`tools/rl_env_check.py`** is the acceptance test for the env config, the analogue of
  `mj_track.py`: the zero policy must land the reference through mjlab. "Landed" is the pass
  criterion; "clean" is reported separately and is marginal at nominal (the head strikes the floor
  at touchdown for two steps; 2 ms of latency decides it in plain MuJoCo as well).

Deployment caveat: the exported ONNX metadata does not carry kp/kd, `ctrl_ff` or the reference
offset. The hardware stage must take kp/kd from `go2_robot.py` and `q_ref`/`dq_ref`/`ctrl_ff` from
the npz, indexed by the same frame counter as the policy.
