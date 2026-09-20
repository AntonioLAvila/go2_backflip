# Backflip trajectory optimization — status

Living log for the current (sagittal / CasADi) formulation. The Drake formulation it replaced,
and its three weeks of logs, were deleted on 2026-09-19; see git history before `30f0d0a`.

## Where it stands (2026-09-18)

`traj_opt/reference/backflip.npz` — 942 samples, 1.882 s: load 0.533, launch 0.278, flight
0.407, absorb 0.664. Apex base height 0.61 m. Cold start, IPOPT `Solve_Succeeded` in 55
iterations (~6 s in IPOPT, ~40 s wall including CasADi graph construction), constraint violation
1e-8, dual infeasibility ~1e-10.

| | old Drake reference (2026-09-06) | current |
|---|---|---|
| IPOPT `is_success()` | never (dual inf. floor ~3, viol 0.58) | yes, from a cold start |
| time to a solution | hours of chained restart bursts | ~40 s |
| audit | 11/11 at knots; tape over hardware envelope by 0.51 N·m | 17/17 on the 500 Hz tape, 5.8 N·m *inside* hardware envelope |
| peak torque hip / thigh / calf (N·m) | 21.6 / 23.2 / 44.1 (all at the limit) | 7.5 / 15.3 / 27.1 |
| closed-loop MuJoCo, PD+FF, nominal | lands, rear knees scrape the floor | lands clean |
| `mj_track --sweep` | 13/18 land | 17/18 land (fails only kp=20) |
| `mj_track --monte-carlo` | 78.5 % land, 6 % clean | 100 % land (500 trials), 54 % clean |
| lands with **no feedforward** at all | no (does not rotate) | yes |

"Clean" = no geom other than a foot ever touches the floor. The remaining stray contacts under
perturbation are rear knees (`thigh_col` + `calf_upper`) grazing during the landing crouch, and
they scale with latency (25 % of trials at 0 ms, 71 % at 6 ms, measured on the pre-ship solve
that differs only by the static-start condition). That is tracking error in the
absorb phase, i.e. the feedback controller's job, not the reference's.

## What changed, and why it worked

1. **Symmetry by parametrisation** (`sagittal.py`): 7 coordinates instead of 18 DOF + pins. No
   quaternion, no mirror rows, no TIGHT boxes. Model verified against `mj_inverse`/`mj_jac` to
   1e-12 (`tools/verify_sagittal.py`). Symmetry leak of the real `go2.xml` (its base inertia is
   not exactly mirror-symmetric): ≤ 0.055 N / N·m over the whole flip envelope — negligible, and
   closed-loop lateral drift is 1.6 mm.
2. **Stance as an ODE**: contact at acceleration level + Baumgarte (α = 20 /s), no position or
   velocity pins. Foot drift on the tape: 44 µm vertical, 32 µm rolling-slip.
3. **Exact Hessians** (CasADi) instead of L-BFGS through Drake's autodiff.
4. **Separated Hermite–Simpson**: dynamics enforced at midpoints with their own `λ`, not a
   first-order hold of the generalized force.

## Tried and rejected (do not retry without a new idea)

* **Torque continuity across lift-off + hard slew limit** (motivated by mesh-dependence of the
  post-lift-off retraction spike). Continuity alone doubles the cost (0.72 → 1.48) because the
  front legs push until the last instant and an unloaded calf under +17 N·m accelerates at
  ~500 rad/s². Adding a slew limit of 20 u_max/s went locally infeasible (all phase durations
  pinned at their upper bounds); 40 and 80 /s converge but to cost 1.4–1.8 with 3–4 audit
  failures. The soft `w_du` rate cost is what stays.
* **`w_du` above 2e-3**: 5e-3 and 1e-2 both lose the friction-0.4 sweep case (measured at
  `body_clearance` 0.05).
* **`body_clearance` 0.05 instead of 0.03**: Monte Carlo clean-landing rate *drops*
  (60 % → 38 %). More floor margin at the knots buys a more aggressive motion elsewhere.
* **Lower `torque_sf`**: 0.8 and 0.7 both still solve, pass 17/17 and land 100 %, but clean rate
  falls to 43 % and 28 %. The flip is feasible at 70 % torque; it is not *better* there.
* **Cold start on a finer mesh**: different, worse local optimum (cost 1.40 vs 0.65). Use
  `--refine`.

## Open

* Mesh dependence: warm-started 2× refinement raises cost ~19 % (0.72 → 0.85), almost all of it
  in the `w_du`/`w_a` terms around the contact switches. Trajectories are visually the same and
  both audit 17/17. Not a blocker; a cleaner fix would be a short explicit "unload" sub-phase.
* The landing is scheduled as a simultaneous four-foot touchdown. A rear-first or front-first
  landing may absorb more gently; with 40 s solves this is now cheap to explore — add a phase to
  `Config.phases`.
* Real joint dry friction is unmodelled (the model reads `go2.xml`'s 0.05 viscous damping only);
  the RL stage randomises it (`joint_friction` 0-0.3 N·m, damping ×0.4-2).
* Out-of-plane stability is untested beyond the 1.6 mm drift and a 100 N shove: the reference is
  exactly sagittal, so roll/yaw are regulated only by the hip PD. RL should randomise there.

## Hand-off

* RL (mjlab tracking task): `traj_opt/reference/backflip_mjlab.npz`, 50 fps, 0.5 s standing hold
  before and 1.0 s after; extras `ctrl_ff`, `contact`, `grf`, `phase`. The mjlab environment now
  exists: `src/go2_backflip/rl/`, tasks `Mjlab-Tracking-Flat-Unitree-Go2-Backflip[-State-Estimation]`
  (see CLAUDE.md, "Stage 2"). Not trained yet.
* Deterministic: `tools/mj_track.py` *is* a working baseline controller (kp 60, kd 2 + `ctrl`).
  The natural upgrade is TVLQR about the tape using `sagittal.py`'s exact derivatives.

## RL stage (2026-09-19)

Environment built and checked, no training run yet. `tools/rl_env_check.py --replay` drives the
env with a zero policy (the action is a residual on the reference, so zero = the mj_track
controller, but held at 50 Hz through mjlab's DcMotor actuators, delay buffers and mujoco_warp):

| | mj_track, 500 Hz | mjlab env, 50 Hz hold |
|---|---|---|
| rotation / final tilt / height | −361.4° / 1.4° / 0.275 m | −360.8° / 0.9° / 0.276 m |
| lands | yes | yes (all 8 envs, also under 0.2 white action noise; 0.3 breaks the landing) |
| clean | yes | no: `head_sphere` hits the floor for 2 steps at touchdown (~3 kN peak in the contact sensor) |

The head strike is marginal, not an env bug: the same trajectory in plain MuJoCo grazes or not
depending on 2 ms of latency (STATUS "54 % clean" above), and solver iterations / a 1 ms physics
step do not change it. A pyramidal friction cone does break the launch, so `cone="elliptic",
impratio=100` (go2.xml's option block) is load-bearing in the env config. Global position error
after landing is ~0.23 m (the robot lands short of the reference's `x_land`); the flip itself
tracks to 1-7 cm mean body error.

Open for training: the stock tracking terminations (`anchor_ori` 0.8, `ee_body_pos` 0.25 m) are
what end noisy replays at touchdown -- expect them to dominate early `Episode_Termination` and
loosen if they do; the adaptive frame sampler has only 4 bins on a 3.4 s clip. Deployment note:
mjlab's ONNX metadata does not carry kp/kd (the DcMotor `<motor>` has gain 1), `ctrl_ff`, or the
reference offset -- the hardware stage reads those from `go2_robot.py` and the npz.
