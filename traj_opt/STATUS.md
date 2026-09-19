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
* `DAMPING` in `constants.py` (0.1, "guess") is unused; the model reads `go2.xml`'s 0.05. Real
  joint friction is unmodelled and is a sim-to-real item for the RL stage's domain randomisation.
* Out-of-plane stability is untested beyond the 1.6 mm drift and a 100 N shove: the reference is
  exactly sagittal, so roll/yaw are regulated only by the hip PD. RL should randomise there.

## Hand-off

* RL (mjlab tracking task): `traj_opt/reference/backflip_mjlab.npz`, 50 fps, 0.5 s standing hold
  before and 1.0 s after; extras `ctrl_ff`, `contact`, `grf`, `phase`. mjlab ships no Go2 tracking
  config — one has to be written against `go2_mjcf/go2.xml` (anchor body `base`).
* Deterministic: `tools/mj_track.py` *is* a working baseline controller (kp 60, kd 2 + `ctrl`).
  The natural upgrade is TVLQR about the tape using `sagittal.py`'s exact derivatives.
