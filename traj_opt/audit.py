"""Physics audit of a solved backflip, independent of SNOPT's own residuals.

The integration check is the one specific to this transcription: Hermite-Simpson holds the
generalized force first-order, so at collocation points the contact term is the average of
the endpoint J^T lambda. Re-simulating each phase with a tight integrator is what prices
that approximation. If it is large, raise the stance knot count before anything else.
"""

from __future__ import annotations

import numpy as np
from pydrake.multibody.tree import BodyIndex, JacobianWrtVariable
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource, TrajectorySource

from go2_backflip import constants as K
from program import make_plant, TUCK_RAMP, XQ, XV
from schedule import FLIGHT, PHASES

G = 9.81
RESULTS: list[tuple[str, bool, str]] = []


def report(name, ok, detail=""):
    RESULTS.append((name, ok, detail))
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


def _pitch(q):
    """Unwrapped pitch from the sagittal quaternion [cos(t/2), 0, sin(t/2), 0].

    Unwrap the HALF angle and double it. Unwrapping the doubled value instead would read a
    single atan2 wrap as 2*pi when it is really 4*pi, and silently pass a half-turn.
    """
    return 2.0 * np.unwrap(np.arctan2(q[:, 2], q[:, 0]))


def check_rotation(phases):
    q = np.vstack([p["x"][:, XQ] for p in phases])
    theta = _pitch(q)
    net = theta[-1] - theta[0]
    # Monotonicity can only be trusted to the solve's own constraint tolerance -- below
    # that it's measuring solver noise, not physics (e.g. a ~1e-4 rad reversal right at a
    # phase seam, where continuity is a soft/TIGHT-level equality, not floating-point
    # exact). Report the worst positive (reversing) step, but only fail on one that
    # clearly exceeds the achieved constraint violation.
    worst_reversal = max(0.0, np.diff(theta).max())
    # 1e-3 rad, not 1e-6: the terminal quaternion is an exact NLP equality, but this
    # checkpoint (like every result so far) is accepted at max_violation ~1e-2, not
    # solved to is_success()==True, so a tolerance tighter than that measures the solve's
    # remaining primal gap, not a rotation defect.
    report("net rotation is one full backflip",
           abs(net + 2 * np.pi) < 1e-3 and worst_reversal < 1e-3,
           f"{np.degrees(net):.4f} deg, worst reversal {np.degrees(worst_reversal):.5f} deg, "
           f"max |step| {np.degrees(np.abs(np.diff(theta)).max()):.1f} deg")


def check_ballistic(plant, phases):
    ph = phases[FLIGHT]
    ctx = plant.CreateDefaultContext()
    t = ph["t"] - ph["t"][0]
    com, ang = [], []
    for x in ph["x"]:
        plant.SetPositions(ctx, x[XQ])
        plant.SetVelocities(ctx, x[XV])
        c = plant.CalcCenterOfMassPositionInWorld(ctx)
        com.append(c)
        ang.append(plant.CalcSpatialMomentumInWorldAboutPoint(ctx, c).rotational())
    com, ang = np.array(com), np.array(ang)

    # z should be exactly a -g/2 parabola; fit only the free constants.
    A = np.stack([np.ones_like(t), t], axis=1)
    resid_z = com[:, 2] + 0.5 * G * t ** 2 - A @ np.linalg.lstsq(A, com[:, 2] + 0.5 * G * t ** 2,
                                                                rcond=None)[0]
    resid_x = com[:, 0] - A @ np.linalg.lstsq(A, com[:, 0], rcond=None)[0]
    err = max(np.abs(resid_z).max(), np.abs(resid_x).max())
    report("flight CoM is ballistic", err < 1e-3, f"max residual {err:.2e} m")

    drift = np.abs(ang - ang[0]).max()
    report("flight angular momentum conserved", drift < 1e-3,
           f"L_y = {ang[0][1]:.3f} N.m.s, max drift {drift:.2e}")


def check_contact(phases):
    worst_cone, worst_neg = 0.0, 0.0
    for ph in phases:
        if not ph["contacts"]:
            continue
        lam = ph["lam"]
        worst_neg = min(worst_neg, lam[:, :, 2].min())
        worst_cone = max(worst_cone,
                         (np.linalg.norm(lam[:, :, :2], axis=2) - K.MU_TO * lam[:, :, 2]).max())
    report("contact forces inside the friction cone", worst_cone < 1e-6 and worst_neg > -1e-6,
           f"worst cone slack {worst_cone:.2e} N, most negative lambda_z {worst_neg:.2e} N")


def check_envelope(phases):
    """Check against torque_speed_halfplanes -- the linear relaxation actually enforced
    in program.py -- not torque_speed_bound. The two are documented (go2_backflip/constants.py,
    tools/check_envelope.py) to diverge in the regenerating quadrant, where the
    halfplanes intentionally leave torque underrated relative to the true envelope; that
    divergence would show up here as a spurious failure against a constraint the NLP was
    never asked to satisfy."""
    k_ts, tau_stall = K.torque_speed_halfplanes()
    tau_pk = K.torque_limits()
    worst = 0.0
    for ph in phases:
        qd, u = ph["x"][:, XV][:, 6:], ph["u"]
        worst = max(worst, (np.abs(u) - tau_pk).max(),
                    (u + k_ts * qd - tau_stall).max(), (-u - k_ts * qd - tau_stall).max())
    report("torques inside the enforced torque-speed halfplanes", worst < 1e-6,
           f"worst overshoot {worst:.2e} N.m")


def check_floor(plant, phases):
    """No collision geom below the floor, over ALL FOUR legs.

    The program only constrains the base and the left legs, leaning on the enforced sagittal
    symmetry for the right ones -- so this deliberately checks all four, and any leak in that
    assumption shows up here rather than silently.

    This check exists because its absence hid a large defect: a trajectory that passed 4/6
    audit checks was driving the rear thigh 69 mm and the head 72 mm through the ground. The
    optimizer had no geometry beyond four contact points, and neither did the audit.

    Sampled BETWEEN knots, not just at them, for the same reason: the constraints bind at
    knots, so a knots-only check can only ever confirm what the solver already reported. It
    hid a second defect that way -- a stance foot pinned to the floor at every knot while
    moving through it at ~0.9 m/s, bulging the reconstruction +-4.4 mm mid-interval.
    """
    ctx = plant.CreateDefaultContext()
    spec = [("base", K.COLLISION_SPHERES["base"])] + [
        (f"{leg}_{kind}", K.COLLISION_SPHERES[kind])
        for leg in K.FEET for kind in ("hip", "thigh", "calf")]
    frames = [(b, plant.GetFrameByName(b), np.ascontiguousarray(pts[:, :3].T), pts[:, 3])
              for b, pts in spec]
    worst = {True: (0.0, ""), False: (0.0, "")}          # keyed by "is a knot"
    for ph in phases:
        fine = np.sort(np.concatenate(
            [ph["t"], *(ph["t"][:-1] + f * np.diff(ph["t"]) for f in (0.25, 0.5, 0.75))]))
        knots = set(ph["t"])
        for t in fine:
            plant.SetPositions(ctx, ph["traj"].value(t - ph["t0"]).ravel()[XQ])
            for name, fr, P, r in frames:
                pen = float((r - plant.CalcPointsPositions(ctx, fr, P, plant.world_frame())[2]).max())
                at_knot = t in knots
                if pen > worst[at_knot][0]:
                    worst[at_knot] = (pen, f"{name} at t={t:.3f}s in {ph['name']}")
    # 2 mm, not zero: a stance foot's own witness sphere IS the contact, and its pin is a
    # TIGHT (1e-4) box that the solve satisfies only to its own achieved violation.
    (knot_pen, knot_where), (mid_pen, mid_where) = worst[True], worst[False]
    report("no geometry below the floor", max(knot_pen, mid_pen) < 2e-3,
           f"worst penetration {knot_pen * 1000:+.2f} mm at knots"
           + (f" ({knot_where})" if knot_pen > 0 else "")
           + f", {mid_pen * 1000:+.2f} mm between them"
           + (f" ({mid_where})" if mid_pen > 0 else ""))


def check_tuck(plant, phases):
    """The flip has to actually tuck: report peak flight I_yy about the CoM.

    Untucked flight is not a cosmetic complaint -- it is what sets the jump. At the sprawled
    I_yy = 0.66 kg.m^2 this trajectory used to hold, the same angular momentum needs 0.68 s to
    turn 4.36 rad, which is a 0.57 m CoM rise; tucked (0.45) it is 0.47 s and 0.27 m.
    """
    ph = phases[FLIGHT]
    ctx = plant.CreateDefaultContext()
    bodies = [BodyIndex(i) for i in range(1, plant.num_bodies())]
    lo, hi = TUCK_RAMP, PHASES[FLIGHT].n_knots - TUCK_RAMP
    worst = 0.0
    for x in ph["x"][lo:hi]:
        plant.SetPositions(ctx, x[XQ])
        com = plant.CalcCenterOfMassPositionInWorld(ctx)
        M = plant.CalcSpatialInertia(ctx, plant.world_frame(), bodies)
        worst = max(worst, M.Shift(com).CalcRotationalInertia().CopyToFullMatrix3()[1, 1])
    # 0.55 sits between a real tuck (0.45) and merely standing there (0.48 -> 0.66 sprawled).
    report("flight is genuinely tucked", worst < 0.55,
           f"peak I_yy over the tucked knots {worst:.4f} kg.m^2 "
           f"(tuck pose {0.452:.3f}, standing {0.484:.3f})")


def check_integration(bp, result, phases):
    """Roll each phase forward with a tight integrator on the solver's own input trajectory.

    Reported per phase, not just as a worst-of: this check and the angular-momentum one moved
    together for three mesh refinements and then stopped, and knowing WHICH phase carries the
    drift is what decides whether the next lever is more flight knots or something else
    entirely (load/absorb are still on the plain kinematic guess, never single-shot).
    """
    per_phase = []
    worst_q, worst_v = 0.0, 0.0
    for p, ph in enumerate(phases):
        u_traj = bp.dc[p].ReconstructInputTrajectory(result)
        builder = DiagramBuilder()
        plant = builder.AddSystem(make_plant())
        src = builder.AddSystem(TrajectorySource(u_traj))
        zero = builder.AddSystem(ConstantVectorSource(np.zeros(12)))
        builder.Connect(src.get_output_port(), plant.GetInputPort("applied_generalized_force"))
        builder.Connect(zero.get_output_port(), plant.get_actuation_input_port())
        diagram = builder.Build()

        sim = Simulator(diagram)
        sim.get_mutable_integrator().set_target_accuracy(1e-10)
        ctx = plant.GetMyMutableContextFromRoot(sim.get_mutable_context())
        plant.SetPositions(ctx, ph["x"][0][XQ])
        plant.SetVelocities(ctx, ph["x"][0][XV])
        sim.Initialize()
        sim.AdvanceTo(ph["t"][-1] - ph["t"][0])

        dq = float(np.abs(plant.GetPositions(ctx) - ph["x"][-1][XQ]).max())
        dv = float(np.abs(plant.GetVelocities(ctx) - ph["x"][-1][XV]).max())
        per_phase.append((ph["name"], dq, dv))
        worst_q, worst_v = max(worst_q, dq), max(worst_v, dv)
    report("collocation matches a tight integrator", worst_q < 5e-3,
           f"worst end-of-phase drift {worst_q:.2e} (q), {worst_v:.2e} (v) -- "
           + ", ".join(f"{n} {q:.1e}/{v:.1e}" for n, q, v in per_phase))


def run(bp, result, phases) -> bool:
    print("physics audit:")
    plant = bp.plant
    check_rotation(phases)
    check_ballistic(plant, phases)
    check_contact(phases)
    check_envelope(phases)
    check_floor(plant, phases)
    check_tuck(plant, phases)
    check_integration(bp, result, phases)
    failed = [n for n, ok, _ in RESULTS if not ok]
    print(f"  {len(RESULTS) - len(failed)}/{len(RESULTS)} audit checks passed"
          + (f" -- FAILED: {', '.join(failed)}" if failed else ""))
    return not failed
