"""The backflip NLP: one DirectCollocation per contact phase over a shared program.

Contact forces reach the dynamics through the `applied_generalized_force` input port. That
port is vector-valued (18) where `applied_spatial_force` is abstract, and gravity plus joint
damping are force elements already inside EvalTimeDerivatives, so the port carries exactly
B*u + sum J_i^T lambda_i -- the term we want as a decision variable. `actuation` is fixed to
zero in the prototype context, since DirectCollocation pins every input it was not given.

Hermite-Simpson first-order-holds that port, so at collocation points the contact term is the
average of the endpoint J^T lambda rather than J(q_col)^T lambda_col, and the contact
constraints below bind at knots only. solve_backflip.py measures what that costs.
"""

from __future__ import annotations


import numpy as np
from pydrake.math import eq
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.planning import DirectCollocation
from pydrake.solvers import MathematicalProgram

from go2_backflip import constants as K
from schedule import PHASES, FLIGHT, IMPACT

NQ, NV, NU, NX = 19, 18, 12, 37
XQ, XV = slice(0, NQ), slice(NQ, NX)          # blocks of a 37-state
P_ANKLE_COL = K.P_ANKLE.reshape(3, 1)
Z_W = np.array([0.0, 0.0, 1.0])

# Base + the two sagittally-distinct legs. The right legs are redundant: the NLP pins the hips
# and mirrors thigh/calf to within TIGHT, so their world z differs from the left by tens of
# microns. audit.py checks all four anyway, so a leak in that assumption cannot pass silently.
CLEARANCE_BODIES = ([("base", K.COLLISION_SPHERES["base"])]
                    + [(f"{leg}_{kind}", K.COLLISION_SPHERES[kind])
                       for leg in ("FL", "RL") for kind in ("hip", "thigh", "calf")])

FOOT_CLEARANCE = 0.02
# Floor clearance demanded of every witness sphere that is NOT a foot. The foot sphere keeps a
# bound of exactly 0 -- it IS the contact, so any margin would contradict the stance pin, and
# its own swing clearance is FOOT_CLEARANCE above. For the rest, 0 is the wrong bound: it is
# satisfied by grazing, and the optimizer grazes, because nothing rewards a gap. The result
# passed every check and still looked wrong in replay -- the rear knee scraped the floor for
# three knots at launch (-0.2 mm) and the head touched down at 0.1 mm before the feet did.
# 10 mm is ~50x the between-knot reconstruction error (0.05 mm) and ~4x the worst knot
# violation IPOPT leaves at the tolerance it converges to here, so it is a gap that survives
# both, and it is the margin the hardware stage will need against tracking error anyway.
BODY_CLEARANCE = 0.010
BASE_Z_MIN = 0.20        # torso half-diagonal is 0.196 m, so this clears the floor at any pitch
WY_MAX = 20.0            # rad/s; also keeps the per-step half-angle far from the pi that aliases
# Flight knots at each end left free to fold in / extend out again -- a FRACTION of the phase,
# not a fixed count. guess.py ramps the tuck over the first and last 20% of flight time and
# requires this to be at least that as a fraction of the phase, or the guess starts outside the
# hard tuck window at exactly the knots that window covers. It was 6, which is 0.23 at the 26
# knots it was written for but only 0.12 at 50 -- and at 50 the solve stalled, inf_pr flat for
# 45 iterations.
#
# The fraction is of INTERVALS, not knots. What has to stay fixed as the mesh changes is the
# span of the phase the window covers, and a ramp of r knots spans r/(n-1) of the phase. Using
# r/n instead moves the window: bisecting 50 -> 99 gave ceil(0.23*99) = 23, a window of
# 0.2347..0.7653 against the source's 0.2449..0.7551, so the first and last hard-tucked knots
# landed half an interval outside where the source solution had ramped to. In a warm start that
# showed up as the two largest collocation defects in the whole guess (18.02 at segments 74/75
# and 9.90 at 22/23, both straddling exactly those knots). The interval form gives 24 there,
# which is 2*12 -- the bisection of the source window, exactly. It reproduces every value the
# knot form ever produced: 6 at 26 knots, 12 at 50, 18 at 76.
TUCK_FRAC = 12 / 49     # the 50-knot window, which is the 26-knot window to within a knot
TUCK_RAMP = round(TUCK_FRAC * (PHASES[FLIGHT].n_knots - 1))
X_LAND_MAX = 0.15
LAMBDA_SCALE = 200.0

# Held-across-many-knots equalities (foot pin, quaternion norm, leg mirror) are each exactly
# implied by the SAME quantity's own collocation defect once it holds at consecutive knots --
# confirmed by a finite-difference Jacobian rank check (SVD nullity dropped from 764 to 50
# after loosening exactly these three families). SNOPT tolerated the resulting rank-deficient
# KKT system by quietly reporting the whole problem infeasible; IPOPT refuses outright
# ("too few degrees of freedom"). A tight box in place of the exact equality removes the row
# from the interior-point method's equality-rank requirement while leaving it numerically
# indistinguishable from exact.
TIGHT = 1e-4

# Same idea, for the stance foot's VELOCITY. Pinning only the position, and only at knots, let
# the foot sit exactly on the floor at every knot while moving through it at ~0.9 m/s: the
# Hermite cubic between two pinned knots then bulges by h*|v|/4, measured at +-4.4 mm on a
# 20 ms load interval, alternating sign knot to knot. Nothing saw it -- the audit sampled knots
# too -- until tools/check_npz.py looked at the resampled output. 1e-3 m/s caps that bulge at
# 5 um. Boxed, not exact, for the LICQ reason above -- it IS the pin's own derivative, so the
# two are dependent by construction, and a box keeps that dependence out of the equality rank.
#
# The VERTICAL component only, and of the sphere's CENTRE. Vertical is where the bulge is (a
# horizontal wiggle between two pinned knots leaves no mark on the floor), and it is the one
# component every other contact condition here already agrees on: the touchdown impulse
# constrains the MATERIAL point at the contact, whose vertical velocity is the centre's, while
# horizontally the two differ by R*omega_y -- the rolling term. Constraining that too would
# assert the sphere neither rolls nor slips, i.e. that the calf does not pitch, which is the
# opposite of what a push-off does.
NO_SLIP = 1e-3

# Half-width of the flight angular-momentum box, when BackflipProgram(amom=...) enables it.
# In flight the only external force is gravity; it acts AT the centre of mass, so its moment
# about the CoM is identically zero and L_com is EXACTLY conserved by the true dynamics. The
# transcription does not conserve it -- audit.py measures the drift, and it is the check this
# problem has never passed. This constraint asserts the invariant directly.
#
# It is an addition, not a relaxation: every continuous solution satisfies it, so the feasible
# set of the underlying problem is unchanged and only the DISCRETE solution set shrinks -- to
# the physically consistent members of it. That also costs a diagnostic, and the trade is
# deliberate: once this is on, audit's "flight angular momentum conserved" is enforced rather
# than emergent, and "collocation matches a tight integrator" becomes the only independent
# readout of transcription error left. Judge a run with amom on by THAT check.
#
# The y-component only. L_x and L_z are zero for a sagittal motion and are already implied by
# _add_symmetry, so constraining them too would add rows that other rows already span --
# the LICQ failure mode this file's TIGHT comment is about.
AMOM_BOX = 1e-4

# The left/right thigh/calf mirror, and the DOFs a sagittal motion holds at zero (quat_x,
# quat_z, base y, the hips), and the unit-quaternion box. All three were loosened on
# 2026-09-03 to get them out of the active set, and all three are back at TIGHT, because the
# thing they bought turned out not to be worth having. See STATUS: loosening them takes the
# 400-iteration feasibility pass from ~69 to 0.0025 and the best restart to 0.0004, but that
# number is not the trajectory. At 0.0004 the audit is 7/11 -- WORSE than the 9/11 the shipped
# 0.0495 trajectory scores -- with flight integration drift of 1.66e-01 against 9.4e-03, and a
# pitch reversal. The solve is finding a spurious discrete solution: the defects are satisfied
# almost exactly AT the collocation points while the cubic rings between them. A looser box is
# a larger feasible set, and most of what the violation number gained was that.
MIRROR = TIGHT
SAGITTAL_BOX = TIGHT
QUAT_BOX = TIGHT




# Generous, physically-loose upper bounds -- go2 weighs ~149 N total, so these are 10-25x a
# static single-foot share, never expected to bind, just there to give IPOPT's interior-point
# method a finite barrier region on every variable (see the TIGHT comment above).
LAMBDA_MAX = 2000.0        # N, per foot GRF
IMPULSE_MAX = 200.0        # N.s, per foot touchdown impulse


def make_plant() -> MultibodyPlant:
    plant = MultibodyPlant(0.0)
    Parser(plant).AddModels(K.MODEL_PATH)
    plant.Finalize()
    return plant


class _Kin:
    """One plant/context pair per scalar type, owned by exactly one constraint.

    SNOPT evaluates constraints out of order, so contexts are never shared between them.
    """

    def __init__(self, plant, plant_ad, feet):
        self.f, self.ad = plant, plant_ad
        self.cf, self.ca = plant.CreateDefaultContext(), plant_ad.CreateDefaultContext()
        self.frames = {
            "f": [plant.GetFrameByName(f"{f}_calf") for f in feet],
            "ad": [plant_ad.GetFrameByName(f"{f}_calf") for f in feet],
        }

    def at(self, q):
        ad = q.dtype == object
        plant, ctx = (self.ad, self.ca) if ad else (self.f, self.cf)
        plant.SetPositions(ctx, q)
        return plant, ctx, self.frames["ad" if ad else "f"]

    # The foot is a 22 mm SPHERE. K.P_FOOT -- its bottom, body-fixed -- is the point touching a
    # flat floor only while the calf is vertical, and the calf is tilted 51.6 deg at HOME and
    # past 90 deg at launch. Pinning it buries the sphere by R*(1 - cos tilt): 8.3 mm standing,
    # 28 mm at launch, measured on the trajectory this replaced. Both methods below therefore
    # work off the sphere instead.

    @staticmethod
    def pos(plant, ctx, frame):
        """World position of the foot sphere's lowest point -- the centre, R below in world z.

        Exact for a flat floor and cheap: the offset is along world z whatever the calf does,
        so no rotation matrix is needed here.
        """
        centre = plant.CalcPointsPositions(ctx, frame, P_ANKLE_COL, plant.world_frame()).ravel()
        return centre - K.R_FOOT * Z_W

    @staticmethod
    def jac_centre(plant, ctx, frame):
        """Jacobian of the sphere CENTRE: exactly d/dt of pos(), since pos() offsets the centre
        by a constant world vector.

        This, not jac() below, is what the stance velocity constraint has to use. jac() is the
        MATERIAL point at the contact, and holding that still is the no-slip condition of a
        ROLLING sphere, whose contact patch travels along the floor -- which contradicts a
        foothold pinned in place. Together the two would force the calf's pitch rate to zero at
        every stance knot, and the calf pitches through most of a radian on the push-off.
        """
        return plant.CalcJacobianTranslationalVelocity(
            ctx, JacobianWrtVariable.kV, frame, P_ANKLE_COL,
            plant.world_frame(), plant.world_frame())

    @staticmethod
    def jac(plant, ctx, frame):
        """Translational Jacobian of the calf's MATERIAL point currently at the contact.

        Not the centre's: the contact force acts R below it, and a friction force through the
        centre instead leaves a spurious moment of |f_x|*R ~ 2.5 N.m about the knee, ~5% of its
        45 N.m limit. The material point is q-dependent (it slides around the sphere as the
        calf rotates), which is fine -- at any instant it is a rigidly-attached point, so this
        is exactly the Jacobian the applied force needs, and autodiff differentiates through
        the q-dependence correctly.
        """
        R_CW = plant.CalcRelativeRotationMatrix(ctx, frame, plant.world_frame()).matrix()
        p_contact = K.P_ANKLE - K.R_FOOT * (R_CW @ Z_W)
        return plant.CalcJacobianTranslationalVelocity(
            ctx, JacobianWrtVariable.kV, frame, p_contact.reshape(3, 1),
            plant.world_frame(), plant.world_frame())


class _Floor:
    """Floor clearance for every collision geom, over one plant/context pair.

    One witness sphere per geom feature (tools/clearance_points.py generates and verifies the
    table): the union contains the geoms, so `p_z(q) >= r` for every witness point keeps the
    whole robot above the floor. Same one-context-per-constraint rule as _Kin.
    """

    def __init__(self, plant, plant_ad):
        self.f, self.ad = plant, plant_ad
        self.cf, self.ca = plant.CreateDefaultContext(), plant_ad.CreateDefaultContext()
        self.spec = [(plant.GetFrameByName(b), plant_ad.GetFrameByName(b),
                      np.ascontiguousarray(pts[:, :3].T), pts[:, 3])
                     for b, pts in CLEARANCE_BODIES]
        self.radii = np.concatenate([r for *_, r in self.spec])
        # Identified by geometry, not by index, so a regenerated table cannot silently move it.
        foot = [np.logical_and(np.all(np.isclose(pts[:, :3], K.P_ANKLE), axis=1),
                               np.isclose(pts[:, 3], K.R_FOOT))
                for _, pts in CLEARANCE_BODIES]
        self.margin = np.concatenate([np.where(f, 0.0, BODY_CLEARANCE) for f in foot])

    def slack(self, q):
        ad = q.dtype == object
        plant, ctx = (self.ad, self.ca) if ad else (self.f, self.cf)
        plant.SetPositions(ctx, q)
        z = [plant.CalcPointsPositions(ctx, fr_ad if ad else fr_f, P, plant.world_frame())[2]
             for fr_f, fr_ad, P, _ in self.spec]
        return np.concatenate(z) - self.radii


class _Momentum:
    """Angular momentum about the instantaneous CoM, over one plant/context pair per
    constraint -- same one-context-per-constraint rule as _Kin and _Floor."""

    def __init__(self, plant, plant_ad):
        self.f, self.ad = plant, plant_ad
        self.cf, self.ca = plant.CreateDefaultContext(), plant_ad.CreateDefaultContext()

    def ly(self, q, v):
        ad = q.dtype == object
        plant, ctx = (self.ad, self.ca) if ad else (self.f, self.cf)
        plant.SetPositions(ctx, q)
        plant.SetVelocities(ctx, v)
        com = plant.CalcCenterOfMassPositionInWorld(ctx)
        return plant.CalcSpatialMomentumInWorldAboutPoint(ctx, com).rotational()[1]


class BackflipProgram:
    def __init__(self, amom: float | None = None, amom_mode: str = "chain"):
        self.plant = make_plant()
        self.plant_ad = self.plant.ToAutoDiffXd()
        self.B = self.plant.MakeActuationMatrix()
        self.prog = MathematicalProgram()

        ctx = self.plant.CreateDefaultContext()
        self.plant.get_actuation_input_port().FixValue(ctx, np.zeros(NU))
        port = self.plant.GetInputPort("applied_generalized_force").get_index()

        self.dc, self.u, self.lam, self.foothold = [], [], [], []
        for p in PHASES:
            dc = DirectCollocation(self.plant, ctx, p.n_knots, p.h_min, p.h_max,
                                   input_port_index=port, prog=self.prog)
            # A uniform step per phase, and it has to stay that way despite pointing straight
            # at the drift. The measurement says the mesh is wrong: at viol 2.5e-3 the flight
            # phase integrates to 1e-6..5e-5 in q on every interior segment but 9.7e-4 and
            # 5.3e-4 on segments 0 and 24 -- the two ends, where the legs fold in after
            # takeoff and extend again for the landing -- so uniform steps starve exactly the
            # two intervals that carry the end-of-phase drift. Letting the steps vary is the
            # textbook answer and it was tried: it is much worse here, 0.0025 -> 0.0706 over
            # the same 400-iteration pass with everything else held, and a second run of it
            # stalled outright at 11.8. Per-interval steps make every defect bilinear in its
            # own h, and this NLP cannot afford that. If the end-of-phase drift needs fixing,
            # the lever is a finer mesh where it is needed -- more knots, or splitting flight
            # at the tuck ramps so each piece keeps its own uniform step -- not free steps.
            dc.AddEqualTimeIntervalsConstraints()
            self.dc.append(dc)
            self.u.append(self.prog.NewContinuousVariables(p.n_knots, NU, f"u_{p.name}"))
            nc = len(p.contacts)
            self.lam.append(
                self.prog.NewContinuousVariables(p.n_knots, 3 * nc, f"lam_{p.name}")
                .reshape(p.n_knots, nc, 3)
                if nc else np.zeros((p.n_knots, 0, 3)))
            self.foothold.append(
                self.prog.NewContinuousVariables(nc, 3, f"pf_{p.name}")
                if nc else np.zeros((0, 3)))
        self.impulse = self.prog.NewContinuousVariables(4, 3, "impulse")

        self._add_coupling()
        self._add_contact()
        self._add_actuator_limits()
        self._add_symmetry()
        self._add_clearance()
        self._add_body_clearance()
        self._add_stitching()
        self._add_boundary()
        if amom is not None:
            self._add_flight_momentum(amom, amom_mode)
        self._scale()

    # --- helpers ---------------------------------------------------------
    def state(self, p, k):
        return self.dc[p].state(k)

    def _kin(self, feet):
        return _Kin(self.plant, self.plant_ad, feet)

    # --- dynamics coupling -----------------------------------------------
    def _add_coupling(self):
        """input(k) == B u_k + sum_i J_i(q_k)^T lambda_i_k."""
        for p, ph in enumerate(PHASES):
            for k in range(ph.n_knots):
                tau, u = self.dc[p].input(k), self.u[p][k]
                if not ph.contacts:
                    self.prog.AddLinearConstraint(eq(tau, self.B @ u))
                    continue
                kin, nc = self._kin(ph.contacts), len(ph.contacts)

                def f(z, kin=kin, nc=nc, B=self.B):
                    q, tau, u, lam = z[:NQ], z[NQ:NQ + NV], z[NQ + NV:NQ + NV + NU], \
                        z[NQ + NV + NU:].reshape(nc, 3)
                    plant, ctx, frames = kin.at(q)
                    r = tau - B @ u
                    for i, fr in enumerate(frames):
                        r = r - kin.jac(plant, ctx, fr).T @ lam[i]
                    return r

                self.prog.AddConstraint(
                    f, np.zeros(NV), np.zeros(NV),
                    np.concatenate([self.state(p, k)[XQ], tau, u, self.lam[p][k].ravel()]),
                    description=f"coupling_{ph.name}_{k}")

    # --- contact ----------------------------------------------------------
    def _add_contact(self):
        for p, ph in enumerate(PHASES):
            if not ph.contacts:
                continue
            nc = len(ph.contacts)
            for f in range(nc):
                self.prog.AddBoundingBoxConstraint(0.0, 0.0, self.foothold[p][f, 2])
                self.prog.AddBoundingBoxConstraint(-1.5, 1.5, self.foothold[p][f, :2])

            for k in range(ph.n_knots):
                kin = self._kin(ph.contacts)

                def g(z, kin=kin, nc=nc):
                    q, pf = z[:NQ], z[NQ:].reshape(nc, 3)
                    plant, ctx, frames = kin.at(q)
                    return np.concatenate([kin.pos(plant, ctx, fr) - pf[i]
                                           for i, fr in enumerate(frames)])

                eps = TIGHT * np.ones(3 * nc)
                self.prog.AddConstraint(
                    g, -eps, eps,
                    np.concatenate([self.state(p, k)[XQ], self.foothold[p].ravel()]),
                    description=f"pin_{ph.name}_{k}")

                # ...and it is not moving there: the exact time derivative of that same pin,
                # imposed at the same knots. Without it the pin holds only pointwise, and the
                # cubic between two pinned knots is free to bulge through the floor.
                #
                # Except where something else already says it, in which case this is a
                # duplicate row in the KKT system rather than a constraint: the trajectory
                # starts and ends at rest (_add_boundary pins all of v), a phase's first knot
                # IS the previous phase's last knot and inherits its no-slip through the
                # state-continuity equality, and absorb's first knot is covered by the
                # touchdown impulse's own post-impact no-slip rows (J @ v_plus == 0, whose
                # vertical component is exactly this).
                if k not in self._noslip_implied(p, ph):
                    kin_v = self._kin(ph.contacts)

                    def gv(z, kin=kin_v, nc=nc):
                        q, v = z[:NQ], z[NQ:]
                        plant, ctx, frames = kin.at(q)
                        return np.array(
                            [(kin.jac_centre(plant, ctx, fr) @ v)[2] for fr in frames])

                    eps_v = NO_SLIP * np.ones(nc)
                    self.prog.AddConstraint(
                        gv, -eps_v, eps_v, self.state(p, k),
                        description=f"noslip_{ph.name}_{k}")

                lam = self.lam[p][k]
                # lambda_y == 0: the motion is sagittal, so a lateral GRF would be a pure
                # internal squeeze reacted by the hips -- a null space, not a mechanism.
                self.prog.AddBoundingBoxConstraint(0.0, 0.0, lam[:, 1])
                self.prog.AddBoundingBoxConstraint(0.0, LAMBDA_MAX, lam[:, 2])
                for i in range(nc):
                    self.prog.AddLinearConstraint(lam[i, 0] <= K.MU_TO * lam[i, 2])
                    self.prog.AddLinearConstraint(-lam[i, 0] <= K.MU_TO * lam[i, 2])

            # Release smoothly: feet that swing in the next phase carry no load at the switch.
            if p + 1 < len(PHASES):
                for i, foot in enumerate(ph.contacts):
                    if foot not in PHASES[p + 1].contacts:
                        self.prog.AddBoundingBoxConstraint(
                            0.0, 0.0, self.lam[p][ph.n_knots - 1][i, 2])

    @staticmethod
    def _noslip_implied(p, ph):
        """Knots of phase p where the stance no-slip row is already imposed by something else."""
        implied = set()
        if p == 0:
            implied.add(0)                              # x0 pins all of v to zero
        else:
            implied.add(0)                              # continuity, or the impact's own rows
        if p == len(PHASES) - 1:
            implied.add(ph.n_knots - 1)                 # xf pins all of v to zero
        return implied

    # --- actuator envelope, joint and speed limits -------------------------
    def _add_actuator_limits(self):
        k_ts, tau_stall = K.torque_speed_halfplanes()
        tau_pk, w_max = K.torque_limits(), K.speed_limits()
        q_lo = self.plant.GetPositionLowerLimits()[7:]
        q_hi = self.plant.GetPositionUpperLimits()[7:]

        for p, ph in enumerate(PHASES):
            for k in range(ph.n_knots):
                x, u = self.state(p, k), self.u[p][k]
                q, v, qd = x[XQ], x[XV], x[XV][6:]
                self.prog.AddBoundingBoxConstraint(q_lo, q_hi, q[7:])
                self.prog.AddBoundingBoxConstraint(-w_max, w_max, qd)
                self.prog.AddBoundingBoxConstraint(-tau_pk, tau_pk, u)
                self.prog.AddLinearConstraint(u + k_ts * qd, -np.inf * tau_stall, tau_stall)
                self.prog.AddLinearConstraint(-u - k_ts * qd, -np.inf * tau_stall, tau_stall)
                # Generous, physically-loose bounds on the otherwise-unbounded base DOFs.
                # IPOPT's own exit message calls out unbounded variables as a known cause of
                # poor interior-point behaviour; these are wide enough to never bind at the
                # true solution, just there to anchor the barrier method.
                self.prog.AddBoundingBoxConstraint(-1.01, 1.01, [q[0], q[2]])
                self.prog.AddBoundingBoxConstraint(-2.0, 2.0, q[4])
                self.prog.AddBoundingBoxConstraint(0.0, 1.5, q[6])
                self.prog.AddBoundingBoxConstraint(-30.0, 30.0, [v[3], v[5]])

    # --- sagittal symmetry -------------------------------------------------
    def _add_symmetry(self):
        """Mirror through the x-z plane. Hip flips sign; thigh and calf do not.

        Position-level pins only, not velocity too: pinning q[1]=0 (a quaternion component)
        together with v[0]=0 (its kinematically-conjugate angular velocity, qdot=f(q)v) makes
        the active-constraint Jacobian at that knot rank-deficient -- the collocation defect's
        own row there is already a linear combination of the two pins. SNOPT reports the
        problem outright infeasible (info=13) even though the pinned point is trivially
        feasible, confirmed empirically on an isolated single-phase stance-hold case that
        solves cleanly once the velocity-level pin is dropped. Same mechanism, same fix, for
        the hip joints (qdot=v exactly for a revolute joint) and the thigh/calf mirror.

        q[5] (base y) and v[4] (its world-frame translational velocity, qdot=v exactly, no
        rotation involved) are the same conjugate pair and had the same redundancy -- an
        FD-Jacobian/SVD check on the shrunk-flight problem found 88% of a 335-row nullity
        traced to this function, concentrated on exactly this pin. Dropped for the same reason.

        Separately -- bug 2's mechanism, not bug 1's: q[1]/q[3]/q[5] and the hip positions are
        pinned to the same constant at EVERY knot of EVERY phase, so once that pin holds at two
        consecutive knots the collocation defect for that same state already implies it at the
        next one. Same re-run of the check (after the v[4] fix above) found 86% of the
        remaining nullity on exactly these two families -- TIGHT-boxed for the same reason the
        foot-pin/quat-norm/leg-mirror families were.
        """
        for p, ph in enumerate(PHASES):
            for k in range(ph.n_knots):
                x, u = self.state(p, k), self.u[p][k]
                q, v = x[XQ], x[XV]
                self.prog.AddBoundingBoxConstraint(
                    -SAGITTAL_BOX, SAGITTAL_BOX, [q[1], q[3], q[5]])
                self.prog.AddBoundingBoxConstraint(-WY_MAX, 0.0, v[1])
                for j in K.HIP_IDX:
                    self.prog.AddBoundingBoxConstraint(-SAGITTAL_BOX, SAGITTAL_BOX, q[7 + j])
                for a, b in ((0, 3), (6, 9)):          # FL/FR and RL/RR leg blocks
                    self.prog.AddLinearConstraint(u[a] + u[b] == 0.0)
                    for d in (1, 2):
                        diff = q[7 + a + d] - q[7 + b + d]
                        self.prog.AddLinearConstraint(-MIRROR <= diff)
                        self.prog.AddLinearConstraint(diff <= MIRROR)
                        self.prog.AddLinearConstraint(u[a + d] == u[b + d])
                for i, j in self._mirror_contact_pairs(ph):
                    self.prog.AddLinearConstraint(self.lam[p][k][i, 0] == self.lam[p][k][j, 0])
                    self.prog.AddLinearConstraint(self.lam[p][k][i, 2] == self.lam[p][k][j, 2])

        # Flight only, as intended: the box was grown around a HOME<->TUCK sweep and is
        # already violated by the launch pose's extended rear leg, which is nowhere near
        # self-collision anyway (stance postures aren't the folded configurations this
        # guards against).
        lo = [K.TUCK_BOX[n][0] for n in ("thigh_front", "calf_front", "thigh_rear", "calf_rear")]
        hi = [K.TUCK_BOX[n][1] for n in ("thigh_front", "calf_front", "thigh_rear", "calf_rear")]
        for k in range(PHASES[FLIGHT].n_knots):
            q = self.state(FLIGHT, k)[XQ]
            self.prog.AddBoundingBoxConstraint(lo, hi, [q[8], q[9], q[14], q[15]])

        # ...and, over the middle of the flight, actually TUCK. TUCK_BOX only asserts "not
        # self-colliding", which near-full extension satisfies, so on its own it let the
        # optimizer hold both knees against their straightest edge for the entire flip:
        # I_yy = 0.66 kg.m^2, worse than standing (0.48), bought with a 0.57 m CoM rise. Inside
        # this window I_yy is 0.45-0.46, which is a 0.27 m rise for the same angular momentum.
        # Strictly inside TUCK_BOX, so the two never conflict. The first and last TUCK_RAMP
        # knots stay free: the rear leg leaves the ground extended and has to fold, and both
        # legs have to come back out for the landing.
        t_lo, t_hi = K.FLIGHT_TUCK["thigh"]
        c_lo, c_hi = K.FLIGHT_TUCK["calf"]
        for k in range(TUCK_RAMP, PHASES[FLIGHT].n_knots - TUCK_RAMP):
            q = self.state(FLIGHT, k)[XQ]
            self.prog.AddBoundingBoxConstraint([t_lo, c_lo, t_lo, c_lo], [t_hi, c_hi, t_hi, c_hi],
                                               [q[8], q[9], q[14], q[15]])

    @staticmethod
    def _mirror_contact_pairs(ph):
        idx = {f: i for i, f in enumerate(ph.contacts)}
        return [(idx[a], idx[b]) for a, b in K.MIRROR_LEGS if a in idx and b in idx]

    # --- swing clearance ---------------------------------------------------
    def _add_clearance(self):
        for p, ph in enumerate(PHASES):
            swing = [f for f in K.FEET if f not in ph.contacts]
            if not swing:
                continue
            for k in range(ph.n_knots):
                # A boundary knot shares its q with the neighbouring stance phase, where the
                # foot is on the ground, so only interior knots can demand real clearance.
                lb = 0.0 if k in (0, ph.n_knots - 1) else FOOT_CLEARANCE
                kin = self._kin(swing)

                def c(q, kin=kin):
                    plant, ctx, frames = kin.at(q)
                    return np.array([kin.pos(plant, ctx, fr)[2] for fr in frames])

                self.prog.AddConstraint(c, np.full(len(swing), lb), np.full(len(swing), np.inf),
                                        self.state(p, k)[XQ], description=f"clear_{ph.name}_{k}")
            if p == FLIGHT:
                for k in range(ph.n_knots):
                    self.prog.AddBoundingBoxConstraint(
                        BASE_Z_MIN, np.inf, self.state(p, k)[XQ][6])

    # --- floor clearance for everything that is not a foot ------------------
    def _add_body_clearance(self):
        """No collision geom below the floor, at any knot of any phase.

        Without this the program's only geometric knowledge of the robot is four contact
        points, and the rest of it sweeps straight through the ground -- the trajectory this
        replaced put the rear thigh 69 mm and the head 72 mm under the floor while passing
        every audit check, because the audit could not see it either.

        The bound is BODY_CLEARANCE for every witness sphere except the foot's, which keeps
        exactly 0: a stance foot's own witness sphere IS the contact, so any positive margin
        there would contradict the foot pin.
        """
        lb = _Floor(self.plant, self.plant_ad).margin
        for p, ph in enumerate(PHASES):
            for k in range(ph.n_knots):
                floor = _Floor(self.plant, self.plant_ad)
                self.prog.AddConstraint(
                    floor.slack, lb, np.full(lb.size, np.inf), self.state(p, k)[XQ],
                    description=f"floor_{ph.name}_{k}")

    # --- phase stitching and the touchdown impulse -------------------------
    def _add_stitching(self):
        for p in range(len(PHASES) - 1):
            a, b = self.dc[p].final_state(), self.dc[p + 1].initial_state()
            if p + 1 == IMPACT:
                self.prog.AddLinearConstraint(eq(b[XQ], a[XQ]))
            else:
                self.prog.AddLinearConstraint(eq(b, a))

        q = self.dc[IMPACT].initial_state()[XQ]
        v_minus = self.dc[IMPACT - 1].final_state()[XV]
        v_plus = self.dc[IMPACT].initial_state()[XV]
        feet = PHASES[IMPACT].contacts
        kin, nc = self._kin(feet), len(feet)

        def impact(z, kin=kin, nc=nc):
            q, vm, vp = z[:NQ], z[NQ:NQ + NV], z[NQ + NV:NQ + 2 * NV]
            imp = z[NQ + 2 * NV:].reshape(nc, 3)
            plant, ctx, frames = kin.at(q)
            r = plant.CalcMassMatrix(ctx) @ (vp - vm)
            touch = []
            for i, fr in enumerate(frames):
                J = kin.jac(plant, ctx, fr)
                r = r - J.T @ imp[i]
                touch.append(J @ vp)
            return np.concatenate([r] + touch)

        n_eq = NV + 3 * nc
        self.prog.AddConstraint(
            impact, np.zeros(n_eq), np.zeros(n_eq),
            np.concatenate([q, v_minus, v_plus, self.impulse.ravel()]), description="impact")

        self.prog.AddBoundingBoxConstraint(0.0, 0.0, self.impulse[:, 1])
        self.prog.AddBoundingBoxConstraint(0.0, IMPULSE_MAX, self.impulse[:, 2])
        for i in range(4):
            self.prog.AddLinearConstraint(self.impulse[i, 0] <= K.MU_TO * self.impulse[i, 2])
            self.prog.AddLinearConstraint(-self.impulse[i, 0] <= K.MU_TO * self.impulse[i, 2])
        for i, j in self._mirror_contact_pairs(PHASES[IMPACT]):
            self.prog.AddLinearConstraint(self.impulse[i, 0] == self.impulse[j, 0])
            self.prog.AddLinearConstraint(self.impulse[i, 2] == self.impulse[j, 2])

    # --- boundary conditions ------------------------------------------------
    def _add_boundary(self):
        x0 = self.dc[0].initial_state()
        q0 = K.mj_to_drake_q(K.mj_qpos(K.HOME_LEGS, K.STAND_BASE_HEIGHT))
        self.prog.AddLinearConstraint(eq(x0[XQ], q0))
        self.prog.AddLinearConstraint(eq(x0[XV], np.zeros(NV)))

        xf = self.dc[-1].final_state()
        # One full backward revolution. With quat_x = quat_z = 0 the base quaternion is
        # [cos(t/2), 0, sin(t/2), 0], so -2*pi lands on the NEGATED identity -- distinct from
        # +identity, which is what makes the winding number a single equality.
        self.prog.AddLinearConstraint(eq(xf[XQ][:4], [-1.0, 0.0, 0.0, 0.0]))
        self.prog.AddLinearConstraint(eq(xf[XQ][7:], K.HOME_LEGS))
        self.prog.AddLinearConstraint(eq(xf[XV], np.zeros(NV)))
        self.prog.AddBoundingBoxConstraint(-X_LAND_MAX, X_LAND_MAX, xf[XQ][4])

        # Unit norm at every knot. quat_x = quat_z = 0 already, so this is a 2-term circle.
        for p, ph in enumerate(PHASES):
            for k in range(ph.n_knots):
                q = self.state(p, k)[XQ]
                norm = q[0] ** 2 + q[2] ** 2
                self.prog.AddConstraint(1.0 - QUAT_BOX <= norm)
                self.prog.AddConstraint(norm <= 1.0 + QUAT_BOX)

    def _add_flight_momentum(self, box: float, mode: str = "chain"):
        """Bound the flight phase's angular-momentum drift. See AMOM_BOX.

        Two forms, and the difference is the Jacobian's sparsity, not the physics:

        `anchor` compares every knot against knot 0, which bounds exactly the quantity the
        audit reports -- and measured much worse. Direct collocation's Jacobian is block
        banded (each row touches one interval), and 49 rows that each reach back to knot 0
        put a dense column block through the middle of it. Seeded on the shipped point under
        `nlp_scaling_method=none`, anchor took inf_pr to 3.95 by iteration 135 where the same
        run without these rows was at 2.87e-03.

        `chain` (default) bounds |L_y(k+1) - L_y(k)| instead, so every row stays inside one
        interval and the banded structure survives. The cost is that the drift the audit
        measures can accumulate to (n_knots - 1) * box in the worst case -- 49 * box -- so the
        box has to be set that much tighter. At the 1e-4 default that worst case is 4.9e-3,
        above the audit's 1e-3 bound, so `chain` wants ~1e-5. The accumulation is a worst case
        that requires every interval to drift the same direction; measure, do not assume it.
        """
        ph = PHASES[FLIGHT]
        pairs = ([(0, k) for k in range(1, ph.n_knots)] if mode == "anchor"
                 else [(k, k + 1) for k in range(ph.n_knots - 1)])
        for a, b in pairs:
            mom = _Momentum(self.plant, self.plant_ad)

            def d(z, mom=mom):
                x, y = z[:NX], z[NX:]
                return np.array([mom.ly(y[XQ], y[XV]) - mom.ly(x[XQ], x[XV])])

            self.prog.AddConstraint(
                d, [-box], [box],
                np.concatenate([self.state(FLIGHT, a), self.state(FLIGHT, b)]),
                description=f"amom_{a}_{b}")

    def _scale(self):
        for p, ph in enumerate(PHASES):
            if not ph.contacts:
                continue
            for var in self.lam[p].ravel():
                self.prog.SetVariableScaling(var, LAMBDA_SCALE)
        for var in self.impulse.ravel():
            self.prog.SetVariableScaling(var, LAMBDA_SCALE * 0.1)

    # --- cost ----------------------------------------------------------------
    def add_cost(self, w_torque=1.0, w_rate=0.1, w_time=1.0, w_tuck=0.5, scale=1.0):
        """Performance costs, scaled by `scale`; the symmetry cost is NOT scaled.

        `scale` exists because at full weight this pass is not affordable. Measured: the
        feasibility pass reaches viol 0.0025 and the costed pass that starts from it ends at
        0.7349 with cost 11.5 -- it walks straight off the feasible manifold, and the restart
        loop then begins 300x worse than the point it was handed. None of these four terms is
        needed for a valid trajectory (the tuck is guaranteed by the hard FLIGHT_TUCK window,
        not by w_tuck); they buy smoothness and effort for the RL stage that consumes this.
        There is NO symmetry term here, despite what this docstring said until 2026-09-06:
        the four above are all of it, so `scale=0` leaves the objective identically flat --
        which is the condition --proximal exists to fix, not a symmetry-only pass.
        """
        w_torque, w_rate = scale * w_torque, scale * w_rate
        w_time, w_tuck = scale * w_time, scale * w_tuck
        inv = 1.0 / K.torque_limits() ** 2
        for p, ph in enumerate(PHASES):
            # Per-interval steps now, so the running cost has to integrate against the actual
            # mesh rather than one scalar h: weight each knot by half of each interval it
            # touches (trapezoid), and price duration as the true sum of the steps.
            hs = [self.dc[p].time_step(k)[0] for k in range(ph.n_knots - 1)]
            eff = 0.0
            for k in range(ph.n_knots):
                dt = (0.0 + (0.5 * hs[k - 1] if k > 0 else 0.0)
                      + (0.5 * hs[k] if k < ph.n_knots - 1 else 0.0))
                eff = eff + dt * sum(float(inv[j]) * self.u[p][k][j] ** 2 for j in range(NU))
            self.prog.AddCost(w_torque * eff)
            rate = sum((self.u[p][k + 1][j] - self.u[p][k][j]) ** 2 * float(inv[j])
                       for k in range(ph.n_knots - 1) for j in range(NU))
            self.prog.AddCost(w_rate * rate)
            self.prog.AddCost(w_time * sum(hs))

        # Pull the tucked knots toward TUCK_LEGS rather than letting them sit anywhere in the
        # window. The hard window guarantees the inertia; this makes the solver settle inside
        # it instead of riding an edge, and keeps the fold smooth. TUCK_LEGS is the deepest
        # pose tools/tuck_box.py cleared, so pulling toward it is pulling toward minimum I_yy.
        tgt = (K.TUCK_LEGS[1], K.TUCK_LEGS[2])
        tuck = sum((self.state(FLIGHT, k)[XQ][j] - t) ** 2
                   for k in range(TUCK_RAMP, PHASES[FLIGHT].n_knots - TUCK_RAMP)
                   for j, t in ((8, tgt[0]), (9, tgt[1]), (14, tgt[0]), (15, tgt[1])))
        self.prog.AddCost(w_tuck * tuck)
