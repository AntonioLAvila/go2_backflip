"""Analytic initial guess. Not feasible -- just in the right basin.

Phase 0 holds the standing pose -- all four feet are pinned there, so any base motion would
need per-leg IK to stay consistent, and the solver does not need the help. Phase 1 pivots the
whole robot rigidly about the rear foothold, which keeps the rear feet planted and lifts the
front pair for free, while the rear legs extend into the launch. Phase 2 is a ballistic base
with a linear pitch ramp to -2*pi and a tuck in the middle. Phase 3 holds the landing pose.
"""

from __future__ import annotations

import numpy as np
from pydrake.multibody.tree import JacobianWrtVariable, MultibodyForces

import go2_constants as K
from schedule import PHASES

G = 9.81
DURATION = (0.15, 0.12, 0.55, 0.20)
# Swept over the reachable poses: this is where the base is still high (0.355 m) and the rear
# foothold already leads the CoM by 0.16 m, which is the moment arm that drives the rotation.
THETA_LAUNCH = np.deg2rad(-50.0)
X_LAND = -0.05
LAUNCH_LEGS_REAR = np.array([0.0, 0.6, -1.3])
TUCK_LEG = np.array([0.0, 2.2, -2.7])
HOME_LEG = np.array([0.0, 0.9, -1.8])


def quat_pitch(theta):
    return np.array([np.cos(theta / 2), 0.0, np.sin(theta / 2), 0.0])


def legs(front, rear):
    return np.concatenate([front, front, rear, rear])


def smoothstep(x):
    """3x^2 - 2x^3, clamped. Zero derivative at both x=0 and x=1, so pieces glued at
    those endpoints are velocity-continuous, not just position-continuous."""
    x = np.clip(x, 0.0, 1.0)
    return x * x * (3 - 2 * x)


def blend(t, t0, t1, a, b):
    return a + (b - a) * smoothstep((t - t0) / (t1 - t0))


class Guess:
    def __init__(self, plant):
        self.plant = plant
        self.ctx = plant.CreateDefaultContext()
        self.frames = {f: plant.GetFrameByName(f"{f}_calf") for f in K.FEET}
        self.P = K.P_FOOT.reshape(3, 1)
        self.mass = plant.CalcTotalMass(self.ctx)
        q0 = K.mj_to_drake_q(K.mj_qpos(K.HOME_LEGS, K.STAND_BASE_HEIGHT))
        self.home_feet = {f: self.foot(q0, f) for f in K.FEET}

    def foot(self, q, f):
        self.plant.SetPositions(self.ctx, q)
        return self.plant.CalcPointsPositions(
            self.ctx, self.frames[f], self.P, self.plant.world_frame()).ravel()

    def place(self, theta, leg_q, pivot, target):
        """q with the base placed so that `pivot` foot sits exactly on `target`."""
        q = np.concatenate([quat_pitch(theta), np.zeros(3), leg_q])
        q[4:7] = target - self.foot(q, pivot)
        return q

    # --- per-phase position paths -----------------------------------------
    # Launch and flight move the base through a genuinely nonlinear path (place() re-solves
    # the whole kinematic chain every step). Differencing that on only 12-26 knots is noisy
    # enough that the acceleration estimate implies centripetal terms no achievable contact
    # force can supply -- the solved-for lambda_z came out NEGATIVE by over 100 N at some
    # knots. Sampling FINE times denser and downsampling after differentiating removes it.
    FINE = 25

    def _launch_path(self, t):
        pivot, target = "RL", self.home_feet["RL"]
        return self.place(THETA_LAUNCH * t,
                          legs(HOME_LEG + (TUCK_LEG - HOME_LEG) * t,
                               HOME_LEG + (LAUNCH_LEGS_REAR - HOME_LEG) * t),
                          pivot, target)

    RAMP = 0.3

    def _flight_path(self, t, z0, x0, vz, tf):
        tt = t * tf
        # Legs must continue from where launch left them (front already at TUCK_LEG, rear at
        # LAUNCH_LEGS_REAR), not reset to HOME_LEG at t=0 -- that reset was a real position
        # discontinuity across the phase seam and, once differentiated, the dominant source
        # of the huge spurious accelerations seen here. Front stays tucked until the last
        # ramp, when it extends for landing; rear first catches up to the tuck, holds, then
        # extends alongside the front. Every segment boundary is a smoothstep endpoint, so
        # the whole path is velocity-continuous.
        r = self.RAMP
        if t < r:
            front, rear = TUCK_LEG, blend(t, 0.0, r, LAUNCH_LEGS_REAR, TUCK_LEG)
        elif t < 1 - r:
            front, rear = TUCK_LEG, TUCK_LEG
        else:
            front = rear = blend(t, 1 - r, 1.0, TUCK_LEG, HOME_LEG)
        q = np.concatenate([quat_pitch(THETA_LAUNCH + (-2 * np.pi - THETA_LAUNCH) * t),
                            np.zeros(3), legs(front, rear)])
        q[4] = x0 + (X_LAND - x0) * t
        q[6] = z0 + vz * tt - 0.5 * G * tt ** 2
        return q

    def positions(self):
        """Knot-resolution q, for the boundary conditions and footholds."""
        n = [p.n_knots for p in PHASES]
        pivot, target = "RL", self.home_feet["RL"]
        stand = self.place(0.0, legs(HOME_LEG, HOME_LEG), pivot, target)
        p0 = [stand.copy() for _ in range(n[0])]
        p1 = [self._launch_path(t) for t in np.linspace(0, 1, n[1])]
        z0, x0 = p1[-1][6], p1[-1][4]
        vz = (K.STAND_BASE_HEIGHT - z0 + 0.5 * G * DURATION[2] ** 2) / DURATION[2]
        p2 = [self._flight_path(t, z0, x0, vz, DURATION[2]) for t in np.linspace(0, 1, n[2])]
        p3 = [p2[-1].copy() for _ in range(n[3])]
        return [np.array(p) for p in (p0, p1, p2, p3)]

    def _fine_kinematics(self, path_fn, n_knots, duration):
        """q, v, vdot at knot resolution, differentiated on a FINE-times-denser grid."""
        nf = (n_knots - 1) * self.FINE + 1
        t_fine = np.linspace(0, duration, nf)
        q_fine = np.array([path_fn(s) for s in np.linspace(0, 1, nf)])
        qd_fine = np.gradient(q_fine, t_fine, axis=0)
        v_fine = np.array([self._map_qdot(q_fine[i], qd_fine[i]) for i in range(nf)])
        vdot_fine = np.gradient(v_fine, t_fine, axis=0)
        idx = np.arange(n_knots) * self.FINE
        return q_fine[idx], v_fine[idx], vdot_fine[idx]

    # --- velocities, forces, torques ---------------------------------------
    def build(self):
        pivot, target = "RL", self.home_feet["RL"]
        stand = self.place(0.0, legs(HOME_LEG, HOME_LEG), pivot, target)
        Q = self.positions()
        out = []
        for p, ph in enumerate(PHASES):
            t = np.linspace(0, DURATION[p], ph.n_knots)
            if p == 1:
                q, v, vdot = self._fine_kinematics(self._launch_path, ph.n_knots, DURATION[p])
            elif p == 2:
                z0, x0 = Q[1][-1][6], Q[1][-1][4]
                vz = (K.STAND_BASE_HEIGHT - z0 + 0.5 * G * DURATION[2] ** 2) / DURATION[2]
                path_fn = lambda s: self._flight_path(s, z0, x0, vz, DURATION[2])
                q, v, vdot = self._fine_kinematics(path_fn, ph.n_knots, DURATION[p])
            else:
                q = Q[p]
                v = np.zeros((ph.n_knots, 18))
                vdot = np.zeros((ph.n_knots, 18))

            nc = len(ph.contacts)
            u = np.zeros((ph.n_knots, 12))
            lam = np.zeros((ph.n_knots, nc, 3))
            for k in range(ph.n_knots):
                u[k], lam[k] = self._wrench(q[k], v[k], vdot[k], ph.contacts)

            # The last knot of a phase that drops feet is pinned lam_z=0 for those feet (the
            # "release" constraint in program.py). Match it here too, or the guess starts
            # this fixed variable at the wrong value.
            if p + 1 < len(PHASES):
                leaving = [i for i, f in enumerate(ph.contacts) if f not in PHASES[p + 1].contacts]
                staying = [i for i in range(nc) if i not in leaving]
                if leaving and staying:
                    k = ph.n_knots - 1
                    lam[k][leaving] = 0.0
                    lam[k][staying, 2] += self.mass * G / len(staying)
                    u[k] = self._wrench_given_lam(q[k], v[k], vdot[k], ph.contacts, lam[k])

            gen = np.array([self._gen_force(q[k], u[k], ph.contacts, lam[k])
                            for k in range(ph.n_knots)])
            out.append(dict(t=t, x=np.hstack([q, v]), u=u, lam=lam, gen=gen))
        return out

    def _map_qdot(self, q, qd):
        self.plant.SetPositions(self.ctx, q)
        return self.plant.MapQDotToVelocity(self.ctx, qd)

    def _jac(self, foot):
        return self.plant.CalcJacobianTranslationalVelocity(
            self.ctx, JacobianWrtVariable.kV, self.frames[foot], self.P,
            self.plant.world_frame(), self.plant.world_frame())

    def _contact_force(self, contacts, lam):
        return sum((self._jac(f).T @ lam[i] for i, f in enumerate(contacts)), np.zeros(18))

    def _wrench(self, q, v, vdot, contacts):
        """(u, lam) realizing vdot, via the manipulator equation, with lam kept physical.

        B has zero rows on the 6 base DOFs, so u alone can never balance them: fixing lam at
        a uniform mg/n_contacts and solving u alone (the first pass at this) leaves the base
        rows unsolved whenever the CoM is not centered over the footholds, which free-drops
        the base to several rad/s^2 of spurious acceleration in what should be a static
        guess. Solving [u; lam] together fixes that -- floating-base plus ground contact can
        realize an arbitrary body wrench -- but for the launch phase's aggressive rigid-pivot
        kinematics, the exact lstsq solution wants over 100 N of TENSION at the pivot foot:
        the prescribed motion is more aggressive than gravity plus a push-only contact can
        produce, which is a property of that guess path, not of the robot. Clipping lam to
        the friction cone and re-closing only the actuated (leg) rows exactly leaves a base-
        row residual instead -- the same kind SNOPT's feasibility pass already has to absorb
        elsewhere -- rather than handing it a guess that starts outside the cone entirely.
        """
        self.plant.SetPositions(self.ctx, q)
        self.plant.SetVelocities(self.ctx, v)
        forces = MultibodyForces(self.plant)
        self.plant.CalcForceElementsContribution(self.ctx, forces)
        rhs = self.plant.CalcInverseDynamics(self.ctx, vdot, forces)
        B = self.plant.MakeActuationMatrix()
        nc = len(contacts)
        if not nc:
            return np.linalg.lstsq(B, rhs, rcond=None)[0], np.zeros((0, 3))

        Jt = np.hstack([self._jac(f).T for f in contacts])
        sol = np.linalg.lstsq(np.hstack([B, Jt]), rhs, rcond=None)[0]
        lam = sol[12:].reshape(nc, 3)
        lam[:, 2] = np.clip(lam[:, 2], 0.05 * self.mass * G / nc, None)
        xy = np.linalg.norm(lam[:, :2], axis=1)
        over = xy > K.MU_TO * lam[:, 2]
        lam[over, :2] *= (K.MU_TO * lam[over, 2] / xy[over])[:, None]

        u = np.linalg.lstsq(B[6:], rhs[6:] - (Jt[6:] @ lam.ravel()), rcond=None)[0]
        return u, lam

    def _wrench_given_lam(self, q, v, vdot, contacts, lam):
        """u alone, for a fully prescribed lam (leg rows only -- see _wrench)."""
        self.plant.SetPositions(self.ctx, q)
        self.plant.SetVelocities(self.ctx, v)
        forces = MultibodyForces(self.plant)
        self.plant.CalcForceElementsContribution(self.ctx, forces)
        rhs = self.plant.CalcInverseDynamics(self.ctx, vdot, forces)
        B = self.plant.MakeActuationMatrix()
        Jt = np.hstack([self._jac(f).T for f in contacts])
        return np.linalg.lstsq(B[6:], rhs[6:] - (Jt[6:] @ lam.ravel()), rcond=None)[0]

    def _gen_force(self, q, u, contacts, lam):
        self.plant.SetPositions(self.ctx, q)
        return self.plant.MakeActuationMatrix() @ u + self._contact_force(contacts, lam)

    def footholds(self):
        out = []
        for ph in PHASES:
            out.append(np.array([self.home_feet[f] * [1, 1, 0] for f in ph.contacts])
                       if ph.contacts else np.zeros((0, 3)))
        return out
