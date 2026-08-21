"""Prove that Drake and MuJoCo agree on go2.xml.

Checks A-E must pass; F only reports, since the engines are expected to disagree on
contact. Check A is the regression guard for the nested-default-class trap that
silently gave the thigh joints the wrong axis.

    PYTHONPATH=/opt/drake/lib/python3.12/site-packages .venv/bin/python tools/verify_parity.py
"""

from __future__ import annotations

import sys

import mujoco
import numpy as np
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JointActuatorIndex, RevoluteJoint

import go2_constants as K

RESULTS: list[tuple[str, bool, str]] = []


def report(name: str, ok: bool, detail: str = "") -> None:
    RESULTS.append((name, ok, detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""))


def load_drake(discrete_dt: float = 0.0):
    plant = MultibodyPlant(discrete_dt)
    Parser(plant).AddModels(K.MODEL_PATH)
    plant.Finalize()
    return plant


def load_mujoco():
    return mujoco.MjModel.from_xml_path(K.MODEL_PATH)


# --- A: structural diff -----------------------------------------------------
def check_structure(plant, m) -> None:
    ok, notes = True, []

    if plant.num_positions() != m.nq or plant.num_velocities() != m.nv:
        ok = False
        notes.append(f"nq/nv {plant.num_positions()}/{plant.num_velocities()} vs {m.nq}/{m.nv}")
    if plant.num_actuators() != m.nu:
        ok = False
        notes.append(f"nu {plant.num_actuators()} vs {m.nu}")

    acts = {
        plant.get_joint_actuator(JointActuatorIndex(i)).joint().name():
        plant.get_joint_actuator(JointActuatorIndex(i))
        for i in range(plant.num_actuators())
    }

    for name in K.JOINT_NAMES:
        dj = plant.GetJointByName(name)
        assert isinstance(dj, RevoluteJoint)
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
        dof = m.jnt_dofadr[jid]

        # axis: MuJoCo stores it in the child body frame, same convention as Drake here
        if not np.allclose(dj.revolute_axis(), m.jnt_axis[jid], atol=1e-12):
            ok = False
            notes.append(f"{name} axis {dj.revolute_axis()} vs {m.jnt_axis[jid]}")
        if not np.allclose([dj.position_lower_limits()[0], dj.position_upper_limits()[0]],
                           m.jnt_range[jid], atol=1e-9):
            ok = False
            notes.append(f"{name} range")
        if not np.isclose(dj.default_damping(), m.dof_damping[dof], atol=1e-12):
            ok = False
            notes.append(f"{name} damping {dj.default_damping()} vs {m.dof_damping[dof]}")

        a = acts[name]
        refl = a.default_rotor_inertia() * a.default_gear_ratio() ** 2
        if not np.isclose(refl, m.dof_armature[dof], atol=1e-12):
            ok = False
            notes.append(f"{name} armature {refl} vs {m.dof_armature[dof]}")

        aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name.replace("_joint", ""))
        if not np.isclose(a.effort_limit(), m.actuator_forcerange[aid][1], atol=1e-9):
            ok = False
            notes.append(f"{name} effort {a.effort_limit()} vs {m.actuator_forcerange[aid][1]}")

    report("A  structure (axes/ranges/damping/armature/effort)", ok,
           "; ".join(notes[:4]) if notes else "12 joints, all fields match")


# --- B: mass and CoM --------------------------------------------------------
def check_mass(plant, m) -> None:
    ctx = plant.CreateDefaultContext()
    d = mujoco.MjData(m)
    dm, mm = plant.CalcTotalMass(ctx), float(m.body_mass.sum())
    ok = np.isclose(dm, mm, rtol=0, atol=1e-12)

    worst = 0.0
    rng = np.random.default_rng(0)
    for _ in range(20):
        legs = rng.uniform(-0.5, 0.5, 12) + K.HOME_LEGS
        q_mj = K.mj_qpos(legs, 0.4)
        d.qpos[:] = q_mj
        mujoco.mj_forward(m, d)
        mj_com = d.subtree_com[0].copy()

        plant.SetPositions(ctx, K.mj_to_drake_q(q_mj))
        worst = max(worst, np.abs(plant.CalcCenterOfMassPositionInWorld(ctx) - mj_com).max())

    ok = ok and worst < 1e-12
    report("B  total mass and CoM", ok, f"mass {dm:.9f} vs {mm:.9f} kg; max CoM err {worst:.2e} m")


# --- C: mass matrix ---------------------------------------------------------
def check_mass_matrix(plant, m) -> None:
    ctx = plant.CreateDefaultContext()
    d = mujoco.MjData(m)
    P = K.mj_to_drake_v_permutation()
    rng = np.random.default_rng(1)

    worst_full, worst_legs = 0.0, 0.0
    for i in range(20):
        legs = rng.uniform(-0.5, 0.5, 12) + K.HOME_LEGS
        # identity base orientation, where the mapping is a pure block swap
        q_mj = K.mj_qpos(legs, 0.4)
        d.qpos[:] = q_mj
        mujoco.mj_forward(m, d)
        M_mj = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, d, M_mj)

        plant.SetPositions(ctx, K.mj_to_drake_q(q_mj))
        M_dr = plant.CalcMassMatrix(ctx)

        worst_full = max(worst_full, np.abs(M_dr - P @ M_mj @ P.T).max())
        worst_legs = max(worst_legs, np.abs(M_dr[6:, 6:] - M_mj[6:, 6:]).max())

    ok = worst_full < 1e-10 and worst_legs < 1e-10
    report("C  mass matrix (18x18 permuted, + 12x12 leg block)", ok,
           f"max err {worst_full:.2e} full, {worst_legs:.2e} legs")


def check_mass_matrix_tilted(plant, m) -> None:
    """Same as C but with the base tilted, to confirm the angular-velocity frame rule."""
    ctx = plant.CreateDefaultContext()
    d = mujoco.MjData(m)
    rng = np.random.default_rng(2)

    worst = 0.0
    for _ in range(20):
        quat = rng.normal(size=4)
        quat /= np.linalg.norm(quat)
        legs = rng.uniform(-0.5, 0.5, 12) + K.HOME_LEGS
        q_mj = K.mj_qpos(legs, 0.4, quat)

        d.qpos[:] = q_mj
        mujoco.mj_forward(m, d)
        M_mj = np.zeros((m.nv, m.nv))
        mujoco.mj_fullM(m, d, M_mj)

        plant.SetPositions(ctx, K.mj_to_drake_q(q_mj))
        M_dr = plant.CalcMassMatrix(ctx)

        # T maps drake v -> mujoco v, built column-wise from the documented rule
        R = K.quat_wxyz_to_R(quat)
        T = np.zeros((18, 18))
        T[0:3, 3:6] = np.eye(3)   # mj v_world <- drake v_world
        T[3:6, 0:3] = R.T         # mj w_body  <- drake w_world
        T[6:, 6:] = np.eye(12)
        worst = max(worst, np.abs(M_dr - T.T @ M_mj @ T).max())

    report("C' mass matrix at tilted base (frame convention)", worst < 1e-10,
           f"max err {worst:.2e}")


# --- D: inverse dynamics ----------------------------------------------------
def check_inverse_dynamics(plant, m) -> None:
    """Sharpest rigid-body test: sensitive to axes, inertias and armature at once.

    Acceleration converts with the same map as velocity: d/dt(R w_body) = R wdot_body,
    since the Rdot term carries w_body x w_body = 0. Generalized forces convert with the
    transpose of that map, which is exactly drake_to_mj_v.
    """
    from pydrake.multibody.tree import MultibodyForces

    ctx = plant.CreateDefaultContext()
    m_nc = load_mujoco()
    m_nc.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONTACT
    d = mujoco.MjData(m_nc)
    rng = np.random.default_rng(3)

    worst = 0.0
    for _ in range(100):
        quat = rng.normal(size=4)
        quat /= np.linalg.norm(quat)
        legs = rng.uniform(-0.4, 0.4, 12) + K.HOME_LEGS
        q_mj = K.mj_qpos(legs, 0.4, quat)
        v_mj = rng.uniform(-1.5, 1.5, 18)
        a_mj = rng.uniform(-4.0, 4.0, 18)

        d.qpos[:], d.qvel[:], d.qacc[:] = q_mj, v_mj, a_mj
        mujoco.mj_inverse(m_nc, d)
        tau_mj = d.qfrc_inverse.copy()

        plant.SetPositions(ctx, K.mj_to_drake_q(q_mj))
        plant.SetVelocities(ctx, K.mj_to_drake_v(v_mj, quat))

        # Gravity and joint damping, matching what mj_inverse folds into qfrc_inverse.
        forces = MultibodyForces(plant)
        plant.CalcForceElementsContribution(ctx, forces)
        tau_dr = plant.CalcInverseDynamics(ctx, K.mj_to_drake_v(a_mj, quat), forces)

        worst = max(worst, np.abs(K.drake_to_mj_v(tau_dr, quat) - tau_mj).max())

    report("D  inverse dynamics (100 random q,v,vdot)", worst < 1e-8, f"max err {worst:.2e} N.m")


# --- E: open-loop torque tape, contacts off ---------------------------------
def check_torque_tape(m) -> None:
    plant = load_drake(0.0)
    ctx = plant.CreateDefaultContext()

    # Disable ALL constraints, not just contact: MuJoCo enforces joint position limits
    # as constraints while Drake's continuous plant ignores them, so leaving limits on
    # makes the two diverge for a reason that has nothing to do with the model.
    m_nc = load_mujoco()
    m_nc.opt.disableflags |= mujoco.mjtDisableBit.mjDSBL_CONSTRAINT
    d = mujoco.MjData(m_nc)

    dt, steps = 5e-4, 1000  # 0.5 s, about one flight phase
    m_nc.opt.timestep = dt
    m_nc.opt.integrator = mujoco.mjtIntegrator.mjINT_RK4

    rng = np.random.default_rng(4)
    freqs = 0.7 + 0.3 * rng.random(12)
    t = np.arange(steps) * dt
    tape = 0.3 * K.torque_limits() * np.sin(2 * np.pi * freqs * t[:, None])

    q0 = K.mj_qpos(K.TUCK_LEGS, 1.0)
    d.qpos[:], d.qvel[:] = q0, 0.0

    from pydrake.systems.analysis import Simulator
    sim = Simulator(plant, ctx.Clone())
    sim.get_mutable_integrator().set_target_accuracy(1e-10)
    sctx = sim.get_mutable_context()
    plant.SetPositions(sctx, K.mj_to_drake_q(q0))
    plant.SetVelocities(sctx, np.zeros(18))
    port = plant.get_actuation_input_port().FixValue(sctx, tape[0])
    sim.Initialize()

    worst_base, worst_joint = 0.0, 0.0
    for k in range(steps):
        d.ctrl[:] = tape[k]
        mujoco.mj_step(m_nc, d)

        port.GetMutableData().set_value(tape[k])
        sim.AdvanceTo((k + 1) * dt)

        if (k + 1) % 100 == 0:
            q_dr = K.drake_to_mj_q(plant.GetPositions(sctx))
            worst_base = max(worst_base, np.abs(q_dr[:3] - d.qpos[:3]).max())
            worst_joint = max(worst_joint, np.abs(q_dr[7:] - d.qpos[7:]).max())

    ok = worst_base < 1e-4 and worst_joint < 1e-4
    report("E  torque-tape rollout, 0.5 s contact-free", ok,
           f"max base err {worst_base:.2e} m, joint err {worst_joint:.2e} rad")


# --- F: contact divergence (report only) ------------------------------------
def check_contact_divergence() -> None:
    """TODO-10: quantify this once the TO produces a landing trajectory."""
    print("[INFO] F  contact divergence: expected and not tuned toward zero.")
    print("          Drake sees condim=3 / mu=0.8 point contact; MuJoCo uses condim=6,")
    print("          elliptic cone, solimp compliance. Quantify once the TO produces a")
    print("          landing trajectory to hand the RL stage a robustness budget.")


def main() -> int:
    plant, m = load_drake(0.0), load_mujoco()
    print(f"Drake nq/nv/nu = {plant.num_positions()}/{plant.num_velocities()}/{plant.num_actuators()}")
    print(f"MuJoCo nq/nv/nu = {m.nq}/{m.nv}/{m.nu}\n")

    check_structure(plant, m)
    check_mass(plant, m)
    check_mass_matrix(plant, m)
    check_mass_matrix_tilted(plant, m)
    check_inverse_dynamics(plant, m)
    check_torque_tape(m)
    check_contact_divergence()

    failed = [n for n, ok, _ in RESULTS if not ok]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} checks passed")
    if failed:
        print("FAILED: " + ", ".join(failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
