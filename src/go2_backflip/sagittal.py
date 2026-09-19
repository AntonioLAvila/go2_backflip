"""The Go2 restricted to its sagittal-symmetric manifold, as CasADi expressions.

A backflip is a planar motion of a mirror-symmetric robot. Solving it on the full 18-DOF
floating-base model means carrying 11 degrees of freedom the motion never uses and then pinning
them back out with constraints -- and every one of those pins is a row that the dynamics defects
already half-imply, which is where this project's LICQ trouble, its TIGHT boxes and its
never-converging IPOPT runs all came from. Here the symmetry is a *parametrisation* instead:

    q = [x, z, theta,  thigh_f, calf_f,  thigh_r, calf_r]            (7)

with the left and right legs of each pair sharing one coordinate. There is no quaternion, no
mirror constraint and nothing to pin: qdot = v exactly, and the full rotation is theta = -2*pi.

Nothing is re-derived by hand. The kinematic tree, masses, inertias, armatures and damping are
read from MuJoCo's compiled go2.xml, the Lagrangian of all 13 bodies is assembled symbolically,
and CasADi differentiates it. The hip-abduction coordinates are carried symbolically as well
(mirrored, so they too respect the symmetry) and evaluated at zero, which is what yields the
hip torque needed to HOLD the hips at zero under load -- a real actuator limit at launch that a
hand-written planar model would not see. tools/verify_sagittal.py proves the result against
mj_inverse / mj_jac to ~1e-12.

Conventions: R_y(+theta) tips the nose down, so a backflip is theta -> -2*pi (same as the rest
of the repo). Forces are PER FOOT and torques PER MOTOR; the factor 2 for the mirrored pair is
applied in here and nowhere else.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache

import casadi as ca
import mujoco
import numpy as np

from . import constants as K

NQ = 7                                   # x z theta | tf cf | tr cr
NJ = 4                                   # actuated sagittal coordinates (per-motor torques)
PAIRS = ("front", "rear")
_LEFT = {"front": "FL", "rear": "RL"}
G = 9.81

# reduced 9-vector [x z th | hf tf cf | hr tr cr]  ->  the 12 MuJoCo joints
_EMBED = np.zeros((12, 9))
for _leg, _cols, _sgn in (("FL", (3, 4, 5), 1), ("FR", (3, 4, 5), -1),
                          ("RL", (6, 7, 8), 1), ("RR", (6, 7, 8), -1)):
    s = K.LEG_SLICE[_leg].start
    _EMBED[s, _cols[0]] = _sgn           # hip abduction mirrors
    _EMBED[s + 1, _cols[1]] = 1.0
    _EMBED[s + 2, _cols[2]] = 1.0
_PLANAR = [0, 1, 2, 4, 5, 7, 8]          # the 7 live coordinates inside the 9
_HIPS = [3, 6]


def _skew(a):
    return np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])


def _axis_rot(axis, ang):
    k = _skew(axis)
    return ca.SX.eye(3) + ca.sin(ang) * k + (1 - ca.cos(ang)) * (k @ k)


def _quat_R(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    return R.reshape(3, 3)


@dataclass(frozen=True)
class Sagittal:
    """CasADi Functions over the 7-coordinate model. All inputs/outputs are column vectors."""
    dyn: ca.Function        # (q,v,a,u,lam_f,lam_r) -> 7 residual of the equations of motion
    hip: ca.Function        # (q,v,a,lam_f,lam_r)   -> 2  torque per hip motor holding h = 0
    mass: ca.Function       # q -> 7x7 M
    bias: ca.Function       # (q,v) -> 7  C(q,v)v + g(q) + damping, i.e. dyn at a=u=lam=0
    act: np.ndarray         # 7x4, generalised force per unit PER-MOTOR torque (the 2 is here)
    foot: ca.Function       # q -> [gx_f, gz_f, gx_r, gz_r]: rolling-sphere contact coordinates
    foot_J: ca.Function     # q -> 4x7 Jacobian of `foot` (== material contact point x,z)
    foot_acc: ca.Function   # (q,v,a) -> 4  d2/dt2 of `foot`
    spheres: ca.Function    # q -> z - r of every witness sphere (floor clearance slack)
    sphere_names: tuple
    com: ca.Function        # q -> CoM [x, z]
    momentum: ca.Function   # (q,v) -> [px, pz, L_y about the CoM]
    inertia: ca.Function    # q -> I_yy about the CoM with joints locked
    total_mass: float
    q_lo: np.ndarray
    q_hi: np.ndarray


def embed_qpos(q):
    """Planar q (7) -> MuJoCo qpos (19)."""
    x, z, th = q[0], q[1], q[2]
    legs = _EMBED[:, _PLANAR] @ np.asarray(q, float)
    return np.concatenate([[x, 0.0, z], [np.cos(th / 2), 0.0, np.sin(th / 2), 0.0], legs])


def embed_qvel(v):
    """Planar v (7) -> MuJoCo qvel (18). Pitch is about y in world AND body, so no frame issue."""
    legs = _EMBED[:, _PLANAR] @ np.asarray(v, float)
    return np.concatenate([[v[0], 0.0, v[1]], [0.0, v[2], 0.0], legs])


def embed_ctrl(u, u_hip):
    """Per-motor sagittal torques (4) + hip holding torques (2) -> MuJoCo ctrl (12)."""
    tf, cf, tr, cr = u
    hf, hr = u_hip
    return np.array([hf, tf, cf, -hf, tf, cf, hr, tr, cr, -hr, tr, cr])


@lru_cache(maxsize=1)
def build() -> Sagittal:
    m = mujoco.MjModel.from_xml_path(K.MODEL_PATH)
    q9, v9, a9 = ca.SX.sym("q", 9), ca.SX.sym("v", 9), ca.SX.sym("a", 9)
    qj, vj = ca.mtimes(_EMBED, q9), ca.mtimes(_EMBED, v9)
    ey = np.array([0.0, 1.0, 0.0])

    # --- forward kinematics of every body, world frame -------------------------------------
    R, p, w = {}, {}, {}                                  # rotation, origin, angular velocity
    base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base")
    R[base] = _axis_rot(ey, q9[2])
    p[base] = ca.vertcat(q9[0], 0, q9[1])
    w[base] = ca.SX(ey) * v9[2]
    for b in range(1, m.nbody):
        if b == base:
            continue
        par = int(m.body_parentid[b])
        j = int(m.body_jntadr[b])
        assert m.body_jntnum[b] == 1 and m.jnt_type[j] == mujoco.mjtJoint.mjJNT_HINGE
        assert not np.any(m.jnt_pos[j]), "joint anchors off the body origin are not handled"
        k = int(m.jnt_qposadr[j]) - 7
        Rfix = R[par] @ _quat_R(m.body_quat[b])
        R[b] = Rfix @ _axis_rot(m.jnt_axis[j], qj[k])
        p[b] = p[par] + R[par] @ m.body_pos[b]
        w[b] = w[par] + (Rfix @ m.jnt_axis[j]) * vj[k]

    # --- Lagrangian -------------------------------------------------------------------------
    T, V, com_num = 0, 0, 0
    Iyy_terms = []
    for b in range(1, m.nbody):
        pc = p[b] + R[b] @ m.body_ipos[b]
        vc = ca.jtimes(pc, q9, v9)
        Ri = R[b] @ _quat_R(m.body_iquat[b])
        wl = Ri.T @ w[b]
        T += 0.5 * m.body_mass[b] * ca.dot(vc, vc) + 0.5 * ca.dot(wl, m.body_inertia[b] * wl)
        V += m.body_mass[b] * G * pc[2]
        com_num += m.body_mass[b] * pc
        Iyy_terms.append((m.body_mass[b], pc, Ri, m.body_inertia[b], w[b]))
    T += 0.5 * ca.dot(vj, m.dof_armature[6:] * vj)
    mass = float(m.body_mass.sum())
    com = com_num / mass

    L = T - V
    mom = ca.gradient(L, v9)
    eom = ca.jtimes(mom, q9, v9) + ca.jtimes(mom, v9, a9) - ca.gradient(L, q9)
    eom += _EMBED.T @ (m.dof_damping[6:] * vj)            # passive damping, moved to the LHS

    # --- contacts: a sphere rolling on the floor -------------------------------------------
    # Holonomic in the plane: the centre stays R above the floor and advances by R per radian of
    # calf pitch. Its Jacobian is exactly the MATERIAL contact point's, which is where the
    # ground force acts, so one J serves both the constraint and the J^T lambda term.
    g, Jc = [], {}
    r_c = np.array([0.0, 0.0, -K.R_FOOT])
    for pair in PAIRS:
        calf = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"{_LEFT[pair]}_calf")
        centre = p[calf] + R[calf] @ K.P_ANKLE
        v_mat = ca.jtimes(centre, q9, v9) + ca.cross(w[calf], r_c)
        Jc[pair] = ca.jacobian(v_mat, v9)                 # 3x9
        pitch = q9[2] + (q9[4] + q9[5] if pair == "front" else q9[7] + q9[8])
        g += [centre[0] - K.R_FOOT * pitch, centre[2] - K.R_FOOT]
    g = ca.vertcat(*g)

    lam_f, lam_r = ca.SX.sym("lam_f", 2), ca.SX.sym("lam_r", 2)      # per foot [x, z]
    contact = 0
    for pair, lam in (("front", lam_f), ("rear", lam_r)):
        contact += 2.0 * Jc[pair].T @ ca.vertcat(lam[0], 0, lam[1])  # left + mirrored right

    u = ca.SX.sym("u", NJ)
    act9 = np.zeros((9, NJ))
    act9[[4, 5, 7, 8], range(NJ)] = 2.0
    resid9 = eom - contact - act9 @ u

    # --- restrict to the plane: hips at zero, not moving --------------------------------------
    q, v, a = ca.SX.sym("q", NQ), ca.SX.sym("v", NQ), ca.SX.sym("a", NQ)
    lift = np.zeros((9, NQ))
    lift[_PLANAR, range(NQ)] = 1.0

    def plane(e):
        return ca.substitute([e], [q9, v9, a9], [lift @ q, lift @ v, lift @ a])[0]

    resid = plane(resid9)
    dyn = ca.Function("dyn", [q, v, a, u, lam_f, lam_r], [resid[_PLANAR]])
    hip = ca.Function("hip", [q, v, a, lam_f, lam_r], [resid[_HIPS] / 2.0])
    zero = [ca.SX.zeros(NQ), ca.SX.zeros(NJ), ca.SX.zeros(2), ca.SX.zeros(2)]
    bias = ca.Function("bias", [q, v], [dyn(q, v, zero[0], zero[1], zero[2], zero[3])])
    Mq = ca.Function("mass", [q], [ca.jacobian(dyn(q, v, a, u, lam_f, lam_r), a)])

    gp = plane(g)
    foot = ca.Function("foot", [q], [gp])
    Jg = ca.jacobian(gp, q)
    foot_J = ca.Function("foot_J", [q], [Jg])
    gd = Jg @ v
    foot_acc = ca.Function("foot_acc", [q, v, a], [Jg @ a + ca.jtimes(gd, q, v)])

    # --- floor clearance witness spheres (left legs + base; the right side mirrors) -----------
    slack, names = [], []
    bodies = [("base", "base")] + [(f"{leg}_{kind}", kind)
                                   for leg in ("FL", "RL") for kind in ("hip", "thigh", "calf")]
    for body, kind in bodies:
        b = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, body)
        for i, (sx, sy, sz, r) in enumerate(K.COLLISION_SPHERES[kind]):
            slack.append((p[b] + R[b] @ np.array([sx, sy, sz]))[2] - r)
            names.append(f"{body}[{i}]")
    spheres = ca.Function("spheres", [q], [plane(ca.vertcat(*slack))])

    # --- momentum bookkeeping for the audit ----------------------------------------------------
    comp = plane(com)
    comv = ca.jtimes(comp, q, v)
    Ly, Iyy = 0, 0
    for mb, pc, Ri, Idiag, wb in Iyy_terms:
        pc_, Ri_, wb_ = plane(pc), plane(Ri), plane(wb)
        vc_ = ca.jtimes(pc_, q, v)
        d = pc_ - comp
        Iw = Ri_ @ ca.diag(Idiag) @ Ri_.T
        Ly += mb * (d[2] * (vc_[0] - comv[0]) - d[0] * (vc_[2] - comv[2])) + (Iw @ wb_)[1]
        Iyy += mb * (d[0] ** 2 + d[2] ** 2) + Iw[1, 1]
    momentum = ca.Function("momentum", [q, v], [ca.vertcat(mass * comv[0], mass * comv[2], Ly)])

    lo = np.array([-np.inf, -np.inf, -np.inf] + [m.jnt_range[1 + i, 0] for i in (1, 2, 7, 8)])
    hi = np.array([np.inf, np.inf, np.inf] + [m.jnt_range[1 + i, 1] for i in (1, 2, 7, 8)])
    return Sagittal(dyn=dyn, hip=hip, mass=Mq, bias=bias, act=act9[_PLANAR], foot=foot,
                    foot_J=foot_J, foot_acc=foot_acc, spheres=spheres,
                    sphere_names=tuple(names),
                    com=ca.Function("com", [q], [comp[[0, 2]]]), momentum=momentum,
                    inertia=ca.Function("inertia", [q], [Iyy]), total_mass=mass,
                    q_lo=lo, q_hi=hi)


HOME = np.array([0.0, K.STAND_BASE_HEIGHT, 0.0, 0.9, -1.8, 0.9, -1.8])
