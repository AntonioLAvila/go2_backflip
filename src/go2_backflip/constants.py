"""Shared Go2 constants and the Drake <-> MuJoCo state mapping.

Single source of truth for both the Drake TO and the mjlab RL config, so the two
stages cannot drift apart. Poses here mirror the <keyframe> block in go2.xml.

The actuator envelope lives ONLY here -- the MJCF deliberately carries just the flat box.
torque_speed_bound() is the checker; torque_speed_halfplanes() is the same envelope in the
form an NLP can hold. Both read the same constants, so they cannot drift.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
MODEL_PATH = str(REPO_ROOT / "go2_mjcf" / "go2.xml")

# The foot is a SPHERE, not a point: radius 22 mm centred at P_ANKLE in the calf frame.
# Drake drops the <site> tags, so the geom is the only handle on the foot.
#
# P_FOOT (the sphere's bottom, body-fixed) is the contact point ONLY while the calf is
# vertical, and the calf is never vertical here -- it is already tilted 51.6 deg at HOME and
# past 90 deg at the launch pose. Treating P_FOOT as the contact point buries the sphere
# R*(1 - cos tilt) below the floor: 8.3 mm just standing, 28 mm at launch. The point actually
# touching a flat floor is the one directly under the centre, R below it in WORLD z, so
# contact geometry must be written about P_ANKLE (see program.py's _Kin).
R_FOOT = 0.022
P_ANKLE = np.array([-0.002, 0.0, -0.213])
P_FOOT = P_ANKLE - np.array([0.0, 0.0, R_FOOT])
FEET = ["FL", "FR", "RL", "RR"]

# Drake sees mu=0.8 on the foot sphere; the TO runs leaner on purpose, as sim-to-real margin.
MU_TO = 0.6

# Joint order as authored in go2.xml. Both engines report this same order.
JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]
ACTUATOR_NAMES = [n.replace("_joint", "") for n in JOINT_NAMES]

# --- Actuator data ----------------------------------------------------------
PEAK_TORQUE = {"hip": 23.7, "thigh": 23.7, "calf": 45.43}      # N.m
MAX_SPEED = {"hip": 30.1, "thigh": 30.1, "calf": 15.7}         # rad/s

KNEE_EXTRA_REDUCTION = PEAK_TORQUE["calf"] / PEAK_TORQUE["hip"]

# Reflected rotor inertia, I_rotor * N^2. This is exactly the MJCF `armature` value and
# exactly Drake's reflected inertia (Drake sets gear_ratio=1, rotor_inertia=armature).
# Source: Unitree's own mjlab RL config (unitree_rl_mjlab). Keep in sync with the
# `armature` attributes in go2.xml -- verify_parity Check A fails if they drift.
ARMATURE = {"hip": 0.01, "thigh": 0.01, "calf": 0.02}

GEAR_RATIO = {"hip": None, "thigh": None, "calf": None}
ROTOR_INERTIA = {"hip": None, "thigh": None, "calf": None}
CONTINUOUS_TORQUE = {"hip": None, "thigh": None, "calf": None} # thermal budget only

DAMPING = 0.1 # best conservative guess, not measured TODO


def joint_kind(name: str) -> str:
    """'FL_calf_joint' -> 'calf'."""
    return name.split("_")[1]


def torque_limits() -> np.ndarray:
    """Peak torque per actuator, in JOINT_NAMES order."""
    return np.array([PEAK_TORQUE[joint_kind(n)] for n in JOINT_NAMES])


def speed_limits() -> np.ndarray:
    return np.array([MAX_SPEED[joint_kind(n)] for n in JOINT_NAMES])


# Corner of the two-segment envelope, as a fraction of no-load speed.
# Digitized from the GO-M8010-6 "FOC Controlled Characteristic Graph" (24 V, p.6 of the
# motor manual): rotor speed vs rotor torque is a straight line,
#     rpm = 1746.5 - 200.2 * tau_rotor      (0.2 .. 4.0 N.m, fit rms 4 rpm)
# At the joint (6.33:1) that is no-load 28.9 rad/s -- the 30.1 rad/s spec -- stalling at
# 55.2 N.m, i.e. 2.33x the 23.7 N.m current limit. Flat until the two meet:
CORNER_SPEED_FRAC = 1.0 - 23.7 / 55.2  # 0.571


def torque_speed_bound(qd: np.ndarray) -> np.ndarray:
    """|tau| <= tau_peak, flat to CORNER_SPEED_FRAC*w_max, then linear to 0 at w_max."""
    w_max = speed_limits()
    derate = (w_max - np.abs(qd)) / (w_max * (1.0 - CORNER_SPEED_FRAC))
    return torque_limits() * np.clip(derate, 0.0, 1.0)


def torque_speed_halfplanes() -> tuple[np.ndarray, np.ndarray]:
    """(k, tau_stall) putting torque_speed_bound into a form an NLP can hold.

    abs() and clip() are non-smooth, so the bound above cannot be a constraint. The same
    envelope is the intersection of four half-planes per joint, linear in (tau, qd):

        -tau_peak <= tau <= tau_peak
         tau + k*qd <= tau_stall
        -tau - k*qd <= tau_stall

    These leave regeneration -- tau opposing qd -- underated, which is the physical behaviour
    and the one place they intentionally differ from torque_speed_bound.
    """
    tau_stall = torque_limits() / (1.0 - CORNER_SPEED_FRAC)
    return tau_stall / speed_limits(), tau_stall


# --- Poses (mirror go2.xml <keyframe>) --------------------------------------
# Leg block ordering matches JOINT_NAMES.
HOME_LEGS = np.array([0.0, 0.9, -1.8] * 4)
TUCK_LEGS = np.array([0.0, 2.2, -2.7] * 4)

HOME_BASE_HEIGHT = 0.27
TUCK_BASE_HEIGHT = 0.30

# HOME_BASE_HEIGHT is the keyframe value and sits well INTO the floor. This is the height that
# rests HOME_LEGS on it, and it is what the trajectory optimization must start and end from.
# Measured to the foot SPHERE, not to P_FOOT. The old P_FOOT-based value (0.2800479196045126)
# is 8.3 mm too low -- it rests the sphere that far INTO the floor, because the calf is tilted
# 51.6 deg at HOME (see R_FOOT above) -- so the trajectory used to start and end already
# penetrating. Only valid together with program.py pinning the sphere rather than P_FOOT.
STAND_BASE_HEIGHT = 0.2883725003026


def mj_qpos(legs: np.ndarray, height: float, quat_wxyz=(1.0, 0.0, 0.0, 0.0)) -> np.ndarray:
    """Full MuJoCo qpos: [x y z, qw qx qy qz, 12 joints]."""
    return np.concatenate([[0.0, 0.0, height], quat_wxyz, legs])


HOME_QPOS_MJ = mj_qpos(HOME_LEGS, HOME_BASE_HEIGHT)
TUCK_QPOS_MJ = mj_qpos(TUCK_LEGS, TUCK_BASE_HEIGHT)


# --- Drake index layout ------------------------------------------------------
# Confirmed against the finalized plant: the 12 joints occupy q[7:19] / v[6:18] in exactly
# JOINT_NAMES order, so the leg blocks below index both.
Q_QUAT, Q_POS, Q_JOINTS = slice(0, 4), slice(4, 7), slice(7, 19)
V_W, V_V, V_QD = slice(0, 3), slice(3, 6), slice(6, 18)

HIP_IDX = [0, 3, 6, 9]
THIGH_IDX = [1, 4, 7, 10]
CALF_IDX = [2, 5, 8, 11]
LEG_SLICE = {"FL": slice(0, 3), "FR": slice(3, 6), "RL": slice(6, 9), "RR": slice(9, 12)}
# Sagittal mirror: left leg <-> right leg. Hip flips sign, thigh and calf do not.
MIRROR_LEGS = [("FL", "FR"), ("RL", "RR")]


# --- Drake <-> MuJoCo state mapping -----------------------------------------
# Both use wxyz quaternions, so no component reordering -- but the blocks are swapped:
#
#            MuJoCo                        Drake
#   q    [x y z, qw qx qy qz]        [qw qx qy qz, x y z]
#   v    [v_world, w_BODY]           [w_world, v_world]
#
# The angular-velocity FRAME difference is the easy one to miss: it only shows up once
# the base is tilted, i.e. exactly during a flip.

def quat_wxyz_to_R(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def mj_to_drake_q(q_mj: np.ndarray) -> np.ndarray:
    """[x y z, quat, joints] -> [quat, x y z, joints]."""
    return np.concatenate([q_mj[3:7], q_mj[0:3], q_mj[7:]])


def drake_to_mj_q(q_dr: np.ndarray) -> np.ndarray:
    """[quat, x y z, joints] -> [x y z, quat, joints]."""
    return np.concatenate([q_dr[4:7], q_dr[0:4], q_dr[7:]])


def mj_to_drake_v(v_mj: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    """[v_world, w_body, qd] -> [w_world, v_world, qd]. Needs the base orientation."""
    R = quat_wxyz_to_R(quat_wxyz)
    return np.concatenate([R @ v_mj[3:6], v_mj[0:3], v_mj[6:]])


def drake_to_mj_v(v_dr: np.ndarray, quat_wxyz: np.ndarray) -> np.ndarray:
    """[w_world, v_world, qd] -> [v_world, w_body, qd]."""
    R = quat_wxyz_to_R(quat_wxyz)
    return np.concatenate([v_dr[3:6], R.T @ v_dr[0:3], v_dr[6:]])


def mj_to_drake_v_permutation() -> np.ndarray:
    """P with v_drake = P @ v_mujoco, valid only at identity base orientation.

    Useful for mass-matrix comparison: M_drake = P @ M_mujoco @ P.T
    """
    P = np.zeros((18, 18))
    P[0:3, 3:6] = np.eye(3)   # drake w  <- mujoco w
    P[3:6, 0:3] = np.eye(3)   # drake v  <- mujoco v
    P[6:, 6:] = np.eye(12)
    return P


# Self-collision pairs reachable inside the joint-limit box, measured over 40k random
# poses in MuJoCo, deepest first. Constrain these in the TO; the other ~184 candidate
# pairs are unreachable and cost nothing but solve time.
#
# Two things fall out of the sweep:
#   - SAME-LEG pairs never appear at all. The knee range already stops a calf from
#     reaching its own hip, so no constraint is needed there.
#   - Leg-to-torso only just grazes (FL_calf/base at -0.3 mm), so it is effectively
#     limit-protected too.
# Everything that genuinely collides is CROSS-LEG and driven by hip abduction rather
# than the sagittal tuck. Constraining abduction near 0 makes most of this inactive.
TUCK_COLLISION_PAIRS = [
    ("FL_thigh", "RL_thigh"), ("FL_thigh", "FR_thigh"), ("RL_thigh", "RR_thigh"),
    ("FR_thigh", "RR_thigh"),
    ("RL_calf", "RR_calf"), ("FL_calf", "RL_calf"), ("FR_calf", "RR_calf"),
    ("FL_calf", "FR_calf"), ("FR_calf", "RL_calf"), ("FL_calf", "RR_calf"),
    ("FL_calf", "RL_thigh"), ("FR_thigh", "RR_calf"), ("FR_calf", "RR_thigh"),
    ("RL_thigh", "RR_calf"), ("FL_thigh", "RL_calf"), ("FL_thigh", "FR_calf"),
    ("FL_calf", "FR_thigh"), ("RL_calf", "RR_thigh"), ("FL_calf", "RR_thigh"),
    ("FR_calf", "RL_thigh"), ("FR_thigh", "RL_calf"), ("FL_thigh", "RR_calf"),
    ("FL_calf", "RL_hip"), ("FR_calf", "RR_hip"),
]


# Self-collision-free box in the sagittal tuck variables (thigh/calf, front/rear), with hips at
# zero and the legs mirrored. Grown by tools/tuck_box.py from the region spanned by HOME_LEGS
# and TUCK_LEGS, with 5 mm of clearance. Contains both poses, so it bounds the whole flip.
TUCK_BOX = {
    "thigh_front": (-1.5708, 3.4907),
    "calf_front": (-2.7227, -1.685983),
    "thigh_rear": (0.741775, 3.5256),
    "calf_rear": (-2.7227, -0.83776),
}


# Floor-clearance witness spheres, per body KIND, generated and verified by
# tools/clearance_points.py -- each row is (x, y, z, radius) in that body's own frame, and the
# union of the spheres contains the body's collision geoms. The trajectory optimization knows
# no geometry beyond the four feet, so without these nothing stops the rest of the robot from
# sweeping through the floor, and it did: a solved trajectory drove the rear thigh 69 mm and
# the head 72 mm below z = 0. A flat floor only needs each geom's lowest point, and every
# collision geom in go2.xml is sphere-swept, which is what makes this cheap enough to impose
# at every knot: `p_z(q) >= r`.
#
# All four legs share one entry per kind (asserted by the generator). The y column is carried
# for completeness and is unused by the z constraint -- with quat_x = quat_z = 0 the base has
# pitch only, so a body point's world z does not depend on its y. That same fact is why +-y
# mirror pairs collapse to a single witness point here.
COLLISION_SPHERES = {
    "base": np.array([
        (-0.18810, -0.04675, +0.05700, 0.00000),
        (+0.18810, -0.04675, +0.05700, 0.00000),
        (+0.28500, +0.00000, +0.05500, 0.05000),
        (+0.28500, +0.00000, -0.03500, 0.05000),
        (-0.18810, -0.04675, -0.05700, 0.00000),
        (+0.18810, -0.04675, -0.05700, 0.00000),
        (+0.29300, +0.00000, -0.06000, 0.04700),
    ]),
    "hip": np.array([
        (+0.00000, +0.06000, +0.00000, 0.04600),
    ]),
    "thigh": np.array([
        (-0.01700, -0.01225, -0.00000, 0.00000),
        (+0.01700, -0.01225, -0.00000, 0.00000),
        (-0.01700, -0.01225, -0.21300, 0.00000),
        (+0.01700, -0.01225, -0.21300, 0.00000),
    ]),
    "calf": np.array([
        (+0.00066, +0.00000, -0.01394, 0.01300),
        (+0.01934, +0.00000, -0.10606, 0.01300),
        (+0.02107, +0.00000, -0.12653, 0.01100),
        (+0.01893, +0.00000, -0.16947, 0.01100),
        (-0.00200, +0.00000, -0.21300, 0.02200),
    ]),
}


# The tuck the flip actually has to hold, as a hard window on the middle of the flight phase.
# TUCK_BOX above only says "not self-colliding", which near-full extension satisfies, and
# nothing else in the cost rewards folding up -- so the optimizer left the legs sprawled at
# I_yy = 0.66 kg.m^2, WORSE than simply standing (0.48), and had to buy the resulting slow
# rotation with a 0.57 m CoM rise. Holding this window instead puts I_yy at 0.45-0.46, which
# is a 0.27 m rise for the same angular momentum -- the tuck is what makes the flip cheap.
# Wide enough to leave the solver real freedom; TUCK_LEGS sits comfortably inside it.
FLIGHT_TUCK = {
    "thigh": (1.30, 2.90),
    "calf": (-2.72, -2.00),
}
