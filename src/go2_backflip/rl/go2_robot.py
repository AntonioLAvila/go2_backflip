"""Unitree Go2 as an mjlab entity, built from this repo's ``go2_mjcf/go2.xml``.

mjlab ships no Go2. This mirrors ``mjlab.asset_zoo.robots.unitree_go1.go1_constants`` but takes
every number from ``go2_backflip.constants`` so the RL model cannot drift from the model the
trajectory was solved and validated on.

Actuators are ``DcMotorActuatorCfg`` -- an explicit ``kp (q* - q) + kd (qd* - qd) + tau_ff`` law
on a ``<motor>`` -- because that is the only mjlab actuator that honours a feedforward torque
(``ActuatorCmd.effort_target``); MuJoCo's built-in position/PD actuators drop it. Its linear
torque-speed clip ``min(peak, stall * (1 - qd / w_max))`` is exactly this repo's hardware
envelope (``constants.torque_speed_halfplanes``) once ``stall = peak / (1 - CORNER_SPEED_FRAC)``.
"""

from __future__ import annotations

import mujoco
from mjlab.actuator import DcMotorActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg

from go2_backflip import constants as K

# Hardware PD gains. The same numbers tools/mj_track.py defaults to (its Monte Carlo lands 100 %
# for kp 40-80, kd 1-3); training randomises around them (env_cfg.py ``pd_gains``). They are
# baked into the action scale below, so a deployed policy must run with exactly these on the
# motor boards.
KP = 60.0
KD = 2.0

# Names as authored in go2.xml.
GO2_FEET = ("FL", "FR", "RL", "RR")  # the foot sphere geoms
GO2_BODY_NAMES = (  # MJCF order; MotionCommand requires the floating base at index 0
    "base",
    "FL_hip", "FL_thigh", "FL_calf",
    "FR_hip", "FR_thigh", "FR_calf",
    "RL_hip", "RL_thigh", "RL_calf",
    "RR_hip", "RR_thigh", "RR_calf",
)
GO2_CALF_BODIES = ("FL_calf", "FR_calf", "RL_calf", "RR_calf")
# Every collision geom that is not a foot: touching the floor with any of these is the known
# failure mode (rear knees grazing in the landing crouch).
GO2_COLLISION_GEOMS = tuple(
    f"{leg}_{part}" for leg in GO2_FEET for part in ("hip_col", "thigh_col", "calf_upper", "calf_lower")
) + ("torso_box", "head_cyl", "head_sphere")


def get_go2_spec() -> mujoco.MjSpec:
    """go2.xml, minus what mjlab supplies itself.

    * The XML's 12 ``<motor>`` actuators go: mjlab adds its own, named after the joints. Leaving
      both would double-actuate every joint.
    * The XML keyframes go with them (their ``ctrl`` no longer matches ``nu``); mjlab writes an
      ``init_state`` keyframe from ``EntityCfg.init_state``.
    * Two IMU sensors are added under the names mjlab's stock task configs read
      (``robot/imu_lin_vel``, ``robot/imu_ang_vel``). The XML's own sensors stay.
    * ``<option cone/impratio>`` are reset to MuJoCo defaults: ``MujocoCfg.apply`` overwrites them
      after compilation anyway (env_cfg.py sets elliptic / 100 there), and a non-default option
      block on an attached entity only produces a warning.
    """
    spec = mujoco.MjSpec.from_file(K.MODEL_PATH)
    for act in list(spec.actuators):
        spec.delete(act)
    for key in list(spec.keys):
        spec.delete(key)
    for name, kind in (
        ("imu_lin_vel", mujoco.mjtSensor.mjSENS_VELOCIMETER),
        ("imu_ang_vel", mujoco.mjtSensor.mjSENS_GYRO),
    ):
        spec.add_sensor(name=name, type=kind, objtype=mujoco.mjtObj.mjOBJ_SITE, objname="imu")
    spec.option.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
    spec.option.impratio = 1.0
    return spec


def _motor(kind: str, names: tuple[str, ...], kp: float, kd: float, delay_max_lag: int) -> DcMotorActuatorCfg:
    peak = K.PEAK_TORQUE[kind]
    return DcMotorActuatorCfg(
        target_names_expr=names,
        stiffness=kp,
        damping=kd,
        effort_limit=peak,
        saturation_effort=peak / (1.0 - K.CORNER_SPEED_FRAC),
        velocity_limit=K.MAX_SPEED[kind],
        # armature / frictionloss / viscous_damping stay None: go2.xml is ground truth
        # (0.01 / 0.02 armature, 0.05 damping); domain randomisation scales them per env.
        delay_min_lag=0,
        delay_max_lag=delay_max_lag,  # physics steps; 3 x 2 ms = the 0-6 ms mj_track sweeps
    )


INIT_STATE = EntityCfg.InitialStateCfg(
    pos=(0.0, 0.0, K.STAND_BASE_HEIGHT),
    joint_pos={".*_hip_joint": 0.0, ".*_thigh_joint": 0.9, ".*_calf_joint": -1.8},  # HOME_LEGS
    joint_vel={".*": 0.0},
)


def get_go2_robot_cfg(kp: float = KP, kd: float = KD, command_delay_max_lag: int = 3) -> EntityCfg:
    """A fresh Go2 EntityCfg (mjlab configs are mutated in place, so never share one)."""
    return EntityCfg(
        init_state=INIT_STATE,
        spec_fn=get_go2_spec,
        # No CollisionCfg: go2.xml already carries the contact parameters the trajectory was
        # validated against (foot sphere condim 6, priority 1, mu 0.8, margin 0).
        articulation=EntityArticulationInfoCfg(
            actuators=(
                _motor("hip", (".*_hip_joint", ".*_thigh_joint"), kp, kd, command_delay_max_lag),
                _motor("calf", (".*_calf_joint",), kp, kd, command_delay_max_lag),
            ),
            # The reference runs the calves to 0.02 rad from the hard limit and the front thighs
            # to 1.70 of 3.49; any factor < 1 clips the reference on reset and penalises it.
            soft_joint_pos_limit_factor=1.0,
        ),
        sort_actuators=True,  # ctrl in joint order instead of [hips+thighs..., calves...]
    )


# Action of +-1 => +-25 % of peak torque at zero velocity error, mjlab's convention
# (0.099 rad on hip/thigh, 0.189 rad on the calf).
GO2_ACTION_SCALE: dict[str, float] = {
    ".*_hip_joint": 0.25 * K.PEAK_TORQUE["hip"] / KP,
    ".*_thigh_joint": 0.25 * K.PEAK_TORQUE["thigh"] / KP,
    ".*_calf_joint": 0.25 * K.PEAK_TORQUE["calf"] / KP,
}
