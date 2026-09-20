"""Go2 backflip tracking environment: mjlab's tracking task (BeyondMimic) on the Go2, driven the
way a Unitree low-level command is driven (PD position target + dq + feedforward) at 50 Hz.

The actor observation is laid out like a proprioceptive walking policy (joint pos/vel, gyro,
projected gravity, last action, each with a 5-frame history and a 0-1 step sensor delay) plus the
tracking task's reference terms (next reference frame, base-orientation error). No base linear
velocity or base position error by default: the Go2 has no reliable estimate of either.
"""

from __future__ import annotations

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs.mdp import dr
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationGroupCfg, ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.managers.termination_manager import TerminationTermCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.sim import MujocoCfg, SimulationCfg
from mjlab.tasks.tracking import mdp
from mjlab.tasks.tracking.mdp import MotionCommandCfg
from mjlab.tasks.tracking.tracking_env_cfg import make_tracking_env_cfg
from mjlab.utils.noise import UniformNoiseCfg as Unoise

from go2_backflip import constants as K
from go2_backflip.rl import mdp as go2_mdp
from go2_backflip.rl.actions import JointPositionFeedforwardActionCfg
from go2_backflip.rl.go2_robot import (
    GO2_ACTION_SCALE,
    GO2_BODY_NAMES,
    GO2_CALF_BODIES,
    GO2_COLLISION_GEOMS,
    GO2_FEET,
    get_go2_robot_cfg,
)

MOTION_FILE = str(K.REPO_ROOT / "traj_opt" / "reference" / "backflip_mjlab.npz")

# 50 Hz control (the DDS rate) from a 500 Hz physics step: the explicit PD in DcMotorActuator is
# integrated by the physics step, and kp = 60 on a 0.01 kg m^2 armature is marginal at 5 ms.
PHYSICS_DT = 0.002
DECIMATION = 10
HISTORY_LENGTH = 5  # 100 ms of proprioception, term-major (oldest -> newest), as the walking policy
OBS_DELAY_STEPS = (0, 1)  # 0-20 ms sensor latency
CMD_DELAY_STEPS = 3  # 0-6 ms command latency, in physics steps (the mj_track Monte-Carlo range)

# A flip in flight cannot absorb the stock 0.5 m/s kicks; these are the reset-state (RSI) noise
# and the in-episode push, both.
VELOCITY_RANGE = {
    "x": (-0.3, 0.3), "y": (-0.3, 0.3), "z": (-0.1, 0.1),
    "roll": (-0.3, 0.3), "pitch": (-0.3, 0.3), "yaw": (-0.4, 0.4),
}
POSE_RANGE = {
    "x": (-0.05, 0.05), "y": (-0.05, 0.05), "z": (-0.01, 0.01),
    "roll": (-0.1, 0.1), "pitch": (-0.1, 0.1), "yaw": (-0.2, 0.2),
}


def _proprio(func, noise: float, params: dict | None = None, delay: bool = True) -> ObservationTermCfg:
    """A proprioceptive actor term: noisy, delayed, with history."""
    return ObservationTermCfg(
        func=func,
        params=params or {},
        noise=Unoise(n_min=-noise, n_max=noise),
        history_length=HISTORY_LENGTH,
        delay_min_lag=OBS_DELAY_STEPS[0] if delay else 0,
        delay_max_lag=OBS_DELAY_STEPS[1] if delay else 0,
    )


def _actor_terms(has_state_estimation: bool, delay: bool) -> dict[str, ObservationTermCfg]:
    terms = {
        # Reference: next frame's joint pos + vel (24) and the base-orientation error to it (6).
        "command": ObservationTermCfg(func=mdp.generated_commands, params={"command_name": "motion"}),
        "motion_anchor_ori_b": ObservationTermCfg(
            func=mdp.motion_anchor_ori_b, params={"command_name": "motion"},
            noise=Unoise(n_min=-0.05, n_max=0.05),
        ),
        # Proprioception, 5 frames each.
        "base_ang_vel": _proprio(mdp.builtin_sensor, 0.2, {"sensor_name": "robot/imu_ang_vel"}, delay),
        "projected_gravity": _proprio(mdp.projected_gravity, 0.05, delay=delay),
        "joint_pos": _proprio(mdp.joint_pos_rel, 0.03, {"biased": True}, delay),
        "joint_vel": _proprio(mdp.joint_vel_rel, 1.5, delay=delay),
        "actions": ObservationTermCfg(func=mdp.last_action, history_length=HISTORY_LENGTH),
    }
    if has_state_estimation:
        terms["motion_anchor_pos_b"] = ObservationTermCfg(
            func=mdp.motion_anchor_pos_b, params={"command_name": "motion"},
            noise=Unoise(n_min=-0.25, n_max=0.25),
        )
        terms["base_lin_vel"] = _proprio(mdp.builtin_sensor, 0.5, {"sensor_name": "robot/imu_lin_vel"}, delay)
    return terms


def _critic_terms() -> dict[str, ObservationTermCfg]:
    c = {"command_name": "motion"}
    return {
        "command": ObservationTermCfg(func=mdp.generated_commands, params=c),
        "motion_anchor_pos_b": ObservationTermCfg(func=mdp.motion_anchor_pos_b, params=c),
        "motion_anchor_ori_b": ObservationTermCfg(func=mdp.motion_anchor_ori_b, params=c),
        "body_pos": ObservationTermCfg(func=mdp.robot_body_pos_b, params=c),
        "body_ori": ObservationTermCfg(func=mdp.robot_body_ori_b, params=c),
        "base_lin_vel": ObservationTermCfg(func=mdp.builtin_sensor, params={"sensor_name": "robot/imu_lin_vel"}),
        "base_ang_vel": ObservationTermCfg(func=mdp.builtin_sensor, params={"sensor_name": "robot/imu_ang_vel"}),
        "projected_gravity": ObservationTermCfg(func=mdp.projected_gravity),
        "joint_pos": ObservationTermCfg(func=mdp.joint_pos_rel),
        "joint_vel": ObservationTermCfg(func=mdp.joint_vel_rel),
        "actions": ObservationTermCfg(func=mdp.last_action),
        "reference_contact": ObservationTermCfg(func=go2_mdp.reference_contact, params=c),
    }


def _events() -> dict[str, EventTermCfg]:
    robot = SceneEntityCfg("robot")
    feet = SceneEntityCfg("robot", geom_names=GO2_FEET)
    startup = lambda func, **params: EventTermCfg(func=func, mode="startup", params=params)
    return {
        # Payload / battery / inertia: mass and inertia scaled together, CoM shifted.
        "base_inertia": startup(dr.pseudo_inertia, asset_cfg=SceneEntityCfg("robot", body_names=("base",)),
                                alpha_range=(-0.1, 0.1), t_range=(-0.02, 0.02)),
        # Foot friction, per foot; torsional / rolling around the XML's 0.02 / 0.01.
        "foot_friction": startup(dr.geom_friction, asset_cfg=feet, operation="abs", axes=[0], ranges=(0.4, 1.2)),
        "foot_friction_spin": startup(dr.geom_friction, asset_cfg=feet, operation="abs", axes=[1],
                                      distribution="log_uniform", ranges=(5e-3, 5e-2)),
        "foot_friction_roll": startup(dr.geom_friction, asset_cfg=feet, operation="abs", axes=[2],
                                      distribution="log_uniform", ranges=(1e-3, 2e-2)),
        # Joint dynamics the TO model does not carry (STATUS.md: dry friction unmodelled).
        "joint_friction": startup(dr.joint_friction, asset_cfg=robot, operation="abs", ranges=(0.0, 0.3)),
        "joint_damping": startup(dr.joint_damping, asset_cfg=robot, operation="scale", ranges=(0.4, 2.0)),
        "joint_armature": startup(dr.joint_armature, asset_cfg=robot, operation="scale", ranges=(0.5, 2.0)),
        "encoder_bias": startup(dr.encoder_bias, asset_cfg=robot, bias_range=(-0.02, 0.02)),
        # Motor: gains and strength.
        "pd_gains": startup(dr.pd_gains, asset_cfg=robot, kp_range=(0.8, 1.25), kd_range=(0.75, 1.3), operation="scale"),
        "effort_limits": startup(dr.effort_limits, asset_cfg=robot, effort_limit_range=(0.85, 1.0), operation="scale"),
        # Gravity, per episode (emulated: constant m*dg on every body).
        "gravity_offset": EventTermCfg(func=go2_mdp.gravity_offset, mode="reset",
                                       params={"g_range": (-0.5, 0.5), "tilt_range": (-0.3, 0.3)}),
        # Pushes: velocity kicks (must not be a wrench, that would overwrite gravity_offset).
        "push_robot": EventTermCfg(func=mdp.push_by_setting_velocity, mode="interval",
                                   interval_range_s=(1.0, 3.0), params={"velocity_range": VELOCITY_RANGE}),
    }


def go2_backflip_tracking_env_cfg(
    has_state_estimation: bool = False,
    play: bool = False,
) -> ManagerBasedRlEnvCfg:
    cfg = make_tracking_env_cfg()
    delay = not play

    # -- scene -------------------------------------------------------------------------------
    cfg.scene.entities = {"robot": get_go2_robot_cfg(command_delay_max_lag=0 if play else CMD_DELAY_STEPS)}
    cfg.scene.num_envs = 4096
    cfg.scene.sensors = (
        ContactSensorCfg(
            name="self_collision",
            primary=ContactMatch(mode="subtree", pattern="base", entity="robot"),
            secondary=ContactMatch(mode="subtree", pattern="base", entity="robot"),
            fields=("found", "force"), reduce="none", num_slots=1, history_length=4,
        ),
        ContactSensorCfg(  # anything but a foot on the floor
            name="stray_ground",
            primary=ContactMatch(mode="geom", pattern=GO2_COLLISION_GEOMS, entity="robot"),
            secondary=ContactMatch(mode="body", pattern="terrain"),
            fields=("found", "force"), reduce="none", num_slots=1, history_length=DECIMATION,
        ),
    )

    # -- action ------------------------------------------------------------------------------
    cfg.actions = {
        "joint_pos": JointPositionFeedforwardActionCfg(  # key must stay "joint_pos" (ONNX export)
            entity_name="robot",
            actuator_names=(".*",),
            scale=GO2_ACTION_SCALE,
            use_default_offset=True,
            offset_mode="reference",
            ff_scale_range=None if play else (0.8, 1.2),
        )
    }

    # -- command -----------------------------------------------------------------------------
    motion = cfg.commands["motion"]
    assert isinstance(motion, MotionCommandCfg)
    motion.motion_file = MOTION_FILE
    motion.anchor_body_name = "base"
    motion.body_names = GO2_BODY_NAMES
    motion.pose_range = POSE_RANGE
    motion.velocity_range = VELOCITY_RANGE
    motion.joint_position_range = (-0.05, 0.05)
    motion.sampling_mode = "adaptive"

    # -- observations ------------------------------------------------------------------------
    cfg.observations = {
        "actor": ObservationGroupCfg(
            terms=_actor_terms(has_state_estimation, delay), concatenate_terms=True, enable_corruption=not play
        ),
        "critic": ObservationGroupCfg(terms=_critic_terms(), concatenate_terms=True, enable_corruption=False),
    }

    # -- events ------------------------------------------------------------------------------
    cfg.events = _events()

    # -- rewards -----------------------------------------------------------------------------
    cfg.rewards["motion_global_root_ori"].weight = 1.0  # the relative terms re-anchor by yaw, ill-posed mid-flip
    cfg.rewards["stray_contact"] = RewardTermCfg(
        func=mdp.self_collision_cost, weight=-2.0,
        params={"sensor_name": "stray_ground", "force_threshold": 5.0},
    )
    cfg.rewards["joint_torques"] = RewardTermCfg(func=mdp.joint_torques_l2, weight=-2e-5)

    # -- terminations ------------------------------------------------------------------------
    cfg.terminations["motion_finished"] = TerminationTermCfg(
        func=go2_mdp.motion_finished, params={"command_name": "motion"}, time_out=True
    )
    cfg.terminations["ee_body_pos"].params["body_names"] = GO2_CALF_BODIES
    cfg.terminations["nan"] = TerminationTermCfg(func=mdp.nan_detection)

    # -- sim ---------------------------------------------------------------------------------
    cfg.sim = SimulationCfg(
        nconmax=48, njmax=300,
        mujoco=MujocoCfg(timestep=PHYSICS_DT, cone="elliptic", impratio=100.0, iterations=10, ls_iterations=20),
    )
    cfg.decimation = DECIMATION
    cfg.episode_length_s = 4.0  # > the 3.4 s clip: motion_finished, not time_out, ends an episode
    cfg.viewer.body_name = "base"
    cfg.viewer.distance = 2.0

    if play:
        cfg.episode_length_s = int(1e9)
        for key in ("push_robot", "gravity_offset", "base_inertia", "foot_friction", "foot_friction_spin",
                    "foot_friction_roll", "joint_friction", "joint_damping", "joint_armature", "encoder_bias",
                    "pd_gains", "effort_limits"):
            cfg.events.pop(key, None)
        motion.pose_range = {}
        motion.velocity_range = {}
        motion.joint_position_range = (0.0, 0.0)
        motion.sampling_mode = "start"

    return cfg
