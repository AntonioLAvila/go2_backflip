"""mjlab task registration for the Go2 backflip. Loaded by mjlab through the ``mjlab.tasks``
entry point declared in pyproject.toml, so ``uv run train/play/list-envs`` see these tasks.

Keep this import path free of casadi / drake: mjlab imports it at startup.
"""

from mjlab.tasks.registry import register_mjlab_task
from mjlab.tasks.tracking.rl import MotionTrackingOnPolicyRunner

from go2_backflip.rl.env_cfg import go2_backflip_tracking_env_cfg
from go2_backflip.rl.rl_cfg import go2_backflip_ppo_runner_cfg

TASK_ID = "Mjlab-Tracking-Flat-Unitree-Go2-Backflip"
TASK_ID_STATE_ESTIMATION = TASK_ID + "-State-Estimation"

register_mjlab_task(
    task_id=TASK_ID,
    env_cfg=go2_backflip_tracking_env_cfg(),
    play_env_cfg=go2_backflip_tracking_env_cfg(play=True),
    rl_cfg=go2_backflip_ppo_runner_cfg(),
    runner_cls=MotionTrackingOnPolicyRunner,
)

register_mjlab_task(
    task_id=TASK_ID_STATE_ESTIMATION,
    env_cfg=go2_backflip_tracking_env_cfg(has_state_estimation=True),
    play_env_cfg=go2_backflip_tracking_env_cfg(has_state_estimation=True, play=True),
    rl_cfg=go2_backflip_ppo_runner_cfg(),
    runner_cls=MotionTrackingOnPolicyRunner,
)
