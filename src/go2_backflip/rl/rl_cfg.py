"""PPO runner config for the Go2 backflip tracking task (rsl_rl through mjlab)."""

from mjlab.rl import RslRlModelCfg, RslRlOnPolicyRunnerCfg, RslRlPpoAlgorithmCfg


def go2_backflip_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
    return RslRlOnPolicyRunnerCfg(
        actor=RslRlModelCfg(
            hidden_dims=(512, 256, 128),
            activation="elu",
            obs_normalization=True,
            # The action is a residual on the reference at 0.1-0.19 rad per unit. tools/rl_env_check.py
            # --action-noise: white noise of 0.2 still lands every replay, 0.3 breaks the landing.
            distribution_cfg={"class_name": "GaussianDistribution", "init_std": 0.2, "std_type": "scalar"},
        ),
        critic=RslRlModelCfg(hidden_dims=(512, 256, 128), activation="elu", obs_normalization=True),
        algorithm=RslRlPpoAlgorithmCfg(
            value_loss_coef=1.0,
            use_clipped_value_loss=True,
            clip_param=0.2,
            entropy_coef=0.005,
            num_learning_epochs=5,
            num_mini_batches=4,
            learning_rate=3.0e-4,
            schedule="adaptive",
            gamma=0.99,
            lam=0.95,
            desired_kl=0.01,
            max_grad_norm=1.0,
        ),
        experiment_name="go2_backflip",
        save_interval=250,
        num_steps_per_env=24,
        max_iterations=10_000,
        logger="tensorboard",
    )
