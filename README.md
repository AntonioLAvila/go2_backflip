# go2_backflip

A dynamically feasible backflip reference for the Unitree Go2 plus the RL environment
that learns a stabilizing controller.

Trajectory Generation:

    uv sync
    uv run traj_opt/solve.py            # solve (~40 s), audit, closed-loop MuJoCo rollout
    uv run tools/mj_track.py --view     # watch the shipped reference tracked by a joint PD
    uv run tools/export_mjlab.py        # clip for mjlab's motion-tracking task

RL:

    uv run train Mjlab-Tracking-Flat-Unitree-Go2-Backflip       # train
    uv run play Mjlab-Tracking-Flat-Unitree-Go2-Backflip \      # visualize
        --agent trained \
        --checkpoint-file <some_model.pt> \
        --motion-file traj_opt/reference/<some_reference.npz> \
        --num-envs 1

See `CLAUDE.md` for the architecture and `traj_opt/STATUS.md` for current numbers.
