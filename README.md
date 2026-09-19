# go2_backflip

A dynamically feasible backflip reference for the Unitree Go2, and the checks that prove it
against MuJoCo. Stage 1 of trajectory optimization → RL imitation → hardware.

    uv sync
    uv run traj_opt/solve.py            # solve (~40 s), audit, closed-loop MuJoCo rollout
    uv run tools/mj_track.py --view     # watch the shipped reference tracked by a joint PD
    uv run tools/export_mjlab.py        # clip for mjlab's motion-tracking task

See `CLAUDE.md` for the architecture and `traj_opt/STATUS.md` for current numbers.
