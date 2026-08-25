"""Shared Go2 model constants.

Everything derived from ``go2_mjcf/go2.xml`` -- actuator limits, poses, joint index
layout, the Drake<->MuJoCo state conversion -- lives in :mod:`go2_backflip.constants`,
so the Drake trajectory optimization and the downstream mjlab RL config cannot drift
apart. Installed as a package precisely so a separate repo can depend on it.
"""
