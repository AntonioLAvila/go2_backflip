"""Contact schedule for the backflip.

Drake/MJCF frame is x-forward, z-up, so R_y(+theta) tips the nose DOWN: a backflip is
w_y < 0 and a net base rotation of -2*pi. About the CoM a vertical GRF gives
tau_y = -r_x * F_z, so nose-up torque needs the support point AHEAD of the CoM.

That fixes the order. From flat stance the front feet (r_x > 0) supply the pitch-up moment;
once the body has reared past ~40 deg the CoM has swung behind the rear feet, which then have
r_x > 0 themselves and keep driving the rotation as they extend to launch. The front pair is
therefore what breaks contact first.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

ALL = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class Phase:
    name: str
    contacts: tuple[str, ...]
    n_knots: int
    h_min: float
    h_max: float


# h_max on the three stance phases was raised to 0.028/0.028/0.033 on 2026-09-03 and put back.
# The raise was well motivated -- every stance phase sat exactly on its bound, so the solve
# wanted more ground time than the schedule allowed, and the bound being active also stacked
# 11 active rows on 10 equal-interval equalities over 11 variables. It is reverted because the
# cost showed up where the audit looks: launch's end-of-phase integration drift goes 1.8e-3 at
# h_max 0.020 to 1.8e-2 at 0.028, and drift is a check that fails while the h_max degeneracy
# only affected a violation number that turned out not to track trajectory quality.
#
# Flight stays at 26. 34 knots was tried on the fixed formulation and it SOLVES (0.0028, where
# the old formulation stalled at 2.13 growing to 32) -- but the extra freedom is what let the
# search reach the spurious 0.0004 point that audits 7/11. Growing flight is available again;
# it is just not obviously wanted.
# Overridable by GO2_FLIGHT_KNOTS so a mesh-refinement run can happen SIDE BY SIDE with
# 50-knot work instead of requiring an edit here. Editing the number is a documented footgun:
# every 50-knot checkpoint in traj_opt/out/ stops loading the moment it changes, including the
# ones a concurrent search is still writing, and warm_start.py has to export BEFORE the edit
# while this file still matches the source checkpoint. An env var keeps both worlds runnable:
#     GO2_FLIGHT_KNOTS=99 uv run traj_opt/solve_backflip.py --warm-start ...
# Refine on a 2n-1 grid so the new knots are a superset of the old -- see warm_start.py.
FLIGHT_KNOTS = int(os.environ.get("GO2_FLIGHT_KNOTS", "50"))

PHASES = (
    Phase("load", ALL, 12, 0.004, 0.020),
    Phase("launch", ("RL", "RR"), 12, 0.004, 0.020),
    Phase("flight", (), FLIGHT_KNOTS, 0.004, 0.032),
    Phase("absorb", ALL, 16, 0.004, 0.025),
)

FLIGHT = 2
IMPACT = 3          # phase entered through the touchdown impulse
N_KNOTS = sum(p.n_knots for p in PHASES)
