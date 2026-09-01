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

from dataclasses import dataclass

ALL = ("FL", "FR", "RL", "RR")


@dataclass(frozen=True)
class Phase:
    name: str
    contacts: tuple[str, ...]
    n_knots: int
    h_min: float
    h_max: float


PHASES = (
    Phase("load", ALL, 12, 0.004, 0.020),
    Phase("launch", ("RL", "RR"), 12, 0.004, 0.020),
    Phase("flight", (), 32, 0.004, 0.026),
    Phase("absorb", ALL, 16, 0.004, 0.025),
)

FLIGHT = 2
IMPACT = 3          # phase entered through the touchdown impulse
N_KNOTS = sum(p.n_knots for p in PHASES)
