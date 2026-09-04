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


# h_max on the three STANCE phases was raised 2026-09-03, and the reason is worth keeping.
# At the old values every one of them sat exactly on its bound -- load 0.0200/0.0200,
# launch 0.0200/0.0200, absorb 0.0249999/0.0250 -- i.e. the solve wanted longer stance than
# the schedule allowed, which is physically unsurprising (more time on the ground is more
# impulse at a fixed torque limit). That costs twice over. It caps the phase duration, and,
# because AddEqualTimeIntervalsConstraints also ties a phase's steps to each other, it puts
# 11 active bounds on top of 10 equalities over 11 variables -- over-determined by 10 per
# phase, and together the two largest families in the active set's rank deficiency.
# Flight is left alone: at 0.0244 against a 0.032 cap it is the one phase already choosing
# its own step, because its duration is ballistic and set by the takeoff velocity, not by a
# bound. The raise is deliberately moderate; local integration error grows like h^5, and
# load's is 1.3e-4 against a 5e-3 audit threshold, so ~40% of step is affordable and much
# more would not be.
PHASES = (
    Phase("load", ALL, 12, 0.004, 0.028),
    Phase("launch", ("RL", "RR"), 12, 0.004, 0.028),
    Phase("flight", (), 26, 0.004, 0.032),
    Phase("absorb", ALL, 16, 0.004, 0.033),
)

FLIGHT = 2
IMPACT = 3          # phase entered through the touchdown impulse
N_KNOTS = sum(p.n_knots for p in PHASES)
