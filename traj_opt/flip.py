"""The backflip as a multi-phase optimal control problem on the sagittal model.

Four phases -- load (all feet), launch (rear feet), flight, absorb (all feet) -- each
transcribed by Hermite-Simpson collocation in *separated* form: every node, knot or midpoint,
carries its own (q, v, a, u, lambda) and satisfies the equations of motion there. Nothing is
first-order-held, so the contact term at a midpoint is J(q_m)^T lambda_m, not an average.

Stance is an ODE, not a DAE. A stance foot obeys the rolling-sphere constraint at the
ACCELERATION level, at every node, with Baumgarte feedback:

    d2g/dt2 + 2*alpha*dg/dt + alpha^2*(g - g0) = 0

Together with the equations of motion that is a square, nonsingular system for (a, lambda)
given (q, v, u), so each phase is an ordinary ODE in (q, v) whose flow stays on the contact
manifold because it starts there: HOME at rest for load, inherited for launch, and the
touchdown impact map (which makes J v+ = 0) for absorb. No position or velocity pins are
imposed on top of the defects, which is the redundancy that made the previous formulation
rank-deficient; for the same reason the terminal condition pins only as many coordinates as
the stance manifold has freedoms.

IPOPT gets exact first and second derivatives from CasADi. Cold start to convergence is a few
seconds.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import casadi as ca
import numpy as np

from go2_backflip import constants as K
from go2_backflip import sagittal as S

NQ, NJ = S.NQ, S.NJ
LEGS = slice(3, 7)


@dataclass(frozen=True)
class Phase:
    name: str
    contacts: tuple[str, ...]          # subset of S.PAIRS
    n: int                             # intervals
    T_lo: float
    T_hi: float
    T0: float                          # guess


@dataclass
class Config:
    phases: tuple[Phase, ...] = (
        Phase("load", ("front", "rear"), 16, 0.10, 0.60, 0.30),
        Phase("launch", ("rear",), 16, 0.08, 0.40, 0.18),
        Phase("flight", (), 30, 0.30, 0.80, 0.50),
        Phase("absorb", ("front", "rear"), 24, 0.15, 0.80, 0.40),
    )
    mu: float = K.MU_TO
    torque_sf: float = 0.90            # fraction of nominal torque the reference may use
    speed_sf: float = 0.90
    body_clearance: float = 0.03       # m, every non-foot witness sphere, every node
    foot_clearance: float = 0.02       # m, swing feet away from their phase boundaries
    clear_ramp: float = 0.2            # fraction of a phase over which that demand ramps in
    joint_inset: float = 0.02          # rad kept clear of every joint stop
    x_land: float = 0.15
    alpha: float = 20.0                # Baumgarte rate, 1/s
    rotation: float = -2 * np.pi
    # cost weights
    w_u: float = 1.0                   # int (u/u_max)^2
    w_du: float = 2e-3                 # int (du/dt / u_max)^2
    w_a: float = 1e-5                  # int qdd_joint^2
    w_impulse: float = 1e-3            # (touchdown impulse / (m g * 0.1 s))^2
    w_dlam: float = 1e-3               # int (dlam/dt / mg)^2
    ipopt: dict = field(default_factory=dict)


class _NLP:
    def __init__(self):
        self.x, self.x0, self.lbx, self.ubx = [], [], [], []
        self.g, self.lbg, self.ubg, self.gname = [], [], [], []
        self.J = 0

    def var(self, name, n, lb=-np.inf, ub=np.inf, guess=0.0, scale=1.0):
        s = ca.MX.sym(name, n)
        one = np.ones(n)
        self.x.append(s)
        self.x0.append(np.asarray(guess, float) * one / scale)
        self.lbx.append(np.asarray(lb, float) * one / scale)
        self.ubx.append(np.asarray(ub, float) * one / scale)
        return s * scale

    def con(self, name, e, lb=0.0, ub=0.0, scale=1.0):
        e = ca.vec(e) / scale
        n = e.numel()
        self.g.append(e)
        self.lbg.append(np.asarray(lb, float) * np.ones(n) / scale)
        self.ubg.append(np.asarray(ub, float) * np.ones(n) / scale)
        self.gname += [name] * n


def _limits(cfg):
    idx = [K.THIGH_IDX[0], K.CALF_IDX[0], K.THIGH_IDX[2], K.CALF_IDX[2]]
    nominal = np.array([K.NOMINAL_TORQUE[K.joint_kind(n)] for n in K.JOINT_NAMES])
    u_max = cfg.torque_sf * nominal[idx]
    w_max = K.speed_limits()[idx]
    stall = u_max / (1.0 - K.CORNER_SPEED_FRAC)
    return u_max, cfg.speed_sf * w_max, stall / w_max, stall, cfg.torque_sf * K.NOMINAL_TORQUE["hip"]


def initial_guess(cfg: Config):
    """Piecewise-linear key poses. Crude on purpose: it only has to wind the right way."""
    T = [p.T0 for p in cfg.phases]
    vz = 0.5 * S.G * T[2]
    H = S.HOME
    keys = [
        H,
        np.array([-0.05, 0.30, -0.70, 0.20, -1.30, 1.40, -2.30]),
        np.array([-0.15, 0.48, -1.50, 0.60, -1.80, 0.80, -1.20]),
        np.array([-0.30, 0.42, cfg.rotation + 0.35, 0.70, -1.60, 0.90, -1.60]),
        H + np.array([-0.35, 0, cfg.rotation, 0, 0, 0, 0]),
    ]
    tuck = np.array([2.2, -2.6, 2.2, -2.6])
    out = []
    for i, ph in enumerate(cfg.phases):
        s = np.linspace(0, 1, 2 * ph.n + 1)[:, None]
        q = (1 - s) * keys[i] + s * keys[i + 1]
        if ph.name == "flight":
            t = s[:, 0] * T[i]
            q[:, 1] = keys[i][1] + vz * t - 0.5 * S.G * t ** 2 + s[:, 0] * (keys[i + 1][1] - keys[i][1])
            bump = np.sin(np.pi * s) ** 2
            q[:, LEGS] = (1 - bump) * q[:, LEGS] + bump * tuck
        v = np.gradient(q, s[:, 0] * T[i], axis=0)
        out.append(dict(q=q, v=v, a=np.zeros_like(q), T=T[i]))
    return out


def build(cfg: Config, guess=None):
    sag = S.build()
    nlp = _NLP()
    u_max, w_max, k_ts, stall, hip_max = _limits(cfg)
    mg = sag.total_mass * S.G
    guess = guess or initial_guess(cfg)
    g_home = np.array(sag.foot(S.HOME)).ravel()
    sph_names = sag.sphere_names
    is_foot = {"front": sph_names.index("FL_calf[4]"), "rear": sph_names.index("RL_calf[4]")}
    tuck_lo = [K.TUCK_BOX[n][0] for n in ("thigh_front", "calf_front", "thigh_rear", "calf_rear")]
    tuck_hi = [K.TUCK_BOX[n][1] for n in ("thigh_front", "calf_front", "thigh_rear", "calf_rear")]

    q_lo, q_hi = sag.q_lo + cfg.joint_inset, sag.q_hi - cfg.joint_inset
    q_lo[:3], q_hi[:3] = [-1.5, 0.05, cfg.rotation - 1.0], [1.5, 1.5, 1.0]
    v_b = np.array([10, 10, 25] + list(w_max))

    V = []                                               # per phase: dict of node variables
    for p, (ph, gs) in enumerate(zip(cfg.phases, guess)):
        nn = 2 * ph.n + 1
        T = nlp.var(f"T_{ph.name}", 1, ph.T_lo, ph.T_hi, gs["T"])
        h = T / ph.n
        flight = not ph.contacts
        lo, hi = q_lo.copy(), q_hi.copy()
        if flight:
            lo[LEGS], hi[LEGS] = np.maximum(lo[LEGS], tuck_lo), np.minimum(hi[LEGS], tuck_hi)
        nodes = []
        for i in range(nn):
            lam_guess = mg / (2 * len(ph.contacts)) if ph.contacts else 0.0
            nd = dict(
                q=nlp.var(f"q_{p}_{i}", NQ, lo, hi, np.clip(gs["q"][i], lo, hi)),
                v=nlp.var(f"v_{p}_{i}", NQ, -v_b, v_b, np.clip(gs["v"][i], -v_b, v_b), 10.0),
                a=nlp.var(f"a_{p}_{i}", NQ, -2e3, 2e3, gs["a"][i], 100.0),
                u=nlp.var(f"u_{p}_{i}", NJ, -u_max, u_max,
                          gs["u"][i] if "u" in gs else 0.0, 20.0),
                lam={c: nlp.var(f"lam_{c}_{p}_{i}", 2, [-2e3, 0.0], [2e3, 2e3],
                                gs["lam"][c][i] if "lam" in gs else [0.0, lam_guess], 100.0)
                     for c in ph.contacts},
            )
            nodes.append(nd)
        V.append(dict(T=T, h=h, nodes=nodes))

        # foothold reference for the Baumgarte term
        if ph.name == "absorb":
            g0 = sag.foot(nodes[0]["q"])
            g0 = ca.vertcat(g0[0], 0, g0[2], 0)
        else:
            g0 = ca.DM(g_home)

        zero2 = ca.MX.zeros(2)
        for i, nd in enumerate(nodes):
            q, v, a, u = nd["q"], nd["v"], nd["a"], nd["u"]
            lf, lr = nd["lam"].get("front", zero2), nd["lam"].get("rear", zero2)
            nlp.con(f"dyn_{ph.name}", sag.dyn(q, v, a, u, lf, lr), scale=20.0)

            if ph.contacts:
                rows = [r for c in ph.contacts for r in ((0, 1) if c == "front" else (2, 3))]
                gdd = sag.foot_acc(q, v, a)
                gd = sag.foot_J(q) @ v
                bg = gdd + 2 * cfg.alpha * gd + cfg.alpha ** 2 * (sag.foot(q) - g0)
                nlp.con(f"contact_{ph.name}", bg[rows], scale=10.0)

            # actuators: torque-speed halfplanes on the sagittal motors, flat bound on the hips
            qd = v[LEGS]
            nlp.con("ts+", u + k_ts * qd, -np.inf, stall, scale=20.0)
            nlp.con("ts-", -u - k_ts * qd, -np.inf, stall, scale=20.0)
            nlp.con("hip", sag.hip(q, v, a, lf, lr), -hip_max, hip_max, scale=20.0)

            # friction cone, except where the force is pinned to zero for a clean release
            last = i == nn - 1
            for c in ph.contacts:
                lam = nd["lam"][c]
                releasing = last and p + 1 < len(cfg.phases) and c not in cfg.phases[p + 1].contacts
                if releasing:
                    nlp.con(f"release_{c}", lam)
                else:
                    nlp.con(f"cone_{c}", ca.vertcat(cfg.mu * lam[1] - lam[0], cfg.mu * lam[1] + lam[0]),
                            0.0, np.inf, scale=100.0)

            # floor clearance; a stance foot's own sphere is held by the contact constraint
            z = sag.spheres(q)
            first = i == 0
            for j in range(z.numel()):
                pair = next((c for c, jj in is_foot.items() if jj == j), None)
                if pair is None:
                    nlp.con("body_clear", z[j], cfg.body_clearance, np.inf)
                elif pair not in ph.contacts:
                    # Where the neighbouring phase holds this foot on the floor, the shared knot's
                    # height is already fixed by that phase; a bound here would duplicate it.
                    prev_c = cfg.phases[p - 1].contacts if p else ()
                    next_c = cfg.phases[p + 1].contacts if p + 1 < len(cfg.phases) else ()
                    if (first and pair in prev_c) or (last and pair in next_c):
                        continue
                    # Ramp the demand in over a fixed FRACTION of the phase next to a lift-off
                    # or touchdown. A node count here would make the problem mesh-dependent:
                    # refining would ask for the same lift in half the time.
                    s_ = i / (nn - 1)
                    ramp = min(1.0, s_ / cfg.clear_ramp if pair in prev_c else 1.0,
                               (1 - s_) / cfg.clear_ramp if pair in next_c else 1.0)
                    nlp.con("foot_clear", z[j], cfg.foot_clearance * ramp, np.inf)

        # Hermite-Simpson, separated form
        for k in range(ph.n):
            a_, m_, b_ = nodes[2 * k], nodes[2 * k + 1], nodes[2 * k + 2]
            for s, f, sc in (("q", "v", 1.0), ("v", "a", 10.0)):
                nlp.con(f"hs_mid_{s}", m_[s] - 0.5 * (a_[s] + b_[s]) - h / 8 * (a_[f] - b_[f]), scale=sc)
                nlp.con(f"hs_end_{s}", b_[s] - a_[s] - h / 6 * (a_[f] + 4 * m_[f] + b_[f]), scale=sc)

            # running cost, Simpson
            def run(nd):
                c = cfg.w_u * ca.sumsqr(nd["u"] / u_max) + cfg.w_a * ca.sumsqr(nd["a"][LEGS])
                return c
            nlp.J += h / 6 * (run(a_) + 4 * run(m_) + run(b_))
            for x_, y_ in ((a_, m_), (m_, b_)):
                nlp.J += cfg.w_du * ca.sumsqr((y_["u"] - x_["u"]) / u_max) / (h / 2)
                for c in ph.contacts:
                    nlp.J += cfg.w_dlam * ca.sumsqr((y_["lam"][c] - x_["lam"][c]) / mg) / (h / 2)

    # --- stitching ------------------------------------------------------------------------------
    impulse = None
    for p in range(len(cfg.phases) - 1):
        a_, b_ = V[p]["nodes"][-1], V[p + 1]["nodes"][0]
        nlp.con("stitch_q", b_["q"] - a_["q"])
        new = [c for c in cfg.phases[p + 1].contacts if c not in cfg.phases[p].contacts]
        if not new:
            nlp.con("stitch_v", b_["v"] - a_["v"], scale=10.0)
            continue
        # touchdown: inelastic impact on the new contacts (all of absorb's, here)
        rows = [r for c in cfg.phases[p + 1].contacts for r in ((0, 1) if c == "front" else (2, 3))]
        Jg = sag.foot_J(b_["q"])[rows, :]
        impulse = nlp.var("impulse", len(rows), -50, 50, 0.0, 5.0)
        nlp.con("impact", sag.mass(b_["q"]) @ (b_["v"] - a_["v"]) - 2.0 * Jg.T @ impulse, scale=1.0)
        nlp.con("impact_stick", Jg @ b_["v"])
        nlp.con("touchdown_z", sag.foot(b_["q"])[[r for r in rows if r % 2]])
        for c in range(len(rows) // 2):
            ix, iz = impulse[2 * c], impulse[2 * c + 1]
            nlp.con("impulse_cone", ca.vertcat(cfg.mu * iz - ix, cfg.mu * iz + ix), 0.0, np.inf)
        nlp.J += cfg.w_impulse * ca.sumsqr(impulse / (mg * 0.1))

    # --- boundary ------------------------------------------------------------------------------
    first, last = V[0]["nodes"][0], V[-1]["nodes"][-1]
    nlp.con("x0_q", first["q"] - S.HOME)
    nlp.con("x0_v", first["v"])
    # ...in static equilibrium, so a standing hold can be spliced on in front. Base rows only:
    # with v = 0 the contact rows already turn a_base = 0 into a_legs = 0.
    nlp.con("x0_a", first["a"][:3], scale=100.0)
    # Terminal: the double-stance manifold has 3 position freedoms plus 2 free footholds, and 3
    # velocity freedoms. Pin exactly that many; base height and joint rates then follow from the
    # contact constraint instead of fighting it.
    nlp.con("xf_theta", last["q"][2] - cfg.rotation)
    nlp.con("xf_legs", last["q"][LEGS] - S.HOME[LEGS])
    nlp.con("xf_v", last["v"][:3], scale=10.0)
    nlp.con("xf_a", last["a"][:3], scale=100.0)
    nlp.con("xf_x", last["q"][0], -cfg.x_land, cfg.x_land)

    return nlp, V, impulse


def solve(cfg: Config | None = None, guess=None, verbose=True):
    cfg = cfg or Config()
    nlp, V, impulse = build(cfg, guess)
    x = ca.vertcat(*nlp.x)
    prob = dict(x=x, f=nlp.J, g=ca.vertcat(*nlp.g))
    # A warm start is already near its optimum: start the barrier low and keep it monotone, or
    # the first iterations push the point back to the centre of every bound it rides.
    warm = ({"mu_strategy": "monotone", "mu_init": 1e-4, "bound_push": 1e-6, "bound_frac": 1e-6}
            if guess is not None else {"mu_strategy": "adaptive"})
    opts = {"expand": True, "print_time": False,
            "ipopt": {"max_iter": 2000, "tol": 1e-8, "constr_viol_tol": 1e-8,
                      "acceptable_tol": 1e-6, "linear_solver": "mumps",
                      "print_level": 5 if verbose else 0, **warm, **cfg.ipopt}}
    solver = ca.nlpsol("flip", "ipopt", prob, opts)
    r = solver(x0=np.concatenate(nlp.x0), lbx=np.concatenate(nlp.lbx),
               ubx=np.concatenate(nlp.ubx), lbg=np.concatenate(nlp.lbg),
               ubg=np.concatenate(nlp.ubg))
    stats = solver.stats()

    # unpack through one Function so the scaled-variable bookkeeping lives in exactly one place
    outs = []
    for ph, Vp in zip(cfg.phases, V):
        nodes = Vp["nodes"]
        cols = [Vp["T"]] + [ca.horzcat(*[nd[k] for nd in nodes]) for k in ("q", "v", "a", "u")]
        cols += [ca.horzcat(*[nd["lam"][c] for nd in nodes]) for c in ph.contacts]
        outs += cols
    outs.append(impulse)
    vals = ca.Function("unpack", [x], outs)(r["x"])
    vals = [np.array(v) for v in vals]
    sol, i = dict(phases=[], cfg=cfg), 0
    for ph in cfg.phases:
        d = dict(name=ph.name, contacts=ph.contacts, T=float(vals[i].ravel()[0]), q=vals[i + 1].T,
                 v=vals[i + 2].T, a=vals[i + 3].T, u=vals[i + 4].T, lam={})
        i += 5
        for c in ph.contacts:
            d["lam"][c] = vals[i].T
            i += 1
        sol["phases"].append(d)
    sol["impulse"] = vals[i].ravel()
    g = np.array(r["g"]).ravel()
    viol = np.maximum(np.concatenate(nlp.lbg) - g, g - np.concatenate(nlp.ubg)).clip(0)
    sol.update(success=bool(stats["success"]), status=stats["return_status"],
               iters=stats["iter_count"], cost=float(r["f"]), max_violation=float(viol.max()),
               worst_constraint=nlp.gname[int(viol.argmax())])
    return sol
