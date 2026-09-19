"""Resample a solved flip onto a uniform tape, in MuJoCo convention, for the stages downstream.

Inside one Hermite-Simpson interval the solution IS a polynomial -- q and v are the cubic
Hermite interpolants of their knot values and rates, and a, u, lambda are the quadratics through
their three nodes -- so the tape is that polynomial evaluated, not a fit to it. The velocity
jumps at touchdown; the sample after the impact carries the post-impact state.

The npz keeps the keys every existing consumer reads (t, qpos, qvel, ctrl) and adds what an
imitation/RL stage wants and should not have to reconstruct: the contact schedule, the planned
ground reaction forces, and the phase of every sample.
"""

from __future__ import annotations

import numpy as np

from go2_backflip import sagittal as S

FEET = ("FL", "FR", "RL", "RR")


def _hermite(s, h, x0, d0, x1, d1):
    s = s[:, None]
    return ((2 * s**3 - 3 * s**2 + 1) * x0 + (s**3 - 2 * s**2 + s) * h * d0
            + (-2 * s**3 + 3 * s**2) * x1 + (s**3 - s**2) * h * d1)


def _quad(s, y0, ym, y1):
    s = s[:, None]
    return (2 * (s - 0.5) * (s - 1)) * y0 + (-4 * s * (s - 1)) * ym + (2 * s * (s - 0.5)) * y1


def _eval_phase(ph, tau):
    """The phase's own polynomials at local times tau -> dict of q, v, a, u, lam{}."""
    n = (ph["q"].shape[0] - 1) // 2
    h = ph["T"] / n
    k = np.minimum((tau / h).astype(int), n - 1)
    s = tau / h - k
    a_, m_, b_ = 2 * k, 2 * k + 1, 2 * k + 2
    quad = lambda y: _quad(s, y[a_], y[m_], y[b_])                              # noqa: E731
    return dict(q=_hermite(s, h, ph["q"][a_], ph["v"][a_], ph["q"][b_], ph["v"][b_]),
                v=_hermite(s, h, ph["v"][a_], ph["a"][a_], ph["v"][b_], ph["a"][b_]),
                a=quad(ph["a"]), u=quad(ph["u"]),
                lam={c: quad(ph["lam"][c]) for c in ph["contacts"]})


def regrid(sol, cfg):
    """A solution resampled onto cfg's mesh, as a warm start. Mesh refinement is this plus a
    re-solve: a cold start on a finer mesh is free to land in a different local optimum, and on
    this problem it does."""
    out = []
    for ph, new in zip(sol["phases"], cfg.phases):
        e = _eval_phase(ph, np.linspace(0.0, ph["T"], 2 * new.n + 1))
        out.append(dict(T=ph["T"], **e))
    return out


def planar(sol, hz=500.0):
    """Uniform-rate planar tape: dict of t, q, v, a, u, lam_front, lam_rear, phase."""
    sag = S.build()
    T = np.array([p["T"] for p in sol["phases"]])
    edges = np.concatenate([[0.0], np.cumsum(T)])
    t = np.arange(0.0, edges[-1], 1.0 / hz)
    out = {k: np.zeros((t.size, n)) for k, n in
           (("q", 7), ("v", 7), ("a", 7), ("u", 4), ("lam_front", 2), ("lam_rear", 2))}
    out["t"], out["phase"] = t, np.zeros(t.size, int)
    for p, ph in enumerate(sol["phases"]):
        last = p == len(sol["phases"]) - 1
        sel = (t >= edges[p]) & ((t < edges[p + 1]) | last)
        e = _eval_phase(ph, t[sel] - edges[p])
        for key in ("q", "v", "a", "u"):
            out[key][sel] = e[key]
        for c in ph["contacts"]:
            out[f"lam_{c}"][sel] = e["lam"][c]
        out["phase"][sel] = p
    lf, lr = out["lam_front"], out["lam_rear"]
    out["u_hip"] = np.array([np.array(sag.hip(q, v, a, f, r)).ravel() for q, v, a, f, r in
                             zip(out["q"], out["v"], out["a"], lf, lr)])
    out["edges"] = edges
    return out


def mujoco_tape(sol, hz=500.0):
    """The shipped artifact: MuJoCo-convention arrays plus the contact plan."""
    pl = planar(sol, hz)
    n = pl["t"].size
    contact = np.zeros((n, 4), bool)
    grf = np.zeros((n, 4, 3))
    for p, ph in enumerate(sol["phases"]):
        sel = pl["phase"] == p
        for c in ph["contacts"]:
            for i in ((0, 1) if c == "front" else (2, 3)):
                contact[sel, i] = True
    for i, key in ((0, "lam_front"), (1, "lam_front"), (2, "lam_rear"), (3, "lam_rear")):
        grf[:, i, 0], grf[:, i, 2] = pl[key][:, 0], pl[key][:, 1]
    return dict(
        t=pl["t"],
        qpos=np.array([S.embed_qpos(q) for q in pl["q"]]),
        qvel=np.array([S.embed_qvel(v) for v in pl["v"]]),
        qacc=np.array([S.embed_qvel(a) for a in pl["a"]]),
        ctrl=np.array([S.embed_ctrl(u, h) for u, h in zip(pl["u"], pl["u_hip"])]),
        contact=contact, grf=grf, phase=pl["phase"],
        phase_names=np.array([p["name"] for p in sol["phases"]]),
        phase_edges=pl["edges"], feet=np.array(FEET),
    )
