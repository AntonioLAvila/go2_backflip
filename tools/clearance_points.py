"""Sphere-swept witness points bounding every collision geom, for the TO's floor constraints.

The trajectory optimization only ever knew about four point-feet, so nothing in it stopped the
rest of the robot from passing through the floor -- and it did: the solved trajectory drove the
rear thigh 69 mm and the head 72 mm below z = 0. Drake's exact tool (a SceneGraph +
MinimumDistanceLowerBoundConstraint) means building the plant with geometry and paying a
broadphase query per knot per autodiff evaluation, which is far more machinery than a flat
floor needs.

A flat floor only needs the LOWEST point of each geom, and every collision geom in go2.xml is
sphere-swept: sphere, capsule, cylinder (bounded by its capsule), or box (lowest point is
always a corner, radius 0). So each geom becomes a handful of (point, radius) pairs in its
body frame, and the whole constraint is `p_z(q) >= r`.

The NLP pins quat_x = quat_z = 0, so the base has pitch only -- no roll, no yaw. World z of a
body point is then independent of its y, which collapses every +-y mirror pair to one witness
point and nearly halves the set. VERIFY below checks the emitted table against MuJoCo's own
geom poses over random configurations, including the no-roll collapse.

    uv run tools/clearance_points.py
"""

from __future__ import annotations

import itertools

import mujoco
import numpy as np

from go2_backflip import constants as K

BODIES = ["base", "FL_hip", "FL_thigh", "FL_calf", "RL_hip", "RL_thigh", "RL_calf",
          "FR_hip", "FR_thigh", "FR_calf", "RR_hip", "RR_thigh", "RR_calf"]
N_VERIFY = 4000
TOL = 1e-9


def _R(quat):
    return K.quat_wxyz_to_R(np.asarray(quat, float))


def geom_spheres(m, g) -> list[tuple[np.ndarray, float]]:
    """(point, radius) pairs in the parent body frame whose union contains geom g."""
    pos, R, s = m.geom_pos[g], _R(m.geom_quat[g]), m.geom_size[g]
    t = m.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_SPHERE:
        return [(pos.copy(), float(s[0]))]
    if t in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        # A cylinder's rim sits inside the capsule of the same radius, so this is a bound.
        half = R @ np.array([0.0, 0.0, s[1]])
        return [(pos + half, float(s[0])), (pos - half, float(s[0]))]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        return [(pos + R @ (np.array(c) * s[:3]), 0.0)
                for c in itertools.product((-1, 1), repeat=3)]
    raise ValueError(f"unhandled geom type {t}")


def by_kind(table) -> dict[str, list[tuple[np.ndarray, float]]]:
    """Collapse the four legs onto one entry per body KIND.

    Every hip/thigh/calf carries the same geoms at the same place in its OWN body frame, so
    one table per kind describes all four legs. Asserted, not assumed. (The hip cylinder's y
    offset flips sign left/right, which cannot change a world z without roll, so the assert
    compares x, z and radius only.)
    """
    out = {"base": table["base"]}
    for kind in ("hip", "thigh", "calf"):
        ref = table[f"FL_{kind}"]
        for leg in K.FEET:
            got = table[f"{leg}_{kind}"]
            assert len(got) == len(ref), f"{leg}_{kind} has {len(got)} points, FL has {len(ref)}"
            for (p, r), (q, rq) in zip(sorted(got, key=lambda pr: tuple(pr[0])),
                                       sorted(ref, key=lambda pr: tuple(pr[0]))):
                assert abs(p[0] - q[0]) < 1e-12 and abs(p[2] - q[2]) < 1e-12 and abs(r - rq) < 1e-12, \
                    f"{leg}_{kind} geometry differs from FL_{kind}"
        out[kind] = ref
    return out


def collapse(pts: list[tuple[np.ndarray, float]]) -> list[tuple[np.ndarray, float]]:
    """Drop +-y mirror duplicates, which share a world z when the body cannot roll or yaw."""
    keep: list[tuple[np.ndarray, float]] = []
    for p, r in pts:
        if any(abs(p[0] - q[0]) < 1e-9 and abs(p[2] - q[2]) < 1e-9 and abs(r - rq) < 1e-9
               for q, rq in keep):
            continue
        keep.append((p, r))
    return keep


def build(m) -> dict[str, list[tuple[np.ndarray, float]]]:
    out: dict[str, list[tuple[np.ndarray, float]]] = {b: [] for b in BODIES}
    for g in range(m.ngeom):
        if m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0:
            continue                                    # visual-only mesh
        if m.geom_type[g] == mujoco.mjtGeom.mjGEOM_PLANE:
            continue
        body = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
        if body in out:
            out[body] += geom_spheres(m, g)
    return {b: collapse(p) for b, p in out.items()}


def true_lowest(m, data, g) -> float:
    """Exact lowest world z of geom g, straight from MuJoCo's own geom pose."""
    p, R, s, t = data.geom_xpos[g], data.geom_xmat[g].reshape(3, 3), m.geom_size[g], m.geom_type[g]
    if t == mujoco.mjtGeom.mjGEOM_SPHERE:
        return p[2] - s[0]
    if t in (mujoco.mjtGeom.mjGEOM_CAPSULE, mujoco.mjtGeom.mjGEOM_CYLINDER):
        half = abs((R @ np.array([0.0, 0.0, s[1]]))[2])
        return p[2] - half - s[0]
    if t == mujoco.mjtGeom.mjGEOM_BOX:
        return p[2] - np.abs(R[2] * s[:3]).sum()
    raise ValueError(t)


def verify(m, table) -> None:
    """The witness set must never report more clearance than the geoms actually have."""
    data = mujoco.MjData(m)
    rng = np.random.default_rng(0)
    lim = np.array([m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]
                    for n in K.JOINT_NAMES])
    gids = [g for g in range(m.ngeom)
            if m.geom_type[g] != mujoco.mjtGeom.mjGEOM_PLANE
            and not (m.geom_contype[g] == 0 and m.geom_conaffinity[g] == 0)]
    bid = {b: m.body(b).id for b in BODIES}
    worst = np.inf
    for _ in range(N_VERIFY):
        legs = rng.uniform(lim[:, 0], lim[:, 1])
        legs[K.HIP_IDX] = 0.0                                   # sagittal, as the NLP pins it
        legs[3:6], legs[9:12] = legs[0:3], legs[6:9]            # mirrored, as the NLP pins it
        pitch = rng.uniform(-np.pi, np.pi)
        data.qpos[:] = K.mj_qpos(legs, rng.uniform(0.1, 0.6),
                                 (np.cos(pitch / 2), 0.0, np.sin(pitch / 2), 0.0))
        mujoco.mj_kinematics(m, data)

        witness = min((data.xpos[bid[b]] + data.xmat[bid[b]].reshape(3, 3) @ p)[2] - r
                      for b, pts in table.items() for p, r in pts)
        exact = min(true_lowest(m, data, g) for g in gids)
        worst = min(worst, exact - witness)
    print(f"  verify: witness set is conservative by at least {worst:+.3e} m "
          f"over {N_VERIFY} random sagittal poses "
          f"({'OK' if worst > -TOL else 'FAILED -- witness set MISSES geometry'})")
    assert worst > -TOL


def emit(kinds) -> None:
    print("\nCOLLISION_SPHERES = {")
    for b, pts in kinds.items():
        print(f'    "{b}": np.array([')
        for p, r in sorted(pts, key=lambda pr: (-pr[0][2], pr[0][0])):
            print(f"        ({p[0]:+.5f}, {p[1]:+.5f}, {p[2]:+.5f}, {r:.5f}),")
        print("    ]),")
    print("}")
    n = len(kinds["base"]) + 2 * sum(len(kinds[k]) for k in ("hip", "thigh", "calf"))
    print(f"\n# {n} witness points per knot once the base and the two sagittally-distinct "
          f"legs are expanded.")


def main() -> None:
    m = mujoco.MjModel.from_xml_path(K.MODEL_PATH)
    table = build(m)
    for b, pts in table.items():
        print(f"  {b:>10s}: {len(pts)} witness points")
    verify(m, table)
    emit(by_kind(table))


if __name__ == "__main__":
    main()
