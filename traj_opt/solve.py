"""Solve the backflip, audit it, track it in MuJoCo, and optionally ship it.

    uv run traj_opt/solve.py                       # -> traj_opt/out/backflip.npz
    uv run traj_opt/solve.py --torque-sf 0.85 --ship --note "more actuator headroom"

A solve is a cold start and takes well under a minute, so the reference is a *recipe* again:
`--ship` records the full Config in manifest.json, and re-running with it reproduces the tape.
Shipping requires IPOPT success, a clean audit, and a closed-loop MuJoCo landing.
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))

import flip                                                            # noqa: E402
import tape                                                            # noqa: E402
import validate                                                        # noqa: E402
import mj_track                                                        # noqa: E402

HERE = Path(__file__).resolve().parent
OUT, REF = HERE / "out", HERE / "reference"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    for f in dataclasses.fields(flip.Config):
        if f.type in ("float", float):
            ap.add_argument(f"--{f.name.replace('_', '-')}", type=float, default=f.default)
    ap.add_argument("--refine", type=float, default=1.0,
                    help="re-solve on a mesh this many times finer, warm-started from the first")
    ap.add_argument("--out", type=Path, default=OUT / "backflip.npz")
    ap.add_argument("--hz", type=float, default=500.0)
    ap.add_argument("--quiet", action="store_true")
    ap.add_argument("--ship", action="store_true")
    ap.add_argument("--note", default="")
    args = ap.parse_args()

    cfg = flip.Config(**{f.name: getattr(args, f.name) for f in dataclasses.fields(flip.Config)
                         if hasattr(args, f.name)})
    t0 = time.time()
    sol = flip.solve(cfg, verbose=not args.quiet)
    if args.refine != 1.0 and sol["success"]:
        print(f"coarse mesh: cost {sol['cost']:.4f} in {sol['iters']} iterations; refining x{args.refine:g}")
        cfg.phases = tuple(dataclasses.replace(p, n=round(p.n * args.refine)) for p in cfg.phases)
        sol = flip.solve(cfg, guess=tape.regrid(sol, cfg), verbose=not args.quiet)
    print(f"\nIPOPT: {sol['status']} in {sol['iters']} iterations, {time.time() - t0:.1f} s wall, "
          f"cost {sol['cost']:.4f}, max violation {sol['max_violation']:.1e}")
    print("phases: " + ", ".join(f"{p['name']} {p['T']:.3f} s" for p in sol["phases"])
          + f"   touchdown impulse per foot {np.round(sol['impulse'], 2)} N.s")

    d = tape.mujoco_tape(sol, args.hz)
    d["torque_sf"] = cfg.torque_sf
    args.out.parent.mkdir(exist_ok=True)
    np.savez(args.out, **d)
    print(f"wrote {args.out}  ({d['t'].size} samples, {d['t'][-1]:.3f} s)\n\naudit:")
    checks = validate.audit(d, mu=cfg.mu, clearance=cfg.body_clearance - 0.01)
    ok = validate.report(checks)

    print("\nclosed loop in MuJoCo (feedforward + joint PD, hardware torque-speed clipping):")
    log, n_ref, _ = mj_track.rollout(d, 60.0, 2.0)
    s = mj_track.score(log, n_ref, d)
    for k, v in s.items():
        print(f"  {k:16s} {v:.4g}" if isinstance(v, float) else f"  {k:16s} {v}")

    good = sol["success"] and ok and s["landed"] and not s["bad_contact"]
    if args.ship:
        if not good:
            print("\nNOT shipped: needs IPOPT success, a clean audit, and a clean landing.")
            return 1
        REF.mkdir(exist_ok=True)
        np.savez(REF / "backflip.npz", **d)
        commit = subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True, text=True,
                                cwd=HERE).stdout.strip()
        cfg_d = dataclasses.asdict(cfg)
        manifest = dict(
            commit=commit, note=args.note, hz=args.hz, config=cfg_d,
            ipopt=dict(status=sol["status"], iterations=sol["iters"], cost=sol["cost"],
                       max_violation=sol["max_violation"]),
            phases={p["name"]: p["T"] for p in sol["phases"]},
            audit={c.name: dict(value=c.value, limit=c.limit, unit=c.unit) for c in checks},
            closed_loop={k: v for k, v in s.items()},
        )
        (REF / "manifest.json").write_text(json.dumps(manifest, indent=2, default=float))
        print(f"\nshipped to {REF}")
    return 0 if good else 1


if __name__ == "__main__":
    sys.exit(main())
