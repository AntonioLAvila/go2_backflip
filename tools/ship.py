"""Promote a solved checkpoint to the tracked reference trajectory in traj_opt/reference/.

traj_opt/out/ is scratch and gitignored -- 35 MB of bursts and logs, most of it worthless, and
a solve is not bit-reproducible across machines anyway (IPOPT's linear algebra is threaded and
hardware-dependent). So the best trajectory has to live in the repo as an artifact, not as a
recipe. This script is how it gets there.

It refuses to ship a checkpoint that audits worse than the one already shipped, on the same
(checks failed, worst overrun) key restart_loop ranks bursts by -- see STATUS.md 2026-09-04 for
why that key and not max_violation. Pass --force to override, and say why in --note.

    uv run tools/ship.py traj_opt/out/checkpoints_backflip_k50/best_10of11.npy
"""

from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "traj_opt"))

import audit                                                    # noqa: E402
from program import BackflipProgram                             # noqa: E402
from schedule import PHASES                                     # noqa: E402
from solve_backflip import (extract, max_violation, resample,    # noqa: E402
                            result_from_vector, save)

REF = Path(__file__).resolve().parent.parent / "traj_opt" / "reference"
MANIFEST = REF / "manifest.json"


def load_manifest() -> dict | None:
    return json.loads(MANIFEST.read_text()) if MANIFEST.exists() else None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("checkpoint", type=Path, help="decision-variable vector to promote")
    ap.add_argument("--force", action="store_true",
                    help="ship even though it audits worse than what is already shipped")
    ap.add_argument("--note", default="", help="one line recorded in the manifest")
    args = ap.parse_args()

    bp = BackflipProgram()
    x = np.load(args.checkpoint)
    if x.shape != (bp.prog.num_vars(),):
        raise SystemExit(f"{args.checkpoint}: {x.shape[0]} variables, this program has "
                         f"{bp.prog.num_vars()} -- a different schedule or formulation")

    result = result_from_vector(bp, x)
    viol = max_violation(bp.prog, result)
    phases = extract(bp, result)
    print(f"[{args.checkpoint.name}] viol={viol:.4f}")
    a = audit.run(bp, result, phases)

    old = load_manifest()
    if old is not None:
        was = (old["checks_failed"], old["worst_overrun"])
        print(f"currently shipped: {11 - was[0]}/11 checks, worst {was[1]:.2f}x over")
        if a.key > was and not args.force:
            raise SystemExit("refusing to ship: this audits WORSE than what is already "
                             "shipped. Use --force with a --note if that is deliberate.")

    REF.mkdir(exist_ok=True)
    t, xs, u = resample(phases)
    save(t, xs, u, REF / "backflip.npz")
    np.save(REF / "backflip.npy", x)
    MANIFEST.write_text(json.dumps({
        "source_checkpoint": str(args.checkpoint),
        "commit": subprocess.run(["git", "rev-parse", "HEAD"], capture_output=True,
                                 text=True).stdout.strip(),
        "flight_knots": PHASES[2].n_knots,
        "max_violation": round(viol, 6),
        "checks_passed": len(a.checks) - a.n_failed,
        "checks_failed": a.n_failed,
        "worst_overrun": round(a.worst, 4),
        "failed": [c.name for c in a.checks if not c.ok],
        "margins": {c.name: round(c.margin, 4) for c in a.checks},
        "note": args.note,
    }, indent=2) + "\n")
    print(f"shipped to {REF}/  ({len(a.checks) - a.n_failed}/{len(a.checks)} checks, "
          f"worst {a.worst:.2f}x over)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
