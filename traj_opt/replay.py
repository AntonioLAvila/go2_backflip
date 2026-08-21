"""Play traj_opt/out/backflip.npz in meshcat.

    PYTHONPATH=/opt/drake/lib/python3.12/site-packages:tools:traj_opt \
        .venv/bin/python traj_opt/replay.py [--speed 0.25] [--loops 3]
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
from pydrake.geometry import Meshcat, MeshcatVisualizer
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder

import go2_constants as K

NPZ = Path(__file__).resolve().parent / "out" / "backflip.npz"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=0.25)
    ap.add_argument("--loops", type=int, default=3)
    ap.add_argument("--npz", type=Path, default=NPZ)
    args = ap.parse_args()

    d = np.load(args.npz)
    t, qpos = d["t"], d["qpos"]

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    Parser(plant).AddModels(K.MODEL_PATH)
    plant.Finalize()
    meshcat = Meshcat()
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    diagram = builder.Build()
    ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(ctx)

    print(f"meshcat: {meshcat.web_url()}   {t[-1]:.3f} s at {args.speed}x")
    for _ in range(args.loops):
        wall = time.time()
        for i in range(t.size):
            plant.SetPositions(plant_ctx, K.mj_to_drake_q(qpos[i]))
            ctx.SetTime(t[i])
            diagram.ForcedPublish(ctx)
            lag = t[i] / args.speed - (time.time() - wall)
            if lag > 0:
                time.sleep(lag)
        time.sleep(0.5)


if __name__ == "__main__":
    main()
