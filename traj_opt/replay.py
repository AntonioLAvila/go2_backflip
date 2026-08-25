"""Play traj_opt/out/backflip.npz in meshcat.

    uv run traj_opt/replay.py [--fps 500] [--npz PATH]
    uv run traj_opt/replay.py --live [--speed 0.25] [--loops 3]

Default mode records the trajectory into a meshcat animation and publishes it, so playback
is driven by the browser's "Animations" panel -- scrub, pause, and drag `timeScale` to
replay at any speed without re-running this script. The process then holds open, because
the meshcat server dies with it; Ctrl-C when done (--no-hold to exit immediately).

--live restores the old behaviour: stream frames in real time at a fixed --speed. Useful
only when you want the wall-clock feel of a single pass; it leaves no scrubbable animation.
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np
from pydrake.geometry import (
    Meshcat,
    MeshcatAnimation,
    MeshcatVisualizer,
    MeshcatVisualizerParams,
)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder

from go2_backflip import constants as K

NPZ = Path(__file__).resolve().parent / "out" / "backflip.npz"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--npz", type=Path, default=NPZ)
    ap.add_argument("--fps", type=float, default=500.0,
                    help="recording frame rate; defaults to the 500 Hz output grid so every "
                         "knot gets its own frame. Lower values silently subsample -- at "
                         "drake's 64 Hz default only 95 of 736 knots survive, which can hide "
                         "the impact transient")
    ap.add_argument("--hold", action=argparse.BooleanOptionalAction, default=True,
                    help="keep the meshcat server alive after publishing (default: on)")
    ap.add_argument("--live", action="store_true",
                    help="stream in real time instead of recording an animation")
    ap.add_argument("--speed", type=float, default=0.25, help="--live playback rate")
    ap.add_argument("--loops", type=int, default=3, help="--live passes")
    args = ap.parse_args()

    d = np.load(args.npz)
    t, qpos = d["t"], d["qpos"]

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, 0.0)
    Parser(plant).AddModels(K.MODEL_PATH)
    plant.Finalize()
    meshcat = Meshcat()
    params = MeshcatVisualizerParams()
    params.publish_period = 1.0 / args.fps  # sets the recording's frame rate
    vis = MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat, params)
    diagram = builder.Build()
    ctx = diagram.CreateDefaultContext()
    plant_ctx = plant.GetMyMutableContextFromRoot(ctx)

    def publish_knot(i: int) -> None:
        plant.SetPositions(plant_ctx, K.mj_to_drake_q(qpos[i]))
        ctx.SetTime(t[i])
        diagram.ForcedPublish(ctx)

    if args.live:
        print(f"meshcat: {meshcat.web_url()}   {t[-1]:.3f} s at {args.speed}x (live)")
        for _ in range(args.loops):
            wall = time.time()
            for i in range(t.size):
                publish_knot(i)
                lag = t[i] / args.speed - (time.time() - wall)
                if lag > 0:
                    time.sleep(lag)
            time.sleep(0.5)
        return

    # Record every knot, then hand the whole animation to the browser at once. Transforms
    # are not pushed live during the sweep -- nothing is watching yet, and skipping them
    # makes the record pass essentially instant.
    vis.StartRecording(set_transforms_while_recording=False)
    for i in range(t.size):
        publish_knot(i)
    vis.StopRecording()

    rec = vis.get_mutable_recording()
    rec.set_loop_mode(MeshcatAnimation.LoopMode.kLoopRepeat)
    rec.set_autoplay(True)
    vis.PublishRecording()

    print(f"meshcat: {meshcat.web_url()}")
    print(f"recorded {t.size} knots -> {t[-1]:.3f} s at {args.fps:g} fps")
    print('open the URL, then use the "Animations" panel: play/pause, scrub, and set '
          "timeScale for speed")
    if args.hold:
        print("holding the meshcat server open -- Ctrl-C to exit")
        try:
            while True:
                time.sleep(0.5)
        except KeyboardInterrupt:
            print()


if __name__ == "__main__":
    main()
