#!/usr/bin/env python3
"""Does Isaac Sim reproduce the same depth image bit for bit?

Two questions, kept apart because they have different answers in general:

  within-run  a static scene captured twice, a few frames apart
  across-run  the same capture from a freshly started process

Each capture is reduced to a SHA-256 of the raw float32 depth array, so two
runs are compared by hash alone. The scene is deliberately minimal -- a ground
plane and one ``FixedCuboid`` -- so that nothing moves and any difference is
the renderer's, not the physics'. ``waited_steps`` is recorded because it is
itself a signal: two runs that fill the annotator after a different number of
steps were not in the same state, whatever the hashes say.

This probe writes only into ``--out``. It touches nothing in the repository.

    /opt/isaacsim/python.sh sim/isaac/determinism_probe.py --out /tmp/det --tag runA
    /opt/isaacsim/python.sh sim/isaac/determinism_probe.py --out /tmp/det --tag runB

⚠️ The interpreter path is part of the experiment. Two Isaac installs on the
remote carry the identical VERSION string ``5.1.0-rc.19+release.26219...`` and
only ``/opt/isaacsim`` runs; ``~/isaacsim`` dies importing ``isaacsim.core.api``
for want of ``typing_extensions``. Record the path with any result.

Measured 2026-09-17 on the remote (kang-MS-7D77, RTX 5070 12GB, driver
580.126.09, /opt/isaacsim 5.1.0-rc.19): all four captures hashed
``36e768b2b5ea2dba2329c522...``, finite 135360, sum 780902.875, waited 2 then 0
in both runs. Within-run and across-run were both bit-identical.

What that result does NOT cover, and what must be re-measured before it is
relied on: a moving or physics-driven scene (NVIDIA documents physics resume as
nondeterministic); a different machine, GPU, driver or Isaac build -- the team's
workstation is a 5070 Ti, the remote a 5070, and cross-machine agreement is
untested; the PathTracing renderer; and any sensor-noise path such as
``SingleViewDepthSensor``, which introduces its own RNG.
"""

import argparse
import hashlib
import json
import sys
from pathlib import Path

ap = argparse.ArgumentParser()
ap.add_argument("--out", type=Path, required=True)
ap.add_argument("--tag", required=True)
ap.add_argument("--warmup", type=int, default=60)
ap.add_argument("--gap", type=int, default=5)
ap.add_argument("--resolution", type=int, nargs=2, default=(640, 480))
args = ap.parse_args()
args.out.mkdir(parents=True, exist_ok=True)
width, height = args.resolution

# SimulationApp must be constructed before any other Isaac import.
from isaacsim import SimulationApp

app = SimulationApp(
    {
        "headless": True,
        "width": width,
        "height": height,
        "renderer": "RaytracedLighting",
        "multi_gpu": False,
    }
)

record = {
    "tag": args.tag,
    "config": {
        "renderer": "RaytracedLighting",
        "resolution": [width, height],
        "warmup": args.warmup,
        "gap": args.gap,
        "executable": sys.executable,
    },
    "captures": [],
    "errors": [],
}

try:
    import numpy as np
    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from isaacsim.core.api.objects import FixedCuboid, GroundPlane
    from isaacsim.sensors.camera import Camera

    world = World(stage_units_in_meters=1.0, physics_dt=1 / 120.0, rendering_dt=1 / 60.0)
    world.scene.add(GroundPlane(prim_path="/World/ground", size=20.0))
    # Fixed, not dynamic: no settling, no contact, no physics drift.
    world.scene.add(
        FixedCuboid(
            prim_path="/World/box",
            name="box",
            position=np.array([3.0, 0.0, 0.25]),
            scale=np.array([0.66, 0.66, 0.09]),
        )
    )
    cam = Camera(prim_path="/World/probe_cam", frequency=-1, resolution=(width, height))
    world.reset()
    cam.initialize()
    cam.set_world_pose(
        position=np.array([0.0, 0.0, 0.5]),
        orientation=np.array([0.5, -0.5, 0.5, 0.5]),  # +x forward, +z up
        camera_axes="usd",
    )
    cam.add_distance_to_image_plane_to_frame()

    for _ in range(args.warmup):
        world.step(render=True)
    record["frame_keys"] = sorted(str(k) for k in cam.get_current_frame().keys())

    def depth_now(tries=30):
        """The annotator key appears before its array does; step until it fills."""
        for i in range(tries):
            depth = cam.get_current_frame().get("distance_to_image_plane")
            if depth is not None and hasattr(depth, "__len__") and len(depth) > 0:
                return np.asarray(depth, dtype=np.float32), i
            try:
                rep.orchestrator.step(rt_subframes=1, delta_time=0.0, pause_timeline=False)
            except Exception:  # orchestrator is optional; stepping the world may suffice
                pass
            world.step(render=True)
        return None, tries

    def grab(label):
        depth, waited = depth_now()
        if depth is None:
            record["errors"].append(f"{label}: annotator never filled after {waited} steps")
            return
        finite = np.isfinite(depth)
        record["captures"].append(
            {
                "label": label,
                "waited_steps": waited,
                "sha256": hashlib.sha256(np.ascontiguousarray(depth).tobytes()).hexdigest(),
                "shape": list(depth.shape),
                "finite": int(finite.sum()),
                "min": float(depth[finite].min()) if finite.any() else None,
                "max": float(depth[finite].max()) if finite.any() else None,
                "sum": float(depth[finite].sum()) if finite.any() else None,
            }
        )

    grab("first")
    for _ in range(args.gap):
        world.step(render=True)
    grab("second")
except BaseException:
    import traceback

    record["errors"].append(traceback.format_exc())
finally:
    (args.out / f"{args.tag}.json").write_text(json.dumps(record, indent=2) + "\n")
    print(
        "PROBE_RESULT",
        json.dumps({k: v for k, v in record.items() if k != "errors"}),
        flush=True,
    )
    for error in record["errors"]:
        print("ERROR:", error, file=sys.stderr, flush=True)
    app.close()
