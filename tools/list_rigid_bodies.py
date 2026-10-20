"""The rigid bodies of a run's exported stage, with their world height range (SLAM localization plan
docs/plans/2026-10-20-slam-localization-session.md L2: what can move in a survey's scene and reach the LiDAR plane).

Opens ``<run>/scene.usda`` (the runner's root layer, its own edits included -- the layout's props have their rigid
bodies removed, ``sim/isaac/scene.py``) in a headless Isaac app, composes it and writes every prim with an enabled,
non-kinematic rigid body to a JSON list: path, the top-level prim under /World it belongs to, z range of its world
bound. Run with the Isaac Python:

    VENV_PY tools/list_rigid_bodies.py RUN/scene.usda OUT.json
"""

from __future__ import annotations

import json
import sys


def main(argv=None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    scene, out = argv
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "extra_args": ["--/audio/deviceBackend=null", "--/app/audio/enabled=false"]})
    try:
        from pxr import Usd, UsdGeom, UsdPhysics

        stage = Usd.Stage.Open(scene)
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        rows = []
        for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
                continue
            body = UsdPhysics.RigidBodyAPI(prim)
            enabled, kinematic = body.GetRigidBodyEnabledAttr().Get(), body.GetKinematicEnabledAttr().Get()
            if enabled is False or kinematic:
                continue
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            path = str(prim.GetPath())
            parts = path.split("/")
            rows.append({"path": path, "root": "/".join(parts[:3]) if len(parts) > 2 else path,
                         "z_min_m": None if box.IsEmpty() else float(box.GetMin()[2]),
                         "z_max_m": None if box.IsEmpty() else float(box.GetMax()[2])})
        with open(out, "w") as handle:
            json.dump({"scene": scene, "rigid_bodies": rows}, handle, indent=1)
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
