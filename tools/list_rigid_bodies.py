"""What can move in a run's exported stage, with its world height range (SLAM localization plan
docs/plans/2026-10-20-slam-localization-session.md L2: what can move in a survey's scene and reach the LiDAR plane).

Opens ``<run>/scene.usda`` (the runner's root layer, its own edits included -- the layout's props have their rigid
bodies removed, ``sim/isaac/scene.py``) in a headless Isaac app, composes it and writes to JSON ``rigid_bodies``:
every prim with an enabled rigid body, kinematic ones too (a script can drive them), and ``time_varying``: every prim
with a time-sampled attribute (an animation; its names under ``sampled``); each with its path, the top-level prim under
/World it belongs to and the z range of its world bound at the default time (for an animated prim that says nothing
about its height over time). This is the stage at the export, not a history of moves. Run with the Isaac Python:

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

        def row(prim, **extra):
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            path = str(prim.GetPath())
            parts = path.split("/")
            return {"path": path, "root": "/".join(parts[:3]) if len(parts) > 2 else path,
                    "z_min_m": None if box.IsEmpty() else float(box.GetMin()[2]),
                    "z_max_m": None if box.IsEmpty() else float(box.GetMax()[2]), **extra}

        rigid, varying = [], []
        for prim in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies()):
            if prim.HasAPI(UsdPhysics.RigidBodyAPI):
                body = UsdPhysics.RigidBodyAPI(prim)
                if body.GetRigidBodyEnabledAttr().Get() is not False:
                    rigid.append(row(prim, kinematic=bool(body.GetKinematicEnabledAttr().Get())))
            # any time-sampled attribute (transform, points, extent, visibility...): an animation, whatever its bound
            # at the default time says (Codex S2 review round 2)
            sampled = [a.GetName() for a in prim.GetAttributes() if a.GetNumTimeSamples() > 0]
            if sampled:
                varying.append(row(prim, sampled=sampled))
        with open(out, "w") as handle:
            json.dump({"scene": scene, "rigid_bodies": rigid, "time_varying": varying}, handle, indent=1)
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
