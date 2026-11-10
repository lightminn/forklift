"""Heights of the warehouse's light-holding top-level parts and of everything wholly above 3 m (read-only)."""
import json
import sys

from pxr import Usd, UsdGeom, UsdLux

stage = Usd.Stage.Open(sys.argv[1])
root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
rows = []
for prim in root.GetChildren():
    lights = [str(p.GetPath()) for p in Usd.PrimRange(prim) if p.HasAPI(UsdLux.LightAPI)]
    box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
    if box.IsEmpty():
        continue
    rows.append({"path": str(prim.GetPath()), "zmin": box.GetMin()[2], "zmax": box.GetMax()[2],
                 "xy": [box.GetMin()[0], box.GetMax()[0], box.GetMin()[1], box.GetMax()[1]],
                 "lights": len(lights)})
# light-holding parts: geometry below them other than the lights
lit = [r for r in rows if r["lights"]]
detail = []
for r in lit:
    prim = stage.GetPrimAtPath(r["path"])
    for c in prim.GetChildren():
        b = cache.ComputeWorldBound(c).ComputeAlignedRange()
        if b.IsEmpty():
            continue
        detail.append({"path": str(c.GetPath()), "type": c.GetTypeName(), "zmin": b.GetMin()[2], "zmax": b.GetMax()[2],
                       "light": any(p.HasAPI(UsdLux.LightAPI) for p in Usd.PrimRange(c))})
print(json.dumps({"up": UsdGeom.GetStageUpAxis(stage), "mpu": UsdGeom.GetStageMetersPerUnit(stage),
                  "n_top": len(rows), "lit_top": lit, "lit_children_low": sorted(detail, key=lambda d: d["zmin"])[:40],
                  "below3_count": sum(1 for r in rows if r["zmin"] < 3.0),
                  "zmin_of_above3": min((r["zmin"] for r in rows if r["zmin"] >= 3.0), default=None)}, indent=1))
