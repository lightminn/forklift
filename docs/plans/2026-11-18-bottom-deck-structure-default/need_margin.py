"""How far each line/random scene's bounds fall short of the bottom-deck side stage's room: the stands and retreats
(bottom_deck_legs) with the planning margin and travel allowance (bottom_deck_legs.side_bounds), for the pallet at
its scene pose and moved by up to +-0.15 m (the target tolerance) and +-5 deg (the far estimate standing in for it).

The hall frame's +y side is the scene frame's -x side (tools/riibotics_isaac_scenes.to_hall: bounds.y_max =
-b.x_min + ty, the translation set by b.y_max and the start only), so room on +y is the generator's min(xs) margin.

    python need_margin.py <repo>
"""

import itertools
import math
import sys
from pathlib import Path

ROOT = Path(sys.argv[1]).resolve()
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "tools"), str(ROOT / "sim/isaac")]
import riibotics_isaac_scenes as R  # noqa: E402
from scene_file import scenario_from_record  # noqa: E402

from forklift_core.mission import bottom_deck_legs as L  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.planning import Bounds  # noqa: E402

pallet = load_pallet_geometry(ROOT / "config/pallet_geometry_epal6.yaml")
geo = R.mission_geometry(R.URDF, pallet, R.RESERVE_M, **R.STRAIGHTS)
fp = geo.unloaded_footprint
margin = min(R.CLEARANCE_M, geo.approach_gap_m / 2)
PERTURB = list(itertools.product((-0.15, 0.0, 0.15), (-0.15, 0.0, 0.15), (-5.0, 0.0, 5.0)))
built = R.scenes()
for name in sorted(n for n in built if n.startswith(("line", "random"))):
    rec = R.scene_record(name, built[name])
    sc = scenario_from_record(rec["scenario"])
    pk = sc.pickup
    z = rec["pickup_zone"]
    zone = Bounds(z["x_min_m"], z["x_max_m"], z["y_min_m"], z["y_max_m"])
    b = sc.bounds
    worst = {"x-": 0.0, "x+": 0.0, "y-": 0.0, "y+": 0.0}
    nominal = None
    for dx, dy, dyaw in PERTURB:
        poses = []
        for f in (1, -1):
            st = L.stand((pk.x_m + dx, pk.y_m + dy), pk.yaw_rad + math.radians(dyaw), "x", f, mount_x_m=0.619,
                         rear_axle_offset_m=0.34)
            poses += [st.rear, L.retreat_pose(st.rear)]
        need = L.side_bounds(zone, R.HALL, poses, fp, margin)
        extra = {"x-": max(0.0, b.x_min_m - need.x_min_m), "x+": max(0.0, need.x_max_m - b.x_max_m),
                 "y-": max(0.0, b.y_min_m - need.y_min_m), "y+": max(0.0, need.y_max_m - b.y_max_m)}
        if (dx, dy, dyaw) == (0.0, 0.0, 0.0):
            nominal = extra
        worst = {k: max(worst[k], extra[k]) for k in worst}
    print(f"{name:10s} hall bounds y_max {b.y_max_m:.2f}  extra nominal "
          + " ".join(f"{k} {v:.2f}" for k, v in nominal.items())
          + "  worst over +-0.15 m, +-5 deg " + " ".join(f"{k} {v:.2f}" for k, v in worst.items()))
