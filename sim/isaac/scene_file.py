"""Scene files for run_transport.py --layout scene (Isaac-free, so the CPU tools read the same).

A scene file (format riibotics_isaac_scene_v1, written by tools/riibotics_isaac_scenes.py) holds
one transport mission in the hall frame: the TransportScenario the planner sees (its props are
the scene's layout as a prior map), the pickup zone (the operator's "the pallet around here",
replacing the bay's placement zone), the observation candidates, and what Isaac draws:

  wall    a solid box (collides; tall ones are what the 1.05 m LiDAR plane meets)
  floor   a floor patch (drawn only, no collision)
  pallet  a static copy of the mission pallet (collides)
"""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path

from forklift_core.planning import Pose2D
from forklift_core.planning.geometry import Bounds, Rectangle
from forklift_core.planning.observation_viewpoints import PickupZone
from forklift_core.planning.pallet_mission import AssetSpec, PalletSite, PlacedProp, TransportScenario

FORMAT = "riibotics_isaac_scene_v1"
VISUAL_KINDS = ("wall", "floor", "pallet")


@dataclass(frozen=True)
class SceneVisual:
    kind: str
    rectangle: Rectangle
    height_m: float
    colour: tuple[float, float, float]


@dataclass(frozen=True)
class SceneFile:
    name: str
    pallet: str
    sha256: str
    scenario: TransportScenario
    pickup_zone: PickupZone
    observation_waypoints: tuple[tuple[float, float, float], ...]
    visuals: tuple[SceneVisual, ...]


def scenario_from_record(s: dict) -> TransportScenario:
    return TransportScenario(
        seed=int(s["seed"]),
        start_rear=Pose2D(**s["start_rear"]),
        pickup=PalletSite(**s["pickup"]),
        destination=PalletSite(**s["destination"]),
        props=tuple(PlacedProp(AssetSpec(**p["asset"]), Rectangle(**p["rectangle"])) for p in s["props"]),
        bounds=Bounds(**s["bounds"]),
    )


def load_scene(path: Path) -> SceneFile:
    raw = Path(path).read_bytes()
    record = json.loads(raw)
    if record.get("format") != FORMAT:
        raise ValueError(f"{path}: not a {FORMAT} scene file")
    visuals = []
    for v in record["visuals"]:
        if v["kind"] not in VISUAL_KINDS:
            raise ValueError(f"{path}: unknown visual kind {v['kind']!r}")
        visuals.append(SceneVisual(v["kind"], Rectangle(v["x_m"], v["y_m"], v["length_m"], v["width_m"], v["yaw_rad"]),
                                   float(v["height_m"]), tuple(float(c) for c in v["colour"])))
    waypoints = tuple(tuple(float(c) for c in w) for w in record["observation_waypoints"])
    if not waypoints or any(len(w) != 3 for w in waypoints):
        raise ValueError(f"{path}: observation waypoints must be x, y, yaw triples")
    return SceneFile(
        name=str(record["name"]),
        pallet=str(record["pallet"]),
        sha256=hashlib.sha256(raw).hexdigest(),
        scenario=scenario_from_record(record["scenario"]),
        pickup_zone=PickupZone(**record["pickup_zone"]),
        observation_waypoints=waypoints,
        visuals=tuple(visuals),
    )
