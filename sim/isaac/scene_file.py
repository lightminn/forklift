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
import math
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
    pickup_nominal: PalletSite | None = None  # the slot the zone and candidates come from (recorded, not read by v1)


def exit_walls(scene: SceneFile, band_bottom_m: float) -> tuple[Rectangle, ...]:
    """The transport exit prefix's obstacles (docs/plans/2026-10-22-riibotics-trailer-exit.md D1): the walls Isaac
    draws and collides that reach the obstacle layer's band (top at or above its bottom, so the 1.05 m LiDAR plane
    meets them). Floor patches and the static pallet copies are not walls; the props stay the planner's."""
    return tuple(v.rectangle for v in scene.visuals if v.kind == "wall" and v.height_m >= band_bottom_m)


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
        pickup_nominal=PalletSite(**record["pickup_nominal"]) if record.get("pickup_nominal") else None,
    )


# --- sessions: several pallets and unloading slots in one scene (docs/plans/2026-10-22-session-scene-pallet-props.md) --
SESSION_FORMAT = "riibotics_isaac_session_v1"


@dataclass(frozen=True)
class SessionZone:
    """Where a pallet's centre may stand (the operator's "around here") and its heading range about yaw_centre_rad."""

    x_min_m: float
    x_max_m: float
    y_min_m: float
    y_max_m: float
    yaw_centre_rad: float
    yaw_half_rad: float

    def centre_zone(self) -> PickupZone:
        """The runner's PickupZone (its yaw bound is absolute about 0: v1's zones are centred there)."""
        return PickupZone(self.x_min_m, self.x_max_m, self.y_min_m, self.y_max_m, self.yaw_half_rad)


def zone_rect(zone: PickupZone | SessionZone, depth_m: float, width_m: float) -> Rectangle:
    """The obstacle a pallet somewhere in ``zone`` is held as: the centre range grown by the pallet's half diagonal on
    every side, axis-aligned (the runner's pickup prior, any heading)."""
    grow = math.hypot(depth_m, width_m) / 2
    return Rectangle((zone.x_min_m + zone.x_max_m) / 2, (zone.y_min_m + zone.y_max_m) / 2,
                     zone.x_max_m - zone.x_min_m + 2 * grow, zone.y_max_m - zone.y_min_m + 2 * grow, 0.0)


@dataclass(frozen=True)
class SessionPallet:
    pallet_id: str
    nominal: PalletSite  # the slot as drawn (heading with its tilt): zone and candidates come from it
    truth: PalletSite  # where it really stands at the start (simulation only)
    registered: PalletSite  # the targets list's pose and the candidates' relocation reference
    zone: SessionZone
    observation_waypoints: tuple[tuple[float, float, float], ...]


@dataclass(frozen=True)
class SessionScene:
    name: str
    pallet: str  # the session's one pallet shape
    sha256: str
    start_rear: Pose2D
    bounds: Bounds
    static_props: tuple[PlacedProp, ...]  # what the planner avoids besides the pallets
    pallets: tuple[SessionPallet, ...]
    destinations: tuple[tuple[str, PalletSite], ...]
    sequence: tuple[tuple[str, str], ...]  # the drawing's order (the selection rule is CT-2's)
    visuals: tuple[SceneVisual, ...]
    source_format: str  # SESSION_FORMAT, or FORMAT for a v1 scene converted

    def pallet_by_id(self, pallet_id: str) -> SessionPallet:
        return next(p for p in self.pallets if p.pallet_id == pallet_id)

    def destination(self, destination_id: str) -> PalletSite:
        return dict(self.destinations)[destination_id]


def _waypoints(raw, where: str) -> tuple[tuple[float, float, float], ...]:
    out = tuple(tuple(float(c) for c in w) for w in raw)
    if not out or any(len(w) != 3 for w in out):
        raise ValueError(f"{where}: observation waypoints must be x, y, yaw triples")
    return out


def _check_session(name: str, pallets, destinations, sequence) -> None:
    ids = [p.pallet_id for p in pallets] + [d for d, _ in destinations]
    if len(set(ids)) != len(ids):
        raise ValueError(f"{name}: pallet and destination ids must be unique together (one id list per session)")
    pallet_ids, destination_ids = {p.pallet_id for p in pallets}, {d for d, _ in destinations}
    picked = [p for p, _ in sequence]
    dropped = [d for _, d in sequence]
    if not set(picked) <= pallet_ids or not set(dropped) <= destination_ids:
        raise ValueError(f"{name}: the sequence names a pallet or destination the session does not have")
    if len(set(picked)) != len(picked) or len(set(dropped)) != len(dropped):
        raise ValueError(f"{name}: the sequence picks a pallet or fills a destination twice")


def _site(raw) -> PalletSite:
    return PalletSite(float(raw["x_m"]), float(raw["y_m"]), float(raw["yaw_rad"]))


def load_session(path: Path) -> SessionScene:
    raw = Path(path).read_bytes()
    record = json.loads(raw)
    if record.get("format") != SESSION_FORMAT:
        raise ValueError(f"{path}: not a {SESSION_FORMAT} scene file")
    shape = str(record["pallet"])
    pallets = []
    for p in record["pallets"]:
        if p.get("pallet", shape) != shape:
            raise ValueError(f"{path}: pallet {p['pallet_id']} is a {p['pallet']}, the session's shape is {shape}")
        nominal = _site(p["nominal"])
        pallets.append(SessionPallet(str(p["pallet_id"]), nominal, _site(p["truth"]), nominal,
                                     SessionZone(**{k: float(v) for k, v in p["zone"].items()}),
                                     _waypoints(p["observation_waypoints"], f"{path} {p['pallet_id']}")))
    destinations = tuple((str(d["destination_id"]), _site(d)) for d in record["destinations"])
    sequence = tuple((str(a), str(b)) for a, b in record["sequence"])
    _check_session(str(path), pallets, destinations, sequence)
    visuals = []
    for v in record["visuals"]:
        if v["kind"] not in ("wall", "floor"):
            raise ValueError(f"{path}: a session's visuals are walls and floors (its pallets come from pallets[].truth)")
        visuals.append(SceneVisual(v["kind"], Rectangle(v["x_m"], v["y_m"], v["length_m"], v["width_m"], v["yaw_rad"]),
                                   float(v["height_m"]), tuple(float(c) for c in v["colour"])))
    return SessionScene(
        name=str(record["name"]), pallet=shape, sha256=hashlib.sha256(raw).hexdigest(),
        start_rear=Pose2D(**record["start_rear"]), bounds=Bounds(**record["bounds"]),
        static_props=tuple(PlacedProp(AssetSpec(**p["asset"]), Rectangle(**p["rectangle"])) for p in record["static_props"]),
        pallets=tuple(pallets), destinations=destinations, sequence=sequence, visuals=tuple(visuals),
        source_format=SESSION_FORMAT)


def register_session(targets, session: SessionScene, *, kind: str, depth_m: float, width_m: float, live: bool) -> None:
    """Every pallet and destination of a session into the targets list (plan 2026-10-22-session-scene-pallet-props
    2.3): each pallet at its registered pose with its real size (a pallet without one would draw no teleop zone) under
    the runner's shape key, then each destination. A converted v1 scene makes the runner's own calls, in its order:
    P-1 (``at`` D-0 live), D-1, then live D-0 = P-1's place. A session's first places are not destinations (CT-2)."""
    from forklift_core.mission.targets import Pose

    v1 = session.source_format == FORMAT
    for p in session.pallets:
        r = p.registered
        targets.add_pallet(p.pallet_id, kind, Pose(float(r.x_m), float(r.y_m), float(r.yaw_rad)),
                           depth_m=float(depth_m), width_m=float(width_m),
                           at="D-0" if live and v1 and p.pallet_id == "P-1" else None)
    for destination_id, site in session.destinations:
        targets.add_destination(destination_id, Pose(float(site.x_m), float(site.y_m), float(site.yaw_rad)))
    if live and v1:
        r = session.pallet_by_id("P-1").registered
        targets.add_destination("D-0", Pose(float(r.x_m), float(r.y_m), float(r.yaw_rad)))


def session_from_scene(scene: SceneFile) -> SessionScene:
    """A v1 scene as a one-pallet session, everything a v1 run reads kept as it is: P-1's registered pose and relocation
    reference are the scenario's pickup (as the runner registers it), its candidates and zone the file's (yaw centred at
    0), D-1 the scenario's destination; the static props are the scene's props, its neighbour pallets included (v1's
    neighbours are props -- the compatibility exception). The nominal is recorded only."""
    s = scene.scenario
    z = scene.pickup_zone
    nominal = scene.pickup_nominal if scene.pickup_nominal is not None else s.pickup
    p1 = SessionPallet("P-1", nominal, s.pickup, s.pickup,
                       SessionZone(z.x_min_m, z.x_max_m, z.y_min_m, z.y_max_m, 0.0, z.yaw_max_rad),
                       scene.observation_waypoints)
    return SessionScene(name=scene.name, pallet=scene.pallet, sha256=scene.sha256, start_rear=s.start_rear,
                        bounds=s.bounds, static_props=s.props, pallets=(p1,), destinations=(("D-1", s.destination),),
                        sequence=(("P-1", "D-1"),), visuals=scene.visuals, source_format=FORMAT)
