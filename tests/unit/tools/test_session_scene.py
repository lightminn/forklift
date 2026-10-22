"""Session scenes (docs/plans/2026-10-22-session-scene-pallet-props.md 2.1-2.2): several pallets and unloading slots in
one scene, the nominal/truth rule, the loader's refusals and v1 as a one-pallet session."""

import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from scene_file import (  # noqa: E402
    SESSION_FORMAT,
    load_scene,
    load_session,
    session_from_scene,
    zone_rect,
)

from tools import riibotics_isaac_scenes as tool  # noqa: E402


@pytest.fixture(scope="module")
def sessions(tmp_path_factory):
    out = tmp_path_factory.mktemp("sessions")
    paths = {}
    for kind in ("line", "random"):
        paths[kind] = out / f"{kind}_session.json"
        paths[kind].write_text(json.dumps(tool.session_record(kind)))
    return {k: load_session(p) for k, p in paths.items()}, paths


@pytest.fixture(scope="module")
def v1(tmp_path_factory):
    out = tmp_path_factory.mktemp("v1")
    scenes = {}
    for name, built in tool.scenes().items():
        if True:  # every v1 scene (the adapter's equivalence covers them all)
            path = out / f"{name}.json"
            path.write_text(json.dumps(tool.scene_record(name, built)))
            scenes[name] = load_scene(path)
    return scenes


def test_a_session_holds_every_pallet_and_slot_with_the_drawings_order(sessions):
    loaded, _ = sessions
    line, random = loaded["line"], loaded["random"]
    assert [p.pallet_id for p in line.pallets] == ["P-1", "P-2", "P-3"]
    assert [d for d, _ in line.destinations] == ["D-1", "D-2", "D-3"]
    assert line.sequence == (("P-1", "D-1"), ("P-2", "D-2"), ("P-3", "D-3"))
    assert [p.pallet_id for p in random.pallets] == ["P-1", "P-2", "P-3", "P-4", "P-5"]
    assert random.sequence == (("P-4", "D-1"), ("P-5", "D-2"), ("P-3", "D-3"))
    assert line.static_props == () and random.static_props == () and line.source_format == SESSION_FORMAT
    assert all(v.kind in ("wall", "floor") for s in loaded.values() for v in s.visuals)


def test_the_nominal_carries_the_drawings_tilt_and_the_truth_its_own_error(sessions):
    loaded, _ = sessions
    tilts = [round(math.degrees(p.nominal.yaw_rad), 6) for p in loaded["random"].pallets]
    assert tilts == [0.0, 0.0, -8.0, 10.0, -6.0]
    errors = {"P-4": tool.PICKUP_ERRORS["random_1"], "P-5": tool.PICKUP_ERRORS["random_2"],
              "P-3": tool.PICKUP_ERRORS["random_3"], "P-1": (0.04, -0.03, 1.0), "P-2": (-0.03, 0.04, -1.0)}
    for p in loaded["random"].pallets:
        dx, dy, dyaw = errors[p.pallet_id]
        assert p.truth.x_m == pytest.approx(p.nominal.x_m + dx) and p.truth.y_m == pytest.approx(p.nominal.y_m + dy)
        assert p.truth.yaw_rad == pytest.approx(p.nominal.yaw_rad + math.radians(dyaw))
        assert p.registered == p.nominal  # a session registers its pallets where they were drawn
        assert p.zone.yaw_centre_rad == p.nominal.yaw_rad and p.zone.yaw_half_rad == pytest.approx(math.radians(12))
        assert p.zone.x_max_m - p.zone.x_min_m == pytest.approx(0.5)
        assert [list(w) for w in p.observation_waypoints] == tool.observation_waypoints(p.nominal)


@pytest.mark.parametrize("kind, k", [("line", 0), ("line", 1), ("line", 2), ("random", 0), ("random", 1), ("random", 2)])
def test_session_mission_k_is_v1_scene_k_with_the_tilt_in_the_nominal(sessions, v1, kind, k):
    loaded, _ = sessions
    s, scene = loaded[kind], v1[f"{kind}_{k + 1}"]
    pid, did = s.sequence[k]
    p, dest = s.pallet_by_id(pid), s.destination(did)
    assert (p.truth.x_m, p.truth.y_m) == pytest.approx((scene.scenario.pickup.x_m, scene.scenario.pickup.y_m))
    assert p.truth.yaw_rad == pytest.approx(scene.scenario.pickup.yaw_rad)
    assert (dest.x_m, dest.y_m, dest.yaw_rad) == pytest.approx(
        (scene.scenario.destination.x_m, scene.scenario.destination.y_m, scene.scenario.destination.yaw_rad))
    assert (p.nominal.x_m, p.nominal.y_m) == pytest.approx((scene.pickup_nominal.x_m, scene.pickup_nominal.y_m))
    assert scene.pickup_nominal.yaw_rad == 0.0  # v1's nominal is unturned; the session's has the tilt
    assert (s.start_rear, s.bounds) == (scene.scenario.start_rear, scene.scenario.bounds)
    # the neighbours v1 drew as props stand on the session's nominal slots (v1's had no error)
    for prop in scene.scenario.props:
        r = prop.rectangle
        assert any(math.hypot(r.x_m - q.nominal.x_m, r.y_m - q.nominal.y_m) < 1e-9 for q in s.pallets) or any(
            math.hypot(r.x_m - d.x_m, r.y_m - d.y_m) < 1e-9 for _, d in s.destinations)


def _record(path):
    return json.loads(Path(path).read_text())


@pytest.mark.parametrize("change, message", [
    (lambda r: r.update(format="x"), "not a"),
    (lambda r: r["pallets"][1].update(pallet="t11_06"), "session's shape"),
    (lambda r: r["destinations"][0].update(destination_id="P-1"), "unique together"),
    (lambda r: r["pallets"][1].update(pallet_id="P-1"), "unique together"),
    (lambda r: r.update(sequence=[["P-1", "D-1"], ["P-1", "D-2"]]), "twice"),
    (lambda r: r.update(sequence=[["P-1", "D-1"], ["P-2", "D-1"]]), "twice"),
    (lambda r: r.update(sequence=[["P-9", "D-1"]]), "does not have"),
    (lambda r: r["visuals"].append({**r["visuals"][0], "kind": "pallet"}), "walls and floors"),
])
def test_the_loader_refuses_mixed_shapes_shared_ids_and_bad_orders(sessions, tmp_path, change, message):
    _, paths = sessions
    record = _record(paths["line"])
    change(record)
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps(record))
    with pytest.raises(ValueError, match=message):
        load_session(bad)


def test_v1_as_a_session_keeps_everything_a_v1_run_reads(v1):
    scene = v1["random_1"]
    s = session_from_scene(scene)
    p1 = s.pallet_by_id("P-1")
    assert p1.registered == scene.scenario.pickup and p1.truth == scene.scenario.pickup  # the truth registration kept
    assert p1.nominal == scene.pickup_nominal and p1.observation_waypoints == scene.observation_waypoints
    z = scene.pickup_zone
    assert (p1.zone.x_min_m, p1.zone.x_max_m, p1.zone.y_min_m, p1.zone.y_max_m) == (
        z.x_min_m, z.x_max_m, z.y_min_m, z.y_max_m)
    assert p1.zone.centre_zone() == z and p1.zone.yaw_centre_rad == 0.0
    assert s.static_props == scene.scenario.props and s.destination("D-1") == scene.scenario.destination
    assert s.sequence == (("P-1", "D-1"),) and s.visuals == scene.visuals


def test_the_zone_obstacle_is_the_runners_pickup_prior():
    from forklift_core.planning.observation_viewpoints import PickupZone

    r = zone_rect(PickupZone(1.0, 1.5, -0.25, 0.25, 0.2), 0.8, 0.6)
    assert (r.x_m, r.y_m, r.length_m, r.width_m, r.yaw_rad) == pytest.approx((1.25, 0.0, 1.5, 1.5, 0.0))


ALL_V1 = ("aisle", "aisle_out", "trailer", "conveyor", "line_1", "line_2", "line_3", "random_1", "random_2", "random_3")


@pytest.mark.parametrize("name", ALL_V1)
@pytest.mark.parametrize("live", [False, True])
def test_registering_a_v1_session_makes_the_runners_own_targets(v1, live, name):
    """Plan 2.3: the same ids, poses, sizes, at and revision as run_transport's registration (5395-5409)."""
    from forklift_core.mission.targets import Pose, TargetsBook

    from scene_file import register_session

    scene = v1[name]
    s = scene.scenario
    runner = TargetsBook()
    runner.add_pallet("P-1", "pallet_geometry_epal6", Pose(float(s.pickup.x_m), float(s.pickup.y_m),
                                                          float(s.pickup.yaw_rad)),
                      depth_m=1.2, width_m=0.8, at="D-0" if live else None)
    runner.add_destination("D-1", Pose(float(s.destination.x_m), float(s.destination.y_m),
                                       float(s.destination.yaw_rad)))
    if live:
        runner.add_destination("D-0", Pose(float(s.pickup.x_m), float(s.pickup.y_m), float(s.pickup.yaw_rad)))
    ours = TargetsBook()
    register_session(ours, session_from_scene(scene), kind="pallet_geometry_epal6", depth_m=1.2, width_m=0.8, live=live)
    assert ours.pallets == runner.pallets and ours.destinations == runner.destinations
    assert ours.revision == runner.revision and ours._issued == runner._issued


def test_a_session_registers_every_pallet_with_its_size_at_its_drawn_slot(sessions):
    from forklift_core.mission.targets import TargetsBook

    from scene_file import register_session

    loaded, _ = sessions
    tb = TargetsBook()
    register_session(tb, loaded["random"], kind="pallet_geometry_epal6", depth_m=1.2, width_m=0.8, live=True)
    assert sorted(tb.pallets) == ["P-1", "P-2", "P-3", "P-4", "P-5"] and sorted(tb.destinations) == ["D-1", "D-2", "D-3"]
    for p in loaded["random"].pallets:
        entry = tb.pallets[p.pallet_id]
        assert (entry.pose.x_m, entry.pose.y_m, entry.pose.yaw_rad) == (p.nominal.x_m, p.nominal.y_m, p.nominal.yaw_rad)
        assert (entry.depth_m, entry.width_m, entry.basis, entry.at) == (1.2, 0.8, "prior", None)


def test_a_tilted_zone_has_no_runner_pickup_zone(sessions):
    loaded, _ = sessions
    with pytest.raises(ValueError, match="centred at 0"):
        loaded["random"].pallet_by_id("P-4").zone.centre_zone()


@pytest.mark.parametrize("name", ALL_V1)
def test_the_adapter_gives_the_runners_own_preparation_inputs_for_every_v1_scene(v1, name):
    """Plan 2.4 / 2.1 (3판 amendment): what run_transport builds for P-1 -- the truth pickup as the reference, the scene's
    candidates as the template, its zone grown into the pickup prior -- is what the v1 session adapter gives, so PR 3
    can build the inputs from the session without changing a v1 run."""
    from forklift_core.planning.observation_viewpoints import PickupZone  # noqa: F401

    scene = v1[name]
    s = scene.scenario
    depth, width = 0.6, 0.8
    z = scene.pickup_zone
    grow = math.hypot(depth, width) / 2  # run_transport 3468-3475
    runner = {"reference": (float(s.pickup.x_m), float(s.pickup.y_m), float(s.pickup.yaw_rad)),
              "template": [tuple(float(v) for v in w) for w in scene.observation_waypoints],
              "zone": ((z.x_min_m + z.x_max_m) / 2, (z.y_min_m + z.y_max_m) / 2, z.x_max_m - z.x_min_m + 2 * grow,
                       z.y_max_m - z.y_min_m + 2 * grow, 0.0)}
    p1 = session_from_scene(scene).pallet_by_id("P-1")
    r = zone_rect(p1.zone, depth, width)
    adapter = {"reference": (float(p1.registered.x_m), float(p1.registered.y_m), float(p1.registered.yaw_rad)),
               "template": [tuple(float(v) for v in w) for w in p1.observation_waypoints],
               "zone": (r.x_m, r.y_m, r.length_m, r.width_m, r.yaw_rad)}
    assert adapter == runner


def test_a_sessions_first_mission_is_the_runners_setup_scene(sessions):
    from scene_file import scene_format, session_as_scene

    loaded, paths = sessions
    assert scene_format(paths["random"]) == SESSION_FORMAT
    s = loaded["random"]
    scene = session_as_scene(s)
    p4 = s.pallet_by_id("P-4")
    assert scene.scenario.pickup == p4.truth and scene.scenario.destination == s.destination("D-1")
    assert scene.scenario.props == s.static_props and scene.scenario.start_rear == s.start_rear
    assert scene.observation_waypoints == p4.observation_waypoints and scene.pickup_nominal == p4.nominal
    assert all(v.kind != "pallet" for v in scene.visuals)
    z = scene.pickup_zone
    assert (z.x_min_m, z.x_max_m, z.yaw_max_rad) == (p4.zone.x_min_m, p4.zone.x_max_m, p4.zone.yaw_half_rad)
    assert session_as_scene(s, 2).scenario.pickup == s.pallet_by_id("P-3").truth
