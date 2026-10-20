"""Scene files for run_transport.py --layout scene (tools/riibotics_isaac_scenes.py, sim/isaac/scene_file.py)."""

import json
import math
import sys
from pathlib import Path

import pytest

from forklift_core.planning import Footprint, Pose2D
from forklift_core.planning.geometry import Rectangle, collision_free_pose

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from scene_file import load_scene  # noqa: E402

from tools import riibotics_isaac_scenes as tool  # noqa: E402

SCENES = ("aisle", "aisle_out", "trailer", "line_1", "line_2", "line_3", "random_1", "random_2", "random_3")
# The planner's unloaded footprint (run_transport: axle to fork tip, rear 0.17, half-width 0.36)
TRUCK = Footprint(1.29, 0.17, 0.36)


@pytest.fixture(scope="module")
def written(tmp_path_factory):
    out = tmp_path_factory.mktemp("scenes")
    built = tool.scenes()
    assert tuple(built) == ("aisle", "aisle_out", "trailer") + tuple(
        name for k in range(3) for name in (f"line_{k + 1}", f"random_{k + 1}"))
    paths = {}
    for name, scene in built.items():
        paths[name] = out / f"{name}.json"
        paths[name].write_text(json.dumps(tool.scene_record(name, scene)))
    return {name: load_scene(path) for name, path in paths.items()}


def _rect_free(rect: Rectangle, others, bounds) -> bool:
    return collision_free_pose(Pose2D(rect.x_m, rect.y_m, rect.yaw_rad), others,
                               Footprint(rect.length_m / 2, rect.length_m / 2, rect.width_m / 2), bounds)


@pytest.mark.parametrize("name", SCENES)
def test_every_scene_stands_in_the_empty_south_hall(written, name):
    b = written[name].scenario.bounds
    assert tool.HALL.x_min_m <= b.x_min_m < b.x_max_m <= tool.HALL.x_max_m
    assert tool.HALL.y_min_m <= b.y_min_m < b.y_max_m <= tool.HALL.y_max_m


@pytest.mark.parametrize("name", SCENES)
def test_the_pickup_enters_along_x_inside_its_zone_and_the_start_is_free(written, name):
    scene = written[name]
    s, z = scene.scenario, scene.pickup_zone
    assert z.x_min_m < s.pickup.x_m < z.x_max_m and z.y_min_m < s.pickup.y_m < z.y_max_m
    assert abs(s.pickup.yaw_rad) <= z.yaw_max_rad
    obstacles = [p.rectangle for p in s.props]
    assert collision_free_pose(s.start_rear, obstacles, TRUCK, s.bounds, margin_m=0.10)
    # the pallets the mission handles stand clear of every prop
    for site in (s.pickup, s.destination):
        assert _rect_free(Rectangle(site.x_m, site.y_m, 0.6, 0.8, site.yaw_rad), obstacles, s.bounds)


@pytest.mark.parametrize("name", SCENES)
def test_every_pallet_prop_is_drawn_as_a_pallet_and_no_two_pallets_overlap(written, name):
    scene = written[name]
    pallets = [v.rectangle for v in scene.visuals if v.kind == "pallet"]
    props = [p.rectangle for p in scene.scenario.props if "pallet" in p.asset.uri]
    for rect in props:
        assert any(all(math.isclose(getattr(rect, f), getattr(v, f), abs_tol=1e-9)
                       for f in ("x_m", "y_m", "length_m", "width_m", "yaw_rad")) for v in pallets)
    for i, rect in enumerate(pallets):
        assert _rect_free(rect, pallets[:i] + pallets[i + 1:], scene.scenario.bounds)
    # a pallet drawn but not in the props stands inside a block the planner avoids
    blocks = [p.rectangle for p in scene.scenario.props if p.asset.uri == "riibotics:block"]
    for rect in pallets:
        if rect not in props:
            assert not _rect_free(rect, blocks, scene.scenario.bounds)


@pytest.mark.parametrize("name", SCENES)
def test_zone_and_candidates_come_from_the_nominal_slot_not_the_pallet(written, name):
    scene = written[name]
    s, z = scene.scenario, scene.pickup_zone
    cx, cy = (z.x_min_m + z.x_max_m) / 2, (z.y_min_m + z.y_max_m) / 2
    x, y, yaw = scene.observation_waypoints[0]
    assert yaw == 0.0 and math.isclose(math.hypot(cx - x, cy - y), 3.4, abs_tol=1e-3) and math.isclose(y, cy, abs_tol=1e-4)
    # the pallet really stands off the nominal slot
    assert math.hypot(s.pickup.x_m - cx, s.pickup.y_m - cy) > 0.02 or abs(s.pickup.yaw_rad) > 0.03
    # the start and every candidate clear the zone the runner plans on before recognition (grown by the
    # pallet's half diagonal), with the planning clearance
    zone = Rectangle(cx, cy, z.x_max_m - z.x_min_m + 1.0, z.y_max_m - z.y_min_m + 1.0, 0.0)
    obstacles = [p.rectangle for p in s.props] + [zone]
    assert collision_free_pose(s.start_rear, obstacles, TRUCK, s.bounds, margin_m=0.10)
    assert all(collision_free_pose(Pose2D(*w), obstacles, TRUCK, s.bounds, margin_m=0.10)
               for w in scene.observation_waypoints[:4])


def test_sequences_carry_the_delivered_pallets_and_the_waiting_ones(written):
    for prefix, waiting in (("line", 2), ("random", 4)):
        for k in range(3):
            props = [p.asset.uri for p in written[f"{prefix}_{k + 1}"].scenario.props]
            assert sum("delivered_pallet" in u for u in props) == k
            assert sum("loading_pallet" in u for u in props) == waiting - k


def test_the_trailer_walls_and_the_rack_frame_reach_the_lidar_plane(written):
    for name, kind in (("trailer", "wall"), ("aisle", "wall")):
        walls = [v for v in written[name].visuals if v.kind == kind]
        assert walls and all(v.height_m > 1.05 for v in walls)


def test_a_scene_file_is_refused_with_another_format_or_an_unknown_kind(tmp_path, written):
    record = tool.scene_record("aisle", tool.scenes()["aisle"])
    bad = tmp_path / "bad.json"
    bad.write_text(json.dumps({**record, "format": "other"}))
    with pytest.raises(ValueError, match="not a"):
        load_scene(bad)
    bad.write_text(json.dumps({**record, "visuals": [{**record["visuals"][0], "kind": "tree"}]}))
    with pytest.raises(ValueError, match="unknown visual kind"):
        load_scene(bad)


def _runner_args(extra):
    import run_transport

    argv = sys.argv
    sys.argv = ["run_transport.py", "--base-scene", "scene.usda", "--output", "out", "--seed", "0",
                "--forklift-urdf", str(ROOT / "sim/models/dls08_measured/forklift.urdf"),
                "--pallet-urdf", str(ROOT / "sim/models/epal6_pallet/pallet.urdf"),
                "--pallet-geometry", str(ROOT / "config/pallet_geometry_epal6.yaml"),
                "--settings", str(ROOT / "config/isaac_transport_measured.yaml"), *extra]
    if "--no-layer" in sys.argv:
        sys.argv.remove("--no-layer")
    else:  # the scene run's mission configuration: perception, SLAM record, obstacle layer acting
        sys.argv += ["--use-perception", "--planning-target", "perception", "--record-slam",
                     "--pallet-prior", str(ROOT / "config/pallet_prior_epal6.yaml"),
                     "--obstacle-layer", str(ROOT / "config/obstacle_layer_single.yaml"), "--obstacle-act"]
    try:
        return run_transport.arguments()
    finally:
        sys.argv = argv


def test_the_runner_takes_a_scene_files_observation_candidates_and_no_runtime_viewpoints(tmp_path):
    path = tmp_path / "aisle.json"
    record = tool.scene_record("aisle", tool.scenes()["aisle"])
    path.write_text(json.dumps(record))
    args = _runner_args(["--layout", "scene", "--scene-file", str(path)])
    assert args.observation_waypoints == record["observation_waypoints"]
    assert args.runtime_viewpoints is False
    assert _runner_args([]).runtime_viewpoints is True   # the bay as before


@pytest.mark.parametrize("extra", [["--layout", "scene"], ["--scene-file", "x.json"],
                                   ["--min-obstacle-height-m", "1.15"], ["--prism-colliders"],
                                   ["--grid-planning"], ["--no-layer"]])
def test_the_runner_refuses_scene_runs_that_would_change_or_hide_the_scene(tmp_path, extra):
    path = tmp_path / "aisle.json"
    path.write_text(json.dumps(tool.scene_record("aisle", tool.scenes()["aisle"])))
    base = [] if extra[0] in ("--layout", "--scene-file") else ["--layout", "scene", "--scene-file", str(path)]
    with pytest.raises(SystemExit):
        _runner_args(base + extra)
    _runner_args(base)   # the same arguments without the refused one are taken


def test_the_near_field_contract_knows_the_scene_file_argument():
    import near_tracking

    assert near_tracking.POST_MEASUREMENT_ARGUMENTS["scene_file"] is None


@pytest.mark.parametrize("pickup_prior", [True, False])
def test_a_scene_runs_cusp_check_never_reads_the_true_pickup(pickup_prior):
    """MissionCore.cusp_leg_checker with a scene run's options (no grid planning, pickup_prior): the zone before
    recognition, the estimate after it -- moving only the true pallet changes nothing (Codex 2026-10-20 P2).
    Without pickup_prior (older records, bay without grid) it still reads the truth, as before."""
    from types import SimpleNamespace

    from forklift_core.mission.core import MissionCore
    from forklift_core.planning.pallet_mission import PalletSite, SyntheticMissionGeometry

    scene = tool.scene_record("line_1", tool.scenes()["line_1"])
    from scene_file import scenario_from_record

    scenario = scenario_from_record(scene["scenario"])
    zone = Rectangle(scenario.pickup.x_m, 0.0, 1.5, 1.5, 0.0)
    estimate = PalletSite(scenario.pickup.x_m - 0.1, 0.1, 0.05)
    poses = [(scenario.pickup.x_m - d, y, 0.0) for d in (1.5, 1.7, 1.9, 2.1) for y in (-0.6, 0.0, 0.6)]

    def checks(truth, known):
        sc = __import__("dataclasses").replace(scenario, pickup=truth)
        fake = SimpleNamespace(
            escape_leg=None, geometry=SyntheticMissionGeometry(), ms=SimpleNamespace(phase="observe"),
            options=SimpleNamespace(grid_planning=False, pickup_prior=pickup_prior, pickup_bounds=None,
                                    pickup_zone=zone, scenario=sc),
            hooks=SimpleNamespace(grid_world=lambda s: s), obstacle={"pickup_estimate": known},
            observe_bounds=lambda: None)
        checker = MissionCore.cusp_leg_checker(fake)
        return [checker.free(p, 0.10) for p in poses]

    moved = PalletSite(scenario.pickup.x_m - 0.6, 0.5, 0.3)
    for known in (None, estimate):
        same = checks(scenario.pickup, known) == checks(moved, known)
        assert same is pickup_prior
