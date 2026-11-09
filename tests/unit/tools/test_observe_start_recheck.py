"""tools/observe_start_recheck.py: the observe-replan start check rebuilt from records (results 2026-11-10 4)."""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))

import observe_start_recheck as O  # noqa: E402

from forklift_core.planning.geometry import Bounds, Footprint, Rectangle  # noqa: E402

FP = Footprint(1.29, 0.17, 0.36)
BIG = Bounds(-20.0, 20.0, -20.0, 20.0)


def test_a_rectangle_ahead_blocks_at_the_clearance_and_backing_off_frees_it():
    # the footprint's front edge at x = 1.29; a box whose near face is 0.05 m ahead of it
    box = ("prior:P-5", Rectangle(1.29 + 0.05 + 0.5, 0.0, 1.0, 1.0, 0.0))
    out = O.recheck([box], FP, BIG, (0.0, 0.0, 0.0), 0.10)
    assert out["free_at_zero"] and not out["free_at_clearance"]
    assert out["blockers"] == ["prior:P-5"]
    assert out["max_free_clearance_m"] == pytest.approx(0.05, abs=1e-3)
    assert out["advice_back"]["backoff_m"] == pytest.approx(0.05, abs=0.011)
    assert out["advice_forward"]["backoff_m"] is None


def test_the_bounds_are_named_when_they_block():
    out = O.recheck([], FP, Bounds(-0.2, 1.33, -1.0, 1.0), (0.0, 0.0, 0.0), 0.10)
    assert out["blockers"] == ["bounds"] and not out["free_at_clearance"]


def test_a_start_inside_an_obstacle_is_not_free_at_zero():
    out = O.recheck([("box", Rectangle(0.5, 0.0, 1.0, 1.0, 0.0))], FP, BIG, (0.0, 0.0, 0.0), 0.10)
    assert not out["free_at_zero"] and out["max_free_clearance_m"] is None


def test_the_search_props_leave_out_the_slot_stage_and_grow_only_tall_props():
    scenario = {"props": [
        {"asset": {"uri": "riibotics:slot_marker", "height_m": 1.3}, "rectangle": {"x_m": 1, "y_m": 2, "length_m": 0.9,
                                                                                  "width_m": 0.03, "yaw_rad": 0}},
        {"asset": {"uri": "session:pallet:P-4", "height_m": 0.144}, "rectangle": {"x_m": 0, "y_m": 0, "length_m": 1,
                                                                                 "width_m": 1.2, "yaw_rad": 0}},
        {"asset": {"uri": "rack@slot", "height_m": 2.0}, "rectangle": {"x_m": 5, "y_m": 5, "length_m": 1, "width_m": 1}}]}
    props = dict(O.props_of(scenario, keepout_m=0.15))
    assert set(props) == {"riibotics:slot_marker", "session:pallet:P-4"}
    assert props["riibotics:slot_marker"].length_m == pytest.approx(1.2)  # 1.3 m tall: grown 0.15 each side
    assert props["session:pallet:P-4"].length_m == pytest.approx(1.0)  # under the plane: as drawn


def test_an_open_start_reports_a_lower_bound_not_a_silent_cap():
    out = O.recheck([], FP, BIG, (0.0, 0.0, 0.0), 0.10)
    assert out["max_free_clearance_m"] > 1.0  # Codex review P3: the old search stopped at 1 m
    clear, capped = O.max_free_clearance(lambda pose, c: True, (0, 0, 0))
    assert capped and clear == 64.0


def _run(tmp_path, *, layout="scene", grid=False, approach_before=False, carried_before=False):
    import json

    run = tmp_path / "run"
    transitions = [{"time_s": 1.0, "to": "observe"}, {"time_s": 40.0, "to": "observe"}]
    missions = {"m2": {"pallet_id": "P-5", "destination_id": "D-2", "outcome": "failed", "started_s": 38.0,
                       "ended_s": 61.0}}
    if approach_before:
        transitions.append({"time_s": 50.0, "to": "approach"})
    if carried_before:
        missions["m1"] = {"pallet_id": "P-5", "destination_id": "D-9", "outcome": "done", "started_s": 1.0,
                          "ended_s": 30.0}
    (run / "missions").mkdir(parents=True)
    for mid, res in missions.items():
        (run / "missions" / mid).mkdir()
        (run / "missions" / mid / "result.json").write_text(json.dumps({"result": res}))
    scenario = {"props": [{"asset": {"uri": "session:pallet:P-4", "height_m": 0.144},
                           "rectangle": {"x_m": -5.0, "y_m": 5.0, "length_m": 1.0, "width_m": 1.2, "yaw_rad": 0.0}}],
                "bounds": {"x_min_m": -10, "x_max_m": 10, "y_min_m": -10, "y_max_m": 10}}
    (run / "missions" / "m2" / "records.json").write_text(json.dumps({"state": {
        "mission_scenario": scenario,
        "observe_replans": [{"time_s": 60.0, "rear_pose": [0.0, 0.0, 0.0], "status": "invalid_start"}]}}))
    (run / "result.json").write_text(json.dumps({
        "arguments": {"layout": layout, "grid_planning": grid}, "transitions": transitions,
        "geometry": {"unloaded_footprint": {"front_m": 1.29, "rear_m": 0.17, "half_width_m": 0.36},
                     "pallet_depth_m": 0.6, "pallet_width_m": 0.8},
        "planner_config": {"clearance_m": 0.1}}))
    return run


def _fake_scene(monkeypatch):
    import types

    zone = types.SimpleNamespace(x_min_m=1.8, x_max_m=2.3, y_min_m=-0.25, y_max_m=0.25)
    fake = types.ModuleType("scene_file")
    fake.load_session = lambda path: types.SimpleNamespace(pallets=[types.SimpleNamespace(pallet_id="P-5", zone=zone)])
    fake.zone_rect = lambda z, d, w: Rectangle((z.x_min_m + z.x_max_m) / 2, (z.y_min_m + z.y_max_m) / 2,
                                               z.x_max_m - z.x_min_m + 2 * 0.5, z.y_max_m - z.y_min_m + 2 * 0.5, 0.0)
    monkeypatch.setitem(sys.modules, "scene_file", fake)


def test_the_inputs_are_read_from_the_records_and_the_scene(tmp_path, monkeypatch):
    _fake_scene(monkeypatch)
    inp = O.inputs_of(_run(tmp_path), "m2", tmp_path / "scene.json")
    assert inp["pallet_id"] == "P-5" and inp["clearance_m"] == 0.1 and inp["pose"] == [0.0, 0.0, 0.0]
    assert inp["zone"] == pytest.approx([2.05, 0.0, 1.5, 1.5, 0.0])
    assert [label for label, _ in inp["rects"]] == ["session:pallet:P-4", "prior:P-5"]
    out = O.recheck(inp["rects"], inp["footprint"], inp["bounds"], inp["pose"], inp["clearance_m"])
    assert out["blockers"] == ["prior:P-5"] and out["free_at_zero"]  # the prior's near face 0.01 m past the front
    assert out["max_free_clearance_m"] == pytest.approx(1.30 - 1.29, abs=1e-3)  # front 1.29, zone face at 1.30


@pytest.mark.parametrize("kw, why", [
    ({"layout": "factory"}, "--layout scene"),
    ({"grid": True}, "--layout scene"),
    ({"approach_before": True}, "approach"),
    ({"carried_before": True}, "carried"),
])
def test_a_record_this_does_not_rebuild_is_refused(tmp_path, monkeypatch, kw, why):
    _fake_scene(monkeypatch)
    with pytest.raises(O.Unsupported, match=why):
        O.inputs_of(_run(tmp_path, **kw), "m2", tmp_path / "scene.json")
