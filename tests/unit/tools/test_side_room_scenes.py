"""The `_side` scene copies (docs/plans/2026-11-18-bottom-deck-structure-default.md section 2): the hall box grown on
+y so the bottom-deck side stands fit, nothing else moved, the originals untouched."""

import json
import sys
from pathlib import Path

import pytest

from forklift_core.mission import bottom_deck_legs as legs
from forklift_core.planning import Footprint

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "sim/isaac"))

from scene_file import load_scene  # noqa: E402

from tools import riibotics_isaac_scenes as tool  # noqa: E402

SEQUENCE_SCENES = tuple(f"{kind}_{k + 1}" for kind in ("line", "random") for k in range(3))
# The planner's unloaded footprint (run_transport: axle to fork tip, rear 0.17, half-width 0.36)
TRUCK = Footprint(1.29, 0.17, 0.36)


@pytest.fixture(scope="module")
def records():
    built = tool.scenes()
    out = {name: tool.scene_record(name, built[name]) for name in built}
    for name in SEQUENCE_SCENES:
        kind = name.split("_")[0]
        out[f"{name}_m"] = tool.scene_record(f"{name}_m", tool.with_slot_markers(kind, built[name]))
    return out


def _without_ymax(record):
    r = json.loads(json.dumps(record))  # as the files hold it (tuples as lists)
    del r["name"]
    del r["scenario"]["bounds"]["y_max_m"]
    r["source"].pop("side_room_m", None)  # the copy's own provenance, the one added field
    return r


@pytest.mark.parametrize("name", SEQUENCE_SCENES + tuple(f"{n}_m" for n in SEQUENCE_SCENES))
def test_a_side_copy_grows_only_the_hall_box_on_plus_y(records, name):
    original = records[name]
    side = tool.side_room_record(original)
    kind = name.split("_")[0]
    assert side["name"] == f"{name}_side"
    grown = side["scenario"]["bounds"]["y_max_m"] - original["scenario"]["bounds"]["y_max_m"]
    assert grown == pytest.approx(tool.SIDE_ROOM_M[kind], abs=1e-12)
    assert _without_ymax(side) == _without_ymax(original)
    assert side["source"]["side_room_m"] == tool.SIDE_ROOM_M[kind]
    assert "side_room_m" not in original["source"] and original["name"] == name  # the input is not changed in place


@pytest.mark.parametrize("name", SEQUENCE_SCENES)
def test_a_side_copy_stays_in_the_south_hall_and_loads(records, name, tmp_path):
    side = tool.side_room_record(records[name])
    path = tmp_path / f"{side['name']}.json"
    path.write_text(json.dumps(side))
    b = load_scene(path).scenario.bounds
    assert tool.HALL.x_min_m <= b.x_min_m < b.x_max_m <= tool.HALL.x_max_m
    assert tool.HALL.y_min_m <= b.y_min_m < b.y_max_m <= tool.HALL.y_max_m


@pytest.mark.parametrize("name", SEQUENCE_SCENES)
def test_the_side_stands_and_retreats_fit_the_copy(records, name, tmp_path):
    """Both faces' stands and retreats (x hypothesis, the measured mount) with the planning margin lie inside the
    copy's box -- the condition the side stage's planner checks first (invalid_goal otherwise)."""
    side = tool.side_room_record(records[name])
    path = tmp_path / f"{side['name']}.json"
    path.write_text(json.dumps(side))
    scenario = load_scene(path).scenario
    pk = scenario.pickup
    for face in (1, -1):
        stand = legs.stand((pk.x_m, pk.y_m), pk.yaw_rad, "x", face, mount_x_m=0.619, rear_axle_offset_m=0.34)
        for pose in (stand.rear, legs.retreat_pose(stand.rear)):
            assert legs.inside(scenario.bounds, pose, TRUCK, 0.10), (name, face, pose)


def test_only_the_sequence_scenes_have_side_copies(records):
    for name in ("aisle", "aisle_out", "trailer", "conveyor"):
        with pytest.raises(ValueError, match="side"):
            tool.side_room_record(records[name])
    with pytest.raises(ValueError, match="side"):
        tool.side_room_record(tool.side_room_record(records["line_1"]))  # never a copy of a copy


def test_the_cli_writes_side_copies_and_leaves_every_original_and_session_file_alone(tmp_path, monkeypatch):
    plain, side = tmp_path / "plain", tmp_path / "side"
    base = ["riibotics_isaac_scenes.py", "--slot-markers", "--rack", "--session", "line", "--session", "random"]
    monkeypatch.setattr(sys, "argv", [*base, "--output", str(plain)])
    tool.main()
    monkeypatch.setattr(sys, "argv", [*base, "--side-room", "--output", str(side)])
    tool.main()
    plain_files = {p.name: p.read_bytes() for p in plain.glob("*.json")}
    side_files = {p.name: p.read_bytes() for p in side.glob("*.json")}
    expected_copies = {f"{n}_side.json" for n in SEQUENCE_SCENES} | {f"{n}_m_side.json" for n in SEQUENCE_SCENES}
    assert set(side_files) == set(plain_files) | expected_copies
    for name, data in plain_files.items():  # originals (18 scenes) and the console's session files, byte for byte
        assert side_files[name] == data, name
    for name in expected_copies:
        record = json.loads(side_files[name])
        assert record["name"] == name.removesuffix(".json")


def test_only_filters_the_written_names_side_copies_included(tmp_path, monkeypatch):
    out = tmp_path / "only"
    monkeypatch.setattr(sys, "argv", ["riibotics_isaac_scenes.py", "--slot-markers", "--side-room",
                                      "--only", "line_2_m_side,aisle", "--output", str(out)])
    tool.main()
    assert sorted(p.name for p in out.glob("*.json")) == ["aisle.json", "line_2_m_side.json"]
    monkeypatch.setattr(sys, "argv", ["riibotics_isaac_scenes.py", "--only", "line_2_m_side", "--output", str(out)])
    with pytest.raises(SystemExit):  # without --slot-markers --side-room nothing writes that name
        tool.main()


def test_an_unknown_only_name_writes_nothing_not_even_the_session_files(tmp_path, monkeypatch):
    out = tmp_path / "typo"
    monkeypatch.setattr(sys, "argv", ["riibotics_isaac_scenes.py", "--session", "line", "--slot-markers",
                                      "--only", "typo", "--output", str(out)])
    with pytest.raises(SystemExit):
        tool.main()
    assert not out.exists() or not any(out.iterdir())
