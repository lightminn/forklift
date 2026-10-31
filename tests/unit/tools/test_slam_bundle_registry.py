"""The survey map bundle registry (live-default plan S3 D1/D2): accepted bundles only, found by the scene file's content
and the odometry model, each with its full map id and its start pose as map <- base_link."""

import json
import math

import numpy as np
import pytest

from tools import slam_bundle_registry as reg


def _manifest(scene_hash="s" * 64, model="team_spec_quantized", map_id="a" * 64):
    return {"map_id": map_id, "inputs": {"scene_file": {"path": "line_m_session.json", "sha256": scene_hash}},
            "run": {"arguments": {"layout": "scene", "encoder_model": model}}}


def _report(ok=True, world_ok=True, position=0.092, yaw=0.006, map_id="a" * 64):
    return {"ok": ok, "map_id": map_id, "world": {"ok": world_ok, "position_max_m": position, "yaw_max_rad": yaw}}


def test_the_start_pose_is_the_base_link_pose_whatever_its_heading():
    yaw = 2.5  # not 0: the quaternion's heading, not the rear axle's place
    row = np.array([1.0, -2.0, 0.1, math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)])
    x, y, h = reg.start_pose(row)
    assert (x, y) == (1.0, -2.0) and math.isclose(h, yaw)


def test_only_an_accepted_survey_enters_the_registry():
    e = reg.entry(_manifest(), _report(), folder="line_m_tsq_aaaaaaaaaaaa", start=[1.0, 2.0, 0.5], name="line_m")
    assert e["map_id"] == "a" * 64 and e["scene_sha256"] == "s" * 64 and e["encoder_model"] == "team_spec_quantized"
    assert e["start_pose"] == [1.0, 2.0, 0.5] and e["v2"] == {"position_max_m": 0.092, "yaw_max_rad": 0.006}
    for bad in (_report(ok=False), _report(world_ok=False)):
        with pytest.raises(ValueError, match="not accepted"):
            reg.entry(_manifest(), bad, folder="f", start=[0, 0, 0], name="line_m")
    with pytest.raises(ValueError, match="scene file"):
        reg.entry({**_manifest(), "inputs": {}}, _report(), folder="f", start=[0, 0, 0], name="factory")


def test_lookup_by_scene_content_and_odometry_model():
    a = reg.entry(_manifest(), _report(), folder="fa", start=[0, 0, 0], name="line_m")
    b = reg.entry(_manifest(scene_hash="t" * 64, map_id="b" * 64), _report(map_id="b" * 64), folder="fb", start=[0, 0, 0],
                  name="random_m")
    book = reg.put(reg.put({"bundles": []}, a), b)
    assert reg.lookup(book, "s" * 64, "team_spec_quantized")["folder"] == "fa"
    assert reg.lookup(book, "t" * 64, "team_spec_quantized")["folder"] == "fb"
    assert reg.lookup(book, "s" * 64, "reported") is None  # a bundle is bound to its odometry model
    assert reg.lookup(book, "u" * 64, "team_spec_quantized") is None
    # a new accepted survey of the same scene and model replaces the old row
    a2 = reg.entry(_manifest(map_id="c" * 64), _report(position=0.05, map_id="c" * 64), folder="fa2", start=[0, 0, 0], name="line_m")
    book = reg.put(book, a2)
    assert len(book["bundles"]) == 2 and reg.lookup(book, "s" * 64, "team_spec_quantized")["map_id"] == "c" * 64


def test_the_lookup_command_prints_what_the_launcher_reads(tmp_path, capsys):
    scene = tmp_path / "line_m_session.json"
    scene.write_text(json.dumps({"name": "line_m"}))
    row = reg.entry(_manifest(scene_hash=reg.sha256(scene)), _report(), folder="fa", start=[1.0, -2.0, 2.5],
                    name="line_m")
    book = tmp_path / "bundles.yaml"
    reg.write(book, reg.put({"bundles": []}, row))
    assert reg.main(["lookup", str(book), "--scene-file", str(scene), "--encoder-model", "team_spec_quantized"]) == 0
    assert capsys.readouterr().out.split() == ["fa", "a" * 64, "1.0,-2.0,2.5", "no"]
    assert reg.main(["lookup", str(book), "--scene-file", str(scene), "--encoder-model", "reported"]) == reg.NONE


def test_a_missing_registry_is_an_error_not_an_empty_one(tmp_path):
    # Codex implementation review P2: a lookup on a registry file that is not there read as "no row" -> mapping
    scene = tmp_path / "s.json"
    scene.write_text("{}")
    with pytest.raises(FileNotFoundError):
        reg.main(["lookup", str(tmp_path / "absent.yaml"), "--scene-file", str(scene), "--encoder-model", "reported"])
    assert reg.read(tmp_path / "absent.yaml", missing_ok=True) == {"bundles": []}  # what add starts from


def test_the_report_must_be_the_bundles_own():
    # Codex P2: an accepted report of survey A with the manifest of bundle B registered B
    with pytest.raises(ValueError, match="map_id"):
        reg.entry(_manifest(map_id="b" * 64), {**_report(), "map_id": "a" * 64}, folder="f", start=[0, 0, 0],
                  name="line_m")
    assert reg.entry(_manifest(), {**_report(), "map_id": "a" * 64}, folder="f", start=[0, 0, 0], name="line_m")
    with pytest.raises(ValueError, match="map_id"):  # round 2: a report without its id is not the bundle's either
        reg.entry(_manifest(), {**_report(), "map_id": None}, folder="f", start=[0, 0, 0], name="line_m")


def test_the_survey_run_and_nodes_must_be_the_reports_own(tmp_path):
    s, t = tmp_path / "s", tmp_path / "t"
    report = {**_report(), "map_id": "a" * 64, "epoch": str(s / "epoch_1"), "readback": str(s / "readback")}
    reg.check_sources(report, s / "epoch_1" / "run", s / "readback" / "dump" / "nodes.csv")
    with pytest.raises(ValueError, match="survey run"):
        reg.check_sources(report, t / "epoch_1" / "run", s / "readback" / "dump" / "nodes.csv")
    with pytest.raises(ValueError, match="nodes"):
        reg.check_sources(report, s / "epoch_1" / "run", t / "readback" / "dump" / "nodes.csv")


# --- "relocalizable at its start" (lead 2026-10-31, start relocalization plan 7.1 (a)) ---
def test_a_new_row_is_not_marked_relocalizable_at_its_start():
    e = reg.entry(_manifest(), _report(), folder="f", start=[0, 0, 0], name="line_m")
    assert e["start_relocalizable"] is False and e["start_reloc_evidence"] is None
    assert reg.start_relocalizable(e) is False
    # a new survey of the same scene and model replaces the row, its mark with it (the evidence was the old map's)
    book = reg.mark(reg.put({"bundles": []}, e), "line_m", True, "ws1 job 1, 2026-10-31: competitor, truth first")
    book = reg.put(book, reg.entry(_manifest(map_id="c" * 64), _report(map_id="c" * 64), folder="g", start=[0, 0, 0],
                                   name="line_m"))
    assert reg.start_relocalizable(reg.lookup(book, "s" * 64, "team_spec_quantized")) is False


def test_marking_a_row_needs_its_evidence_and_its_name():
    book = reg.put({"bundles": []}, reg.entry(_manifest(), _report(), folder="f", start=[0, 0, 0], name="line_m"))
    marked = reg.mark(book, "line_m", True, "ws1 job 13426, 2026-10-31")
    row = reg.lookup(marked, "s" * 64, "team_spec_quantized")
    assert reg.start_relocalizable(row) is True and row["start_reloc_evidence"] == "ws1 job 13426, 2026-10-31"
    assert reg.start_relocalizable(reg.lookup(book, "s" * 64, "team_spec_quantized")) is False  # a copy, not in place
    for evidence in (None, "", "  "):
        with pytest.raises(ValueError, match="evidence"):
            reg.mark(book, "line_m", True, evidence)
    unmarked = reg.mark(marked, "line_m", False, "ws1 job 13426: uninformative x3")
    assert reg.start_relocalizable(reg.lookup(unmarked, "s" * 64, "team_spec_quantized")) is False
    with pytest.raises(ValueError, match="no registry row"):
        reg.mark(book, "random_m", True, "x")


def test_a_mark_that_is_not_a_yes_or_no_does_not_read():
    row = reg.entry(_manifest(), _report(), folder="f", start=[0, 0, 0], name="line_m")
    for bad in ("yes", 1, None):
        with pytest.raises(ValueError, match="start_relocalizable"):
            reg.start_relocalizable({**row, "start_relocalizable": bad})
    with pytest.raises(ValueError, match="evidence"):  # a yes without its evidence (edited by hand)
        reg.start_relocalizable({**row, "start_relocalizable": True})
    legacy = {k: v for k, v in row.items() if k not in ("start_relocalizable", "start_reloc_evidence")}
    assert reg.start_relocalizable(legacy) is False  # a row written before the mark: no


def test_the_lookup_command_says_whether_the_start_relocalizes(tmp_path, capsys):
    scene = tmp_path / "line_m_session.json"
    scene.write_text("{}")
    row = reg.entry(_manifest(scene_hash=reg.sha256(scene)), _report(), folder="fa", start=[1.0, -2.0, 2.5],
                    name="line_m")
    book = tmp_path / "bundles.yaml"
    reg.write(book, reg.put({"bundles": []}, row))
    look = ["lookup", str(book), "--scene-file", str(scene), "--encoder-model", "team_spec_quantized"]
    assert reg.main(look) == 0 and capsys.readouterr().out.split()[3] == "no"
    assert reg.main(["mark", str(book), "--name", "line_m", "--relocalizable", "yes", "--evidence", "job 1"]) == 0
    capsys.readouterr()
    assert reg.main(look) == 0 and capsys.readouterr().out.split() == ["fa", "a" * 64, "1.0,-2.0,2.5", "yes"]


def test_every_registered_row_says_whether_it_relocalizes_at_its_start_and_why():
    from pathlib import Path

    book = reg.read(Path(reg.__file__).resolve().parents[1] / "config" / "slam_map_bundles.yaml")
    marks = {}
    for row in book["bundles"]:
        assert isinstance(row.get("start_relocalizable"), bool) and str(row.get("start_reloc_evidence") or "").strip()
        marks[row["name"]] = reg.start_relocalizable(row)
    # the Isaac smoke of 2026-10-31 (ws1 13426): random_m's start found with its twins (an operator's pick), line_m's
    # start too bare to search (uninformative x3)
    assert marks == {"line_m": False, "random_m": True}
