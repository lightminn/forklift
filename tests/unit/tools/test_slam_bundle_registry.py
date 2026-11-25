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
        if reg.is_site(row):  # a real site row: no until its bench says otherwise (design 6.1 item 5)
            assert isinstance(row.get("start_relocalizable"), bool)
            continue
        assert isinstance(row.get("start_relocalizable"), bool) and str(row.get("start_reloc_evidence") or "").strip()
        marks[row["name"]] = reg.start_relocalizable(row)
    # the Isaac smoke of 2026-10-31 (ws1 13426): random_m's start found with its twins (an operator's pick), line_m's
    # start too bare to search (uninformative x3)
    assert marks == {"line_m": False, "random_m": True}


# --- real-site rows (design 2026-11-25-real-sources-pr7 6.1 item 5, PR 7c-2a) ---

H = "f" * 64


def _site_report(**kw):
    base = {"kind": "site_survey/1", "ok": True, "site": "hall", "site_record_sha256": "1" * 64, "map_id": "a" * 64,
            "encoder_model": "team_spec_quantized", "start_pose": [0.33, 0.0, 0.0],
            "config_sha256": {"lidar": "2" * 64, "chassis": "3" * 64},
            "raw": [{"path": "observe_20261201.jsonl", "sha256": "4" * 64}],
            "site_check": {"points": 5, "position_max_m": 0.05, "yaw_max_rad": 0.02,
                           "limits": {"position_m": 0.10, "yaw_rad": 0.05}}}
    return {**base, **kw}


def _site(report=None, manifest=None, **kw):
    args = dict(name="hall_tsq", folder="hall_tsq_aaaaaaaaaaaa", report_name="site_report.json", report_sha256=H,
                site_record_sha256="1" * 64, s10a_map_id="a" * 64, s10a_start_rear=(0.0, 0.0, 0.0), rear_offset_m=0.34,
                tol_m=0.10, tol_rad=math.radians(5.0))
    return reg.site_entry(manifest or {"map_id": "a" * 64}, report or _site_report(), **{**args, **kw})


def test_a_site_row_binds_its_record_report_bundle_and_start():
    row = _site()
    assert reg.is_site(row) and "scene_sha256" not in row
    assert row["site_record_sha256"] == "1" * 64 and row["survey_report"] == {"path": "site_report.json", "sha256": H}
    assert row["map_id"] == "a" * 64 and row["start_pose"] == [0.33, 0.0, 0.0]
    assert row["v2"] == {"position_max_m": 0.05, "yaw_max_rad": 0.02} and row["start_relocalizable"] is False
    assert row["site_check_limits"] == {"position_m": 0.10, "yaw_rad": 0.05}


@pytest.mark.parametrize("change, why", [
    (dict(site_record_sha256="9" * 64), "not the report's"),            # A's report, another site record
    (dict(s10a_map_id="b" * 64), "S10-A settings name map"),            # the S10-A file of another map
    (dict(manifest={"map_id": "b" * 64}), "not the bundle's"),          # A's report, B's bundle
    (dict(s10a_start_rear=(0.12, 0.0, 0.0)), "from the S10-A"),         # start_rear disagrees (0.13 m)
    (dict(s10a_start_rear=(0.0, 0.0, 0.1)), "from the S10-A"),          # 5.7 deg
    # Codex 7c-2a round 1: 9.9 cm / 4.9 deg at base_link is 12.8 cm at the rear axle the loop compares
    (dict(report=_site_report(start_pose=[0.34, -0.099, math.radians(4.9)])), "at the rear axle"),
    (dict(report=_site_report(encoder_model=7)), "encoder model"),
    (dict(report=_site_report(encoder_model="  ")), "encoder model"),
    (dict(report=_site_report(ok=False)), "not accepted"),
    (dict(report=_site_report(kind="survey")), "site_survey/1"),
    (dict(report=_site_report(site_check={"points": 4, "position_max_m": 0.05, "yaw_max_rad": 0.02,
                                          "limits": {"position_m": 0.1, "yaw_rad": 0.05}})), "marks"),
    (dict(report=_site_report(site_check={"points": 5, "position_max_m": 0.12, "yaw_max_rad": 0.02,
                                          "limits": {"position_m": 0.1, "yaw_rad": 0.05}})), "over its limits"),
    (dict(report=_site_report(site_check={"points": 5, "position_max_m": 0.05, "yaw_max_rad": 0.02})), "limits are not"),
    (dict(report=_site_report(raw=[])), "survey records"),
    (dict(report=_site_report(config_sha256={"lidar": "2" * 64})), "settings hashes"),
    (dict(report=_site_report(start_pose=[0.3, float("nan"), 0.0])), "three finite"),
    (dict(manifest={"map_id": "a" * 64, "run": {"arguments": {"encoder_model": "reported"}}}), "encoder model"),
])
def test_a_site_row_is_refused_when_its_parts_disagree(change, why):
    with pytest.raises(ValueError, match=why):
        _site(**change)


def test_a_mixed_registry_keeps_both_kinds_apart():
    scene = reg.entry(_manifest(), _report(), folder="fa", start=[0, 0, 0], name="line_m")
    site = _site()
    book = reg.put(reg.put({"bundles": []}, scene), site)
    assert reg.lookup(book, "s" * 64, "team_spec_quantized")["folder"] == "fa"
    assert reg.lookup_site(book, "1" * 64, "team_spec_quantized")["folder"] == "hall_tsq_aaaaaaaaaaaa"
    assert reg.lookup_site(book, "s" * 64, "team_spec_quantized") is None  # a scene hash never finds a site row
    assert reg.lookup(book, "1" * 64, "team_spec_quantized") is None
    assert reg.lookup_site(book, "1" * 64, "reported") is None
    # a new survey of the same site record and model replaces only the site row
    book = reg.put(book, _site(folder="hall_tsq_bbbbbbbbbbbb"))
    assert len(book["bundles"]) == 2
    assert reg.lookup_site(book, "1" * 64, "team_spec_quantized")["folder"] == "hall_tsq_bbbbbbbbbbbb"
    # marking works by name on either kind
    book = reg.mark(book, "hall_tsq", True, "bench 7, 2026-12-01")
    assert reg.start_relocalizable(reg.lookup_site(book, "1" * 64, "team_spec_quantized"))


def _bundle(root, name, data=b"data"):
    """A real bundle folder (load_bundle reads it) under the bundle root."""
    from forklift_core.localization import slam_map as sm

    folder = root / name
    folder.mkdir(parents=True)
    (folder / "map.posegraph").write_bytes(b"graph")
    (folder / "map.data").write_bytes(data)
    np.savez(folder / "grid.npz", data=np.zeros((10, 10), np.int8), origin=np.asarray((0.0, 0.0, 0.0)),
             resolution_m=0.1)
    hashes = {n: sm.file_sha256(folder / n) for n in sm.BUNDLE_FILES}
    (folder / "manifest.json").write_text(json.dumps({"format": sm.FORMAT, "map_id": sm.map_id_of(hashes),
                                                      "files": hashes}))
    return folder, sm.map_id_of(hashes)


CLEAN_FOOTER = '{"t": "footer", "dropped": 0, "write_errors": 0}\n'
OBS_HEADER = {"t": "header", "schema": "observe_record/1", "tool": "real_observe.py", "device": "/dev/forklift-mcu",
              "synthetic": None}


def _tel_header(cal, **kw):
    return {"t": "header", "schema": "teleop_bridge_record/1", "tool": "teleop_bridge.py", "device": "/dev/forklift-mcu",
            "synthetic": None, "config": {"vehicle": {"calibration_id": cal}}, **kw}


def _rewrite(p, name, text):
    """Rewrite a survey file and the report's hash for it."""
    (p["survey"] / name).write_text(text)
    raw = [{**r, "sha256": reg.sha256(p["survey"] / r["path"])} for r in p["body"]["raw"]]
    p["body"] = {**p["body"], "raw": raw}
    p["report"].write_text(json.dumps(p["body"]))


def _site_setup(tmp_path):
    """A bundle root with one bundle and a survey folder (report, raw records, .ok), the S10-A file naming the
    bundle's map, the site record; returns the add-site argv and the parts."""
    import yaml

    from forklift_core.live.real_autonomy import load_s10a
    from forklift_core.live.real_run import chassis_reference
    from forklift_core.localization.start_reloc import base_from_rear
    from forklift_core.plant.real_vehicle import chassis_model_path, load_real_vehicle

    repo = reg.Path(reg.__file__).resolve().parents[1]
    vehicle, lidar = repo / "config/real/vehicle.yaml", repo / "config/real/lidar.yaml"
    urdf = chassis_model_path(vehicle).parent / "forklift.urdf"
    root = tmp_path / "bundles"
    bundle, map_id = _bundle(root, "hall_tsq_aaaaaaaaaaaa")
    s10a = yaml.safe_load((repo / "config/real/s10a.yaml").read_text())
    s10a["map_id"] = map_id
    s10a_path = tmp_path / "s10a.yaml"
    s10a_path.write_text(yaml.safe_dump(s10a))
    _tip, rear_offset, _c = chassis_reference(urdf)
    start = base_from_rear(tuple(float(v) for v in load_s10a(s10a_path).start_rear), rear_offset)
    record = tmp_path / "site_record.md"
    record.write_text("hall, 2026-12-01, marks taped\n")
    survey = root / "survey_hall"
    survey.mkdir()
    cal = load_real_vehicle(vehicle).calibration_id
    (survey / "observe.jsonl").write_text(json.dumps(OBS_HEADER) + '\n{"t": "end", "complete": true}\n')
    (survey / "observe.jsonl.ok").write_text(reg.sha256(survey / "observe.jsonl") + "\n")
    (survey / "teleop.jsonl").write_text(json.dumps(_tel_header(cal)) + '\n{"t": "state"}\n' + CLEAN_FOOTER)
    raw = [{"kind": k, "path": n, "sha256": reg.sha256(survey / n)}
           for k, n in (("observe", "observe.jsonl"), ("ok", "observe.jsonl.ok"), ("teleop", "teleop.jsonl"))]
    report = survey / "site_report.json"
    body = _site_report(site_record_sha256=reg.sha256(record), start_pose=list(start), map_id=map_id, raw=raw,
                        config_sha256={"lidar": reg.sha256(lidar), "chassis": reg.sha256(urdf)},
                        vehicle_calibration_id=load_real_vehicle(vehicle).calibration_id)
    report.write_text(json.dumps(body))
    book = tmp_path / "bundles.yaml"
    argv = ["add-site", str(book), "--name", "hall_tsq", "--bundle", str(bundle), "--report", str(report),
            "--site-record", str(record), "--s10a", str(s10a_path), "--vehicle", str(vehicle), "--lidar", str(lidar)]
    return argv, {"book": book, "record": record, "report": report, "body": body, "survey": survey, "root": root,
                  "bundle": bundle, "map_id": map_id, "start": start, "cal": cal}


def test_the_add_site_command_hashes_the_record_itself(tmp_path, capsys):
    """The registration reads the site record and hashes it; the S10-A start is compared at the rear axle; the row
    keeps the report's path relative to the bundle root."""
    argv, p = _site_setup(tmp_path)
    assert reg.main(argv) == 0
    capsys.readouterr()
    row = reg.read(p["book"])["bundles"][0]
    assert row["survey_report"] == {"path": "survey_hall/site_report.json", "sha256": reg.sha256(p["report"])}
    look = ["lookup-site", str(p["book"]), "--site-record", str(p["record"]), "--encoder-model", "team_spec_quantized"]
    assert reg.main(look) == 0
    folder, map_id, pose, yes = capsys.readouterr().out.split()
    assert folder == "hall_tsq_aaaaaaaaaaaa" and map_id == p["map_id"] and yes == "no"
    assert [float(v) for v in pose.split(",")] == pytest.approx(list(p["start"]))
    # an edited site record no longer matches the report
    p["record"].write_text("hall, 2026-12-01, marks taped (edited)\n")
    with pytest.raises(ValueError, match="not the report's"):
        reg.main(argv)
    assert reg.main(look) == 3


def test_a_left_over_manifest_does_not_carry_another_bundles_files_in(tmp_path):
    """Codex 7c-2a round 1: A's manifest with B's map file in the folder is refused (load_bundle)."""
    from forklift_core.localization.slam_map import MapBundleError

    argv, p = _site_setup(tmp_path)
    (p["bundle"] / "map.data").write_bytes(b"B's data")
    with pytest.raises(MapBundleError, match="differ from the manifest"):
        reg.main(argv)


@pytest.mark.parametrize("change, why", [
    (lambda p: (p["survey"] / "observe.jsonl").write_text("edited\n"), "differs from the report's hash"),
    (lambda p: (p["survey"] / "observe.jsonl.ok").unlink(), "is not there"),
    (lambda p: p["report"].write_text(json.dumps({**p["body"], "raw": [r for r in p["body"]["raw"]
                                                                       if r["kind"] != "ok"]})), "no .ok listed"),
    (lambda p: p["report"].write_text(json.dumps({**p["body"], "raw": [r for r in p["body"]["raw"]
                                                                       if r["kind"] != "teleop"]})), "teleop"),
    (lambda p: p["report"].write_text(json.dumps({**p["body"], "raw": [{**r, "kind": "video"} if r["kind"] == "teleop"
                                                                       else r for r in p["body"]["raw"]]})), "by its content"),
    (lambda p: p["report"].write_text(json.dumps({**p["body"], "vehicle_calibration_id": "0" * 16})), "calibration"),
    (lambda p: p["report"].write_text(json.dumps({**p["body"], "raw": [{"path": "../../s10a.yaml",
                                                                       "sha256": "0" * 64}]})), "not under"),
    (lambda p: p["report"].write_text(json.dumps({**p["body"], "config_sha256": {**p["body"]["config_sha256"],
                                                                                "chassis": "0" * 64}})), "chassis"),
    (lambda p: p["report"].write_text(json.dumps({**p["body"], "config_sha256": {**p["body"]["config_sha256"],
                                                                                "lidar": "0" * 64}})), "lidar"),
])
def test_the_report_must_bind_one_survey_on_disk(tmp_path, change, why):
    argv, p = _site_setup(tmp_path)
    change(p)
    with pytest.raises(ValueError, match=why):
        reg.main(argv)


@pytest.mark.parametrize("change, why", [
    # Codex 7c-2a round 3: the marker tagged as the teleop record
    (lambda p: p["report"].write_text(json.dumps({**p["body"], "raw": [
        {**r, "kind": "teleop"} if r["kind"] == "ok" else r for r in p["body"]["raw"] if r["kind"] != "teleop"]})),
     "by its content"),
    # the observation record listed again as the teleop record
    (lambda p: p["report"].write_text(json.dumps({**p["body"], "raw": p["body"]["raw"] + [
        {"kind": "teleop", "path": "./observe.jsonl", "sha256": reg.sha256(p["survey"] / "observe.jsonl")}]})),
     "listed twice"),
    (lambda p: _rewrite(p, "teleop.jsonl", json.dumps(_tel_header("0" * 16)) + "\n"), "another vehicle calibration"),
    (lambda p: _rewrite(p, "teleop.jsonl", json.dumps(_tel_header(p["cal"])) + '\n{"t": "refused", "why": "x"}\n'),
     "was refused"),
    (lambda p: _rewrite(p, "teleop.jsonl", json.dumps(_tel_header(p["cal"], device="/dev/pts/7")) + "\n"),
     "rehearsal"),
    # Codex 7c-2a round 4: a teleop record is complete only with a clean footer last
    (lambda p: _rewrite(p, "teleop.jsonl", json.dumps(_tel_header(p["cal"])) + "\n"), "clean footer"),
    (lambda p: _rewrite(p, "teleop.jsonl", json.dumps(_tel_header(p["cal"])) + '\n{"t": "state", "x": 1'), "does not parse"),
    (lambda p: _rewrite(p, "teleop.jsonl", json.dumps(_tel_header(p["cal"])) + "\nnull\n" + CLEAN_FOOTER),
     "not an object"),
    (lambda p: _rewrite(p, "teleop.jsonl", json.dumps(_tel_header(p["cal"]))
                        + '\n{"t": "footer", "dropped": 2, "write_errors": 0}\n'), "clean footer"),
    (lambda p: _rewrite(p, "teleop.jsonl", json.dumps(_tel_header(p["cal"]))
                        + '\n{"t": "footer", "dropped": 0, "write_errors": 1}\n'), "clean footer"),
    (lambda p: _rewrite(p, "teleop.jsonl", json.dumps(_tel_header(p["cal"], synthetic={"bench": {}})) + "\n"),
     "rehearsal"),
])
def test_each_record_is_what_its_file_says_and_real(tmp_path, change, why):
    argv, p = _site_setup(tmp_path)
    change(p)
    with pytest.raises(ValueError, match=why):
        reg.main(argv)


def test_a_marker_must_be_a_listed_observation_records(tmp_path):
    argv, p = _site_setup(tmp_path)
    (p["survey"] / "other.jsonl.ok").write_text("0" * 64 + "\n")
    p["body"]["raw"].append({"kind": "ok", "path": "other.jsonl.ok", "sha256": reg.sha256(p["survey"] / "other.jsonl.ok")})
    p["report"].write_text(json.dumps(p["body"]))
    with pytest.raises(ValueError, match="not a listed observation"):
        reg.main(argv)


def test_an_observation_record_is_complete_only_by_its_own_ok(tmp_path):
    """Codex 7c-2a round 2: an .ok of another run, or a record without its complete end, does not count."""
    argv, p = _site_setup(tmp_path)
    sv, body = p["survey"], p["body"]
    (sv / "observe.jsonl").write_text(json.dumps(OBS_HEADER) + "\n")  # no complete end, its .ok made for it
    (sv / "observe.jsonl.ok").write_text(reg.sha256(sv / "observe.jsonl") + "\n")
    raw = [{**r, "sha256": reg.sha256(sv / r["path"])} for r in body["raw"]]
    p["report"].write_text(json.dumps({**body, "raw": raw}))
    with pytest.raises(ValueError, match="not complete by its .ok"):
        reg.main(argv)
    (sv / "observe.jsonl").write_text(json.dumps(OBS_HEADER) + '\n{"t": "end", "complete": true}\n')
    (sv / "observe.jsonl.ok").write_text("0" * 64 + "\n")  # another run's marker
    raw = [{**r, "sha256": reg.sha256(sv / r["path"])} for r in body["raw"]]
    p["report"].write_text(json.dumps({**body, "raw": raw}))
    with pytest.raises(ValueError, match="not complete by its .ok"):
        reg.main(argv)


def test_a_report_given_through_a_link_is_read_and_kept_at_its_target(tmp_path):
    """Codex 7c-2a round 2 P3: one resolved path for the sources and the row."""
    argv, p = _site_setup(tmp_path)
    link = p["root"] / "report_link.json"
    link.symlink_to(p["report"])
    argv[argv.index("--report") + 1] = str(link)
    assert reg.main(argv) == 0
    assert reg.read(p["book"])["bundles"][0]["survey_report"]["path"] == "survey_hall/site_report.json"


def test_a_report_outside_the_bundle_root_is_refused(tmp_path):
    argv, p = _site_setup(tmp_path)
    outside = tmp_path / "site_report.json"
    outside.write_text(p["report"].read_text())
    for n in ("observe.jsonl", "observe.jsonl.ok", "teleop.jsonl"):
        (tmp_path / n).write_text((p["survey"] / n).read_text())
    argv[argv.index("--report") + 1] = str(outside)
    with pytest.raises(ValueError, match="not under the bundle root"):
        reg.main(argv)


def test_a_scene_row_without_its_scene_hash_stays_an_error():
    """Codex 7c-2a round 1: the launcher must not read a broken scene row as no row (exit 3 -> online mapping)."""
    book = {"bundles": [{"name": "line_m", "scen_sha256": "s" * 64, "encoder_model": "team_spec_quantized"}]}
    with pytest.raises(KeyError):
        reg.lookup(book, "s" * 64, "team_spec_quantized")
