"""The operating unlock U2b (docs/plans/2026-11-26-real-operating-assembly.md 2.8, C6): the operating REAL preparation
finds its map bundle through the registry by the site record and the one odometry model, reads the row's survey report
under the bundle root by its hash, and refuses -- never bypassably -- a report that does not bind the files this run
reads; the LiDAR calibration the layer and the scan source take is parsed from the bytes hashed at the start."""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

import numpy as np
import pytest
import yaml

from forklift_core.live import real_run as rr
from forklift_core.live import real_site as rsite
from forklift_core.live.real_assembly import AssemblyRefused
from forklift_core.localization import slam_map as sm
from forklift_core.live.real_site import REAL_ENCODER_MODEL
from forklift_core.plant.real_vehicle import chassis_model_path, load_real_vehicle
from test_real_run import InlineJobs

REPO = rr.REPO
MS = 1_000_000
START = 10_000 * MS


def sha(path) -> str:
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _bundle(root):
    folder = root / "site_bundle"
    folder.mkdir()
    data = np.zeros((50, 70), np.int8)
    (folder / "map.posegraph").write_bytes(b"graph")
    (folder / "map.data").write_bytes(b"data")
    np.savez(folder / "grid.npz", data=data, origin=np.asarray((-1.0, -2.5, 0.0)), resolution_m=0.1)
    hashes = {name: sm.file_sha256(folder / name) for name in sm.BUNDLE_FILES}
    map_id = sm.map_id_of(hashes)
    (folder / "manifest.json").write_text(json.dumps({"format": sm.FORMAT, "map_id": map_id, "files": hashes}))
    return folder, map_id


def make_site(tmp_path, *, report_over=None, row_over=None, rows=None):
    """A registered site: (RunPaths for the operating run, the pieces)."""
    root = tmp_path / "bundles"
    root.mkdir(parents=True)
    folder, map_id = _bundle(root)
    d = yaml.safe_load((REPO / "config/real/s10a.yaml").read_text())
    d.update(map_id=map_id, check_pallet_pose=[3.0, 0.0, 0.0])
    d["pallet"].update(depth_m=0.6, width_m=0.4)
    s10a = tmp_path / "s10a.yaml"
    s10a.write_text(yaml.safe_dump(d))
    record = tmp_path / "site_record.yaml"
    record.write_text("site: test hall\n")
    vehicle = REPO / "config/real/vehicle.yaml"
    urdf = chassis_model_path(vehicle).parent / "forklift.urdf"
    rear_offset = rr.chassis_reference(urdf)[1]
    x, y, yaw = (float(v) for v in d["start_rear"])
    start_base = [x + rear_offset * math.cos(yaw), y + rear_offset * math.sin(yaw), yaw]
    report = {"kind": "site_survey/1", "ok": True, "site": "test hall", "site_record_sha256": sha(record),
              "map_id": map_id, "encoder_model": REAL_ENCODER_MODEL, "start_pose": start_base,
              "config_sha256": {"lidar": sha(REPO / "config/real/lidar.yaml"), "chassis": sha(urdf)},
              "vehicle_calibration_id": load_real_vehicle(vehicle).calibration_id, "raw": [],
              "site_check": {"points": 5}}
    report.update(report_over or {})
    rpath = folder / "site_survey_report.json"
    rpath.write_text(json.dumps(report))
    row = {"name": "hall", "site": "test hall", "site_record_sha256": sha(record),
           "survey_report": {"path": "site_bundle/site_survey_report.json", "sha256": sha(rpath)},
           "encoder_model": REAL_ENCODER_MODEL, "folder": folder.name, "map_id": map_id, "start_pose": start_base,
           "start_relocalizable": False, "start_reloc_evidence": None}
    row.update(row_over or {})
    registry = tmp_path / "registry.yaml"
    registry.write_text(yaml.safe_dump({"bundles": rows if rows is not None else [row]}))
    paths = rr.RunPaths(s10a=s10a, hil=False, site_record=record, registry=registry, bundle_root=root)
    return paths, {"row": row, "report": report, "folder": folder, "map_id": map_id, "record": record, "root": root}


def prepare(paths, **kw):
    return rr.prepare_run(paths, clock=lambda: START, jobs_factory=InlineJobs, **kw)


def refused(paths, **kw) -> list:
    with pytest.raises(AssemblyRefused) as err:
        prepare(paths, **kw)
    return err.value.reasons


def test_a_registered_site_gives_the_bundle_the_report_and_the_hashes(tmp_path):
    paths, p = make_site(tmp_path)
    prep = prepare(paths)
    assert prep.map_id == p["map_id"] and prep.site is not None and prep.site.bundle == p["folder"].resolve()
    assert prep.hashes["site_record"] == sha(p["record"]) and prep.hashes["registry"] == sha(paths.registry)
    assert prep.hashes["site_report"] == p["row"]["survey_report"]["sha256"]
    assert prep.lidar is not None  # the one LiDAR calibration of the run


def test_a_bundle_given_by_hand_is_refused(tmp_path):
    paths, p = make_site(tmp_path)
    paths.map_bundle = p["folder"]
    assert refused(paths) == ["precondition:site_bundle:map_bundle_by_hand"]


@pytest.mark.parametrize("field, why", [("site_record", "site_record_unset"), ("bundle_root", "bundle_root_unset")])
def test_an_unset_site_record_or_bundle_root_is_refused(tmp_path, field, why):
    paths, _ = make_site(tmp_path)
    setattr(paths, field, None)
    assert refused(paths) == [f"precondition:site_bundle:{why}"]


def test_no_row_for_this_site_record_or_model(tmp_path):
    paths, p = make_site(tmp_path)
    p["record"].write_text("site: another hall\n")
    assert refused(paths) == ["precondition:site_bundle:no_row"]
    paths, _ = make_site(tmp_path / "b", row_over={"encoder_model": "team_spec_quantized"})
    assert refused(paths) == ["precondition:site_bundle:no_row"]  # a survey made with another odometry model


def test_two_rows_and_a_malformed_registry(tmp_path):
    paths, p = make_site(tmp_path)
    paths.registry.write_text(yaml.safe_dump({"bundles": [p["row"], dict(p["row"], name="again")]}))
    assert refused(paths) == ["precondition:site_bundle:two_rows"]
    paths.registry.write_text("bundles: 5\n")
    assert refused(paths) == ["precondition:site_bundle:registry_unreadable:shape"]
    paths.registry.write_text("bundles: [\n")
    assert refused(paths)[0].startswith("precondition:site_bundle:registry_unreadable:")


@pytest.mark.parametrize("over, why", [
    ({"folder": "../elsewhere"}, "row_folder"), ({"folder": "a/b"}, "row_folder"), ({"folder": ".."}, "row_folder"),
    ({"survey_report": {"path": "site_bundle/site_survey_report.json", "sha256": "0" * 64}}, "report_sha"),
    ({"survey_report": {"path": "../outside.json", "sha256": "0" * 64}}, "row_report_outside_root"),
    ({"survey_report": None}, "row_report"),
])
def test_the_row_points_only_inside_the_bundle_root_at_the_report_it_hashed(tmp_path, over, why):
    paths, _ = make_site(tmp_path, row_over=over)
    assert refused(paths) == [f"precondition:site_bundle:{why}"]


@pytest.mark.parametrize("over, why", [
    ({"config_sha256": {"lidar": "0" * 64, "chassis": None}}, ["lidar", "chassis"]),
    ({"vehicle_calibration_id": "0" * 16}, ["vehicle_calibration"]),
    ({"encoder_model": "team_spec_quantized"}, ["encoder_model"]),
    ({"site_record_sha256": "0" * 64}, ["site_record"]),
    ({"map_id": "f" * 64}, ["map_id"]),
    ({"start_pose": "here"}, ["start_pose"]),
])
def test_a_report_that_does_not_bind_this_run_is_refused(tmp_path, over, why):
    """Plan 5 (C6): another lidar hash, an encoder model, a start 0.11 m off -- each its own reason."""
    paths, _ = make_site(tmp_path, report_over=over)
    assert refused(paths) == [f"precondition:site_bundle:{w}" for w in why]


def _moved_start(tmp_path, dx_m, dyaw_deg):
    paths, p = make_site(tmp_path)
    x, y, yaw = p["report"]["start_pose"]
    p["report"]["start_pose"] = [x + dx_m, y, yaw + math.radians(dyaw_deg)]
    rpath = p["folder"] / "site_survey_report.json"
    rpath.write_text(json.dumps(p["report"]))
    p["row"]["survey_report"]["sha256"] = sha(rpath)
    p["row"]["start_pose"] = list(p["report"]["start_pose"])  # registered from the report
    paths.registry.write_text(yaml.safe_dump({"bundles": [p["row"]]}))
    return paths


@pytest.mark.parametrize("dx_m, dyaw_deg", [(0.11, 0.0), (0.0, 5.5)])
def test_a_surveyed_start_off_the_taped_one_is_refused(tmp_path, dx_m, dyaw_deg):
    """Plan 5 (C6): a start 0.11 m (or 5.5 deg) from S10-A's taped start at the rear axle."""
    assert refused(_moved_start(tmp_path, dx_m, dyaw_deg)) == ["precondition:site_bundle:start"]


def test_a_start_within_the_tolerance_is_taken(tmp_path):
    assert prepare(_moved_start(tmp_path, 0.09, 4.0)).site is not None


def test_a_report_not_accepted_is_refused(tmp_path):
    paths, _ = make_site(tmp_path, report_over={"ok": False})
    assert refused(paths) == ["precondition:site_bundle:report_not_accepted"]


def test_the_operating_lidar_file_is_parsed_from_the_bytes_hashed_at_the_start(tmp_path, monkeypatch):
    """Codex U2a round 2 P2-2: other bytes read for parsing than those hashed at the start refuse the run."""
    paths, _ = make_site(tmp_path)
    real_sha = rr._sha  # noqa: SLF001
    calls = {"lidar": 0}

    def fake(path):
        h = real_sha(path)
        if str(path) == str(paths.lidar_config):
            calls["lidar"] += 1
            return "0" * 64 if calls["lidar"] == 1 else h  # A at the start, B parsed, A at the end
        return h

    monkeypatch.setattr(rr, "_sha", fake)
    assert "files_changed_while_preparing:lidar" in refused(paths)


def test_the_hil_and_replay_runs_need_no_site():
    """The HIL run keeps its own map (S10-A's id or a bundle by hand): no registry is read."""
    assert rsite.PREFIX == "precondition:site_bundle"
    prep = rr.prepare_run(rr.RunPaths(s10a=REPO / "config/real/s10a_hil.yaml", registry=REPO / "nonexistent.yaml"),
                          clock=lambda: START, jobs_factory=InlineJobs)
    assert prep.site is None


def test_the_bridge_laser_tool_prints_the_hellos_value():
    import sys

    sys.path.insert(0, str(REPO / "tools"))
    import real_bridge_laser as tool

    from forklift_core.sensors.lidar_cfg import load_lidar_config

    urdf = chassis_model_path(REPO / "config/real/vehicle.yaml").parent / "forklift.urdf"
    want = rr.bridge_laser_xyz_yaw(load_lidar_config(REPO / "config/real/lidar.yaml"), rr.chassis_reference(urdf)[1])
    assert tool.laser_param(REPO / "config/real/lidar.yaml", REPO / "config/real/vehicle.yaml") == list(want)
    assert tool.main(["--lidar", str(REPO / "nonexistent.yaml")]) == 1


# --- Codex U2b round 1 ---------------------------------------------------------------------------------------------------
@pytest.mark.parametrize("row_start", [None, "here", [0.0, 0.0], "moved"])
def test_the_rows_start_must_be_the_reports(tmp_path, row_start):
    """P2-1: the row's start (the hello's and relocalization's) is checked against the report's, not skipped."""
    paths, p = make_site(tmp_path)
    if row_start == "moved":
        row_start = [p["row"]["start_pose"][0] + 0.11, *p["row"]["start_pose"][1:]]
    p["row"]["start_pose"] = row_start
    paths.registry.write_text(yaml.safe_dump({"bundles": [p["row"]]}))
    assert refused(paths) == ["precondition:site_bundle:row_start"]


def test_the_chassis_urdf_is_parsed_from_the_bytes_hashed_at_the_start(tmp_path, monkeypatch):
    """P2-2: other URDF bytes than those hashed at the start refuse the run (A -> B -> A)."""
    paths, _ = make_site(tmp_path)
    urdf = chassis_model_path(paths.vehicle).parent / "forklift.urdf"
    real_sha = rr._sha  # noqa: SLF001
    calls = {"n": 0}

    def fake(path):
        h = real_sha(path)
        if Path(path) == urdf:
            calls["n"] += 1
            return "0" * 64 if calls["n"] == 1 else h
        return h

    monkeypatch.setattr(rr, "_sha", fake)
    assert "files_changed_while_preparing:chassis_urdf" in refused(paths)


def test_the_bundle_grid_is_the_one_its_hash_names(tmp_path, monkeypatch):
    """P2-3: the grid the planner takes is parsed from the bytes whose hash makes the map id (one read per file)."""
    from forklift_core.live import real_site as rs_

    paths, p = make_site(tmp_path)
    reads = []
    real_read = rs_._read  # noqa: SLF001

    def counting(path, what):
        reads.append(Path(path).name)
        return real_read(path, what)

    monkeypatch.setattr(rs_, "_read", counting)
    prep = prepare(paths)
    assert reads.count("grid.npz") == 1 and prep.map_id == p["map_id"]


def test_a_bundle_whose_files_differ_from_the_manifest_or_another_map(tmp_path):
    paths, p = make_site(tmp_path)
    (p["folder"] / "map.data").write_bytes(b"other")
    assert refused(paths) == ["precondition:site_bundle:bundle_files"]
    paths, p = make_site(tmp_path / "b")
    d = yaml.safe_load(paths.s10a.read_text())
    d["map_id"] = "e" * 64
    paths.s10a.write_text(yaml.safe_dump(d))
    assert refused(paths) == ["precondition:site_bundle:map_id"]  # the plan's reason, not the loader's


def test_a_folder_linked_out_of_the_root_is_refused(tmp_path):
    paths, p = make_site(tmp_path)
    outside = tmp_path / "outside"
    p["folder"].rename(outside)
    (p["root"] / p["folder"].name).symlink_to(outside, target_is_directory=True)
    assert refused(paths) == ["precondition:site_bundle:row_folder_outside_root"]


def test_a_missing_registry_tool_refuses_and_leaves_no_half_module(monkeypatch, tmp_path):
    import sys

    from forklift_core.live import real_site as rs_

    monkeypatch.setattr(rs_, "REPO", tmp_path)
    monkeypatch.delitem(sys.modules, "slam_bundle_registry", raising=False)
    for _ in range(2):
        with pytest.raises(rs_.SiteRefused) as err:
            rs_._registry_module()  # noqa: SLF001
        assert err.value.reason.startswith("precondition:site_bundle:registry_tool:")
        assert "slam_bundle_registry" not in sys.modules


# --- Codex U2b round 2 P3 ------------------------------------------------------------------------------------------------
def test_a_missing_registry_tool_is_its_own_reason_in_the_preparation(tmp_path, monkeypatch):
    import sys

    from forklift_core.live import real_site as rs_

    paths, _ = make_site(tmp_path)
    monkeypatch.setattr(rs_, "REPO", tmp_path / "nowhere")
    monkeypatch.delitem(sys.modules, "slam_bundle_registry", raising=False)
    [why] = refused(paths)
    assert why.startswith("precondition:site_bundle:registry_tool:")


def test_a_rotated_grid_origin_is_a_structured_refusal(tmp_path):
    paths, p = make_site(tmp_path)
    folder = p["folder"]
    np.savez(folder / "grid.npz", data=np.zeros((50, 70), np.int8), origin=np.asarray((-1.0, -2.5, 0.1)),
             resolution_m=0.1)
    hashes = {name: sm.file_sha256(folder / name) for name in sm.BUNDLE_FILES}
    map_id = sm.map_id_of(hashes)
    (folder / "manifest.json").write_text(json.dumps({"format": sm.FORMAT, "map_id": map_id, "files": hashes}))
    d = yaml.safe_load(paths.s10a.read_text())
    d["map_id"] = map_id
    paths.s10a.write_text(yaml.safe_dump(d))
    for key in ("map_id",):
        p["report"][key] = map_id
        p["row"][key] = map_id
    rpath = folder / "site_survey_report.json"
    rpath.write_text(json.dumps(p["report"]))
    p["row"]["survey_report"]["sha256"] = sha(rpath)
    paths.registry.write_text(yaml.safe_dump({"bundles": [p["row"]]}))
    [why] = refused(paths)
    assert why.startswith("precondition:site_bundle:bundle_grid:")


@pytest.mark.parametrize("name", ["layer", "budget"])
def test_the_operating_layer_and_budget_are_parsed_from_the_bytes_hashed_at_the_start(tmp_path, monkeypatch, name):
    """Codex U3e-2 P2-1: a layer or budget file replaced and put back while preparing (A hashed at the start, B parsed,
    A at the end) refuses the run instead of leaving B's stop model in the prep."""
    paths, _ = make_site(tmp_path)
    real_sha = rr._sha  # noqa: SLF001
    target = str(getattr(paths, name))
    calls = {"n": 0}

    def fake(path):
        h = real_sha(path)
        if str(path) == target:
            calls["n"] += 1
            return "0" * 64 if calls["n"] == 1 else h
        return h

    monkeypatch.setattr(rr, "_sha", fake)
    assert f"files_changed_while_preparing:{name}" in refused(paths)


def test_a_budget_that_could_never_end_an_overdue_is_refused(tmp_path):
    """Codex U3e-2 P2-3: a .nan (or non-positive) overdue_end would hold layer_overdue for good."""
    import yaml

    import dataclasses

    paths, _ = make_site(tmp_path)
    b = yaml.safe_load(Path(paths.budget).read_text())
    b["overdue_end"]["seconds"] = float("nan")
    budget = tmp_path / "budget_nan.yaml"  # a copy: the repository's own file is never written
    budget.write_text(yaml.safe_dump(b))
    assert "precondition:C13:budget:values_invalid" in refused(dataclasses.replace(paths, budget=str(budget)))
