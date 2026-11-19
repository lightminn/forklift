"""Design 2026-11-19-real-assembly 1, 9: tools/real_autonomy.py --dry-assemble -- 2 for a structural refusal (the
repository's template, a missing or another map bundle, unreadable settings, a reference plan the map blocks), 0 for a
sound assembly whose reference plan succeeds on the surveyed map; the record names the options, the hooks, the caps
and the files' hashes. No device is opened."""

import json
import sys
from pathlib import Path

import numpy as np
import yaml

from forklift_core.localization import slam_map as sm

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "tools"))
import real_autonomy as tool  # noqa: E402


def bundle(tmp_path, wall=False):
    """An empty surveyed field over s10a.yaml's bounds (0.1 m cells), optionally a wall across the approach."""
    folder = tmp_path / ("bundle_wall" if wall else "bundle")
    folder.mkdir()
    data = np.zeros((50, 70), np.int8)  # rows = y from -2.5, columns = x from -1.0
    if wall:
        data[:, 30] = 100  # x = 2.0 m, the whole field's width
    (folder / "map.posegraph").write_bytes(b"graph")
    (folder / "map.data").write_bytes(b"data")
    np.savez(folder / "grid.npz", data=data, origin=np.asarray((-1.0, -2.5, 0.0)), resolution_m=0.1)
    hashes = {name: sm.file_sha256(folder / name) for name in sm.BUNDLE_FILES}
    (folder / "manifest.json").write_text(json.dumps({"format": sm.FORMAT, "map_id": sm.map_id_of(hashes),
                                                      "files": hashes}))
    return folder, sm.map_id_of(hashes)


def filled(tmp_path, map_id):
    d = yaml.safe_load((ROOT / "config/real/s10a.yaml").read_text())
    d.update(map_id=map_id, check_pallet_pose=[3.0, 0.0, 0.0])
    d["pallet"].update(depth_m=0.6, width_m=0.4)
    path = tmp_path / f"s10a_{map_id[:6]}.yaml"
    path.write_text(yaml.safe_dump(d))
    return path


def run(*argv, out):
    code = tool.main(["--dry-assemble", "--out", str(out), *argv])
    return code, json.loads(out.read_text())


def test_the_template_is_refused_with_its_unset_values(tmp_path):
    code, rec = run(out=tmp_path / "a.json")
    assert code == 2 and rec["assembled"] is False
    assert {"map_id_unset", "pallet_depth_m_unset", "check_pallet_pose_unset"} <= set(rec["refused"])


def test_a_filled_configuration_assembles_and_plans_its_reference_on_the_map(tmp_path):
    folder, map_id = bundle(tmp_path)
    code, rec = run("--s10a", str(filled(tmp_path, map_id)), "--map-bundle", str(folder), out=tmp_path / "a.json")
    assert code == 0 and rec["assembled"] is True and rec["check_plan"] == "success", rec
    assert rec["options"]["dt"] == 1 / 50 and rec["options"]["s10_stop_at_lift"] is True
    caps = rec["curvature_caps"]
    assert caps["planner"] == caps["travel"] == caps["tracker"] == caps["preview"]
    assert rec["ready"].startswith("not_ready") and set(rec["hashes"]) == {"s10a", "settings", "vehicle"}


def test_no_bundle_or_another_bundle_is_refused(tmp_path):
    folder, map_id = bundle(tmp_path)
    cfg = filled(tmp_path, map_id)
    code, rec = run("--s10a", str(cfg), out=tmp_path / "a.json")
    assert code == 2 and any(r.startswith("map_id_mismatch") for r in rec["refused"])
    other = filled(tmp_path, "0" * 64)
    code, rec = run("--s10a", str(other), "--map-bundle", str(folder), out=tmp_path / "b.json")
    assert code == 2 and rec["refused"][0].startswith("map:")


def test_a_reference_plan_the_map_blocks_is_refused(tmp_path):
    folder, map_id = bundle(tmp_path, wall=True)
    code, rec = run("--s10a", str(filled(tmp_path, map_id)), "--map-bundle", str(folder), out=tmp_path / "a.json")
    assert code == 2 and rec["assembled"] is True and rec["check_plan"] != "success"


def test_unreadable_settings_are_refused(tmp_path):
    bad = tmp_path / "autonomy.yaml"
    bad.write_text("lift_target_m: [unclosed")
    code, rec = run("--settings", str(bad), out=tmp_path / "a.json")
    assert code == 2 and rec["refused"][0].startswith("settings_unreadable")
    partial = tmp_path / "partial.yaml"
    partial.write_text("lift_target_m: 0.2\n")
    code, rec = run("--settings", str(partial), out=tmp_path / "b.json")
    assert code == 2 and "setting_missing" in rec["refused"][0]


def test_an_unreadable_vehicle_file_is_a_structural_refusal(tmp_path):
    folder, map_id = bundle(tmp_path)
    empty = tmp_path / "vehicle.yaml"
    empty.write_text("")
    code, rec = run("--s10a", str(filled(tmp_path, map_id)), "--map-bundle", str(folder), "--vehicle", str(empty),
                    out=tmp_path / "a.json")
    assert code == 2 and rec["refused"][0].startswith("vehicle_unreadable")


def test_run_refuses_any_device_before_opening_anything(tmp_path, monkeypatch, capsys):
    """4c design 5: ``--run --device X`` exits 2 naming the refusal; nothing is opened (an open() of the device path
    would fail the test); without a device the run is refused as not wired."""
    import builtins

    opened = []
    real_open = builtins.open

    def spy(path, *a, **k):
        opened.append(str(path))
        return real_open(path, *a, **k)

    monkeypatch.setattr(builtins, "open", spy)
    assert tool.main(["--run", "--device", "/dev/ttyACM0"]) == 2
    assert not any("ttyACM0" in p for p in opened)
    record = json.loads(capsys.readouterr().out)
    assert record["refused"] == ["device_refused:4c"] and record["run"] is False
    assert tool.main(["--dry-assemble", "--device", "/dev/ttyACM0"]) == 2  # a device never rides along a dry assembly
    capsys.readouterr()
    assert tool.main(["--run"]) == 2
    assert json.loads(capsys.readouterr().out)["refused"] == ["run_not_wired:4c"]
