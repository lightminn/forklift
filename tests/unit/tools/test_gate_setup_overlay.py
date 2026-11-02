"""The replay gate's setup-input overlay (tools/gate_setup_overlay.py): a record's non-code setup inputs are written
back from its recording snapshot onto a marked copy of the gate snapshot, code is never written."""

import hashlib
import json
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "tools"))
import gate_setup_overlay as G  # noqa: E402


def sha(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def make(tmp_path, files: dict, gate_files: dict, marker=True, extra_entries=()):
    """files: the recording snapshot's {repo-relative path: bytes}; gate_files: the gate copy's. The record reads
    every file in `files` by its relative path, plus one absolute code entry inside the recording snapshot."""
    rec = tmp_path / "snapshots/rec_1"
    gate = tmp_path / "gate"
    gate.mkdir(parents=True)
    for root, content in ((rec, files), (gate, gate_files)):
        for rel, data in content.items():
            (root / rel).parent.mkdir(parents=True, exist_ok=True)
            (root / rel).write_bytes(data)
    (rec / "src/m.py").parent.mkdir(parents=True, exist_ok=True)
    (rec / "src/m.py").write_bytes(b"x = 1\n")
    entries = [{"path": str(rec / "src/m.py"), "repo_relative": "src/m.py", "sha256": sha(b"x = 1\n"), "bytes": 6}]
    entries += [{"path": rel, "repo_relative": rel, "sha256": sha(data), "bytes": len(data)} for rel, data in files.items()]
    entries += list(extra_entries)
    run = tmp_path / "run"
    (run / "boundary").mkdir(parents=True)
    (run / "boundary/setup_snapshot.json").write_text(json.dumps({"files": entries, "names": {}}))
    if marker:
        (gate / G.MARKER).write_text("copy\n")
    return run, rec, gate


def test_a_changed_config_is_written_back_from_the_recording_snapshot_and_reported(tmp_path):
    run, rec, gate = make(tmp_path, {"config/a.yaml": b"mass: 10\n", "config/b.json": b"{}"},
                          {"config/a.yaml": b"mass: 4.93\n", "config/b.json": b"{}"})
    out = G.overlay(run, gate)
    assert (gate / "config/a.yaml").read_bytes() == b"mass: 10\n"
    assert out["recording_snapshot"] == str(rec)
    assert out["written"] == [{"path": "config/a.yaml", "recorded_sha256": sha(b"mass: 10\n"),
                               "gate_sha256": sha(b"mass: 4.93\n"), "source": str(rec / "config/a.yaml")}]
    assert out["unchanged"] == 1  # b.json already matches


def test_a_file_missing_from_the_gate_copy_is_written_too(tmp_path):
    run, rec, gate = make(tmp_path, {"config/new.yaml": b"a: 1\n"}, {})
    out = G.overlay(run, gate)
    assert (gate / "config/new.yaml").read_bytes() == b"a: 1\n"
    assert out["written"][0]["gate_sha256"] is None


@pytest.mark.parametrize("rel", ["sim/isaac/x.py", "src/forklift_core/__pycache__/m.cpython-311.pyc",
                                 "tools/run.sh", "lib/ext.so", "config/noext"])
def test_a_changed_code_file_is_refused_and_nothing_is_written(tmp_path, rel):
    run, rec, gate = make(tmp_path, {"config/a.yaml": b"old\n", rel: b"old code"},
                          {"config/a.yaml": b"new\n", rel: b"new code"})
    with pytest.raises(G.OverlayRefused, match="not a setup data file"):
        G.overlay(run, gate)
    assert (gate / "config/a.yaml").read_bytes() == b"new\n"
    assert (gate / rel).read_bytes() == b"new code"


def test_an_unmarked_directory_is_never_written(tmp_path):
    run, rec, gate = make(tmp_path, {"config/a.yaml": b"old\n"}, {"config/a.yaml": b"new\n"}, marker=False)
    with pytest.raises(G.OverlayRefused, match=G.MARKER):
        G.overlay(run, gate)
    assert (gate / "config/a.yaml").read_bytes() == b"new\n"


def test_a_recording_snapshot_that_no_longer_holds_the_recorded_bytes_is_refused(tmp_path):
    run, rec, gate = make(tmp_path, {"config/a.yaml": b"old\n"}, {"config/a.yaml": b"new\n"})
    (rec / "config/a.yaml").write_bytes(b"edited later\n")
    with pytest.raises(G.OverlayRefused, match="recording snapshot"):
        G.overlay(run, gate)
    assert (gate / "config/a.yaml").read_bytes() == b"new\n"


def test_absolute_inputs_outside_the_snapshot_are_left_to_the_replay(tmp_path):
    art = tmp_path / "artifacts/scene.usda"
    art.parent.mkdir(parents=True)
    art.write_bytes(b"changed")
    entry = {"path": str(art), "repo_relative": None, "sha256": sha(b"recorded"), "bytes": 8}
    run, rec, gate = make(tmp_path, {"config/a.yaml": b"same\n"}, {"config/a.yaml": b"same\n"}, extra_entries=[entry])
    out = G.overlay(run, gate)
    assert out["written"] == [] and art.read_bytes() == b"changed"


@pytest.mark.parametrize("rel", ["../escape.yaml", "/abs/but/no/repo_relative.yaml"])
def test_paths_that_leave_the_copy_are_refused(tmp_path, rel):
    entry = {"path": rel, "repo_relative": rel, "sha256": sha(b"x"), "bytes": 1}
    run, rec, gate = make(tmp_path, {}, {}, extra_entries=[entry])
    if rel.startswith("/"):
        assert G.overlay(run, gate)["written"] == []  # an absolute path is never written: the replay checks it
    else:
        with pytest.raises(G.OverlayRefused, match="leaves"):
            G.overlay(run, gate)


def test_two_recording_snapshots_in_one_record_are_refused(tmp_path):
    other = tmp_path / "snapshots/rec_2/src/n.py"
    other.parent.mkdir(parents=True)
    other.write_bytes(b"y")
    entry = {"path": str(other), "repo_relative": "src/n.py", "sha256": sha(b"y"), "bytes": 1}
    run, rec, gate = make(tmp_path, {"config/a.yaml": b"old\n"}, {"config/a.yaml": b"new\n"}, extra_entries=[entry])
    with pytest.raises(G.OverlayRefused, match="recording snapshot"):
        G.overlay(run, gate)


def test_the_cli_writes_its_report_and_exits_2_on_refusal(tmp_path):
    run, rec, gate = make(tmp_path, {"config/a.yaml": b"old\n"}, {"config/a.yaml": b"new\n"})
    report = tmp_path / "o.json"
    assert G.main([str(run), str(gate), "--json", str(report)]) == 0
    assert json.loads(report.read_text())["written"][0]["path"] == "config/a.yaml"
    run2, _, gate2 = make(tmp_path / "b", {"x.py": b"a"}, {"x.py": b"b"})
    assert G.main([str(run2), str(gate2), "--json", str(tmp_path / "o2.json")]) == 2
    assert "refused" in json.loads((tmp_path / "o2.json").read_text())
