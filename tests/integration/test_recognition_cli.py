# tests/integration/test_recognition_cli.py
import csv
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def _run(*args):
    return subprocess.run([sys.executable, *args], cwd=ROOT, check=True, capture_output=True, text=True)


def _rows(path):
    with (path / "verdicts.csv").open() as f:
        return [{k: v for k, v in row.items() if k != "elapsed_s"} for row in csv.DictReader(f)]


def test_oracle_self_check_and_label_free_baseline(tmp_path):
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1,S2,S7", "--limit", "3", "--noise", "none")
    assert len(list(data.glob("*/scene.json"))) == 3 * 2 * 2 + 3
    for detector in ("oracle", "baseline"):
        out = tmp_path / detector
        _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", detector, "--output", str(out))
        metrics = json.loads((out / "metrics.json").read_text())
        assert metrics["render_source"] == "rig" and not metrics["official"]
        if detector == "oracle":
            for name, gate in metrics["gates"].items():
                if gate.get("n") and name.endswith((":miss", ":extra", ":dangerous_observed_zero", ":target_error", ":fit_error")):
                    assert gate["failures"] == 0, name
    # Baseline must not consult the scene's shape label: corrupt it and expect identical verdicts.
    first = next(data.glob("s1_epal6_*/scene.json"))
    meta = json.loads(first.read_text())
    meta["shape_key"] = "t11_06"
    first.write_text(json.dumps(meta))
    out2 = tmp_path / "baseline2"
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "baseline", "--output", str(out2))
    strip = lambda rows: [{k: v for k, v in r.items() if k not in ("shape_key", "population")} for r in rows]
    assert strip(_rows(tmp_path / "baseline")) == strip(_rows(out2))


def test_known_shape_truth_records_the_site_default_source(tmp_path):
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1", "--limit", "2", "--noise", "none")
    truths = sorted(data.glob("s1_*/truth.json"))
    assert len(truths) == 4  # S1 x two shapes x --limit 2; S1 instructions carry no load
    for truth in truths:
        inst = json.loads(truth.read_text())["instances"][0]
        assert {f["fit"]["load_source"] for f in inst["faces"]} == {"site_default"}


def test_digest_only_prints_the_catalogue_digest():
    out = _run("tools/generate_recognition_scenes.py", "--out", "/nonexistent", "--digest-only").stdout
    assert "catalogue_sha256" in out and "numpy" in out
