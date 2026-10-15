# tests/integration/test_recognition_cli.py
import csv
import hashlib
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from forklift_core.perception.recognition_dataset import load_recognition_sample
from tools import generate_recognition_scenes as gen

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


def test_source_revision_outside_a_git_checkout_is_recorded_as_unknown(tmp_path, monkeypatch):
    sys.path.insert(0, str(ROOT))
    from tools import evaluate_recognition as ev

    monkeypatch.setattr(ev, "ROOT", tmp_path)  # not a git checkout, no manifest
    source = ev._source(None)
    assert source["revision"] is None and "no git checkout" in source["reason"]


GOLDEN = {  # measured with the pre-refactor generator (stage 1B task 4, step 1)
    ('default', 'n_negative_1000926'): '3c240092ca9418a25630c9ee135b960b25ad2e045dc312618485eb1347f0487d',
    ('default', 's2_epal6_1000122'): 'adcd96d2a651afcc9746690d56a8e6278f365722d4561cff7edc7335a5cb530c',
    ('default', 's2_t11_06_1000183'): '831d4bf609d3abc5da1bb7718b5578354998d0f8f9cf890d29a3284217f133fd',
    ('default', 's7_nonstandard_1000732'): 'd82400922a204843aa03227f08238a0d3560204d84fb7255de9405f1314b6940',
    ('none', 'n_negative_1000926'): 'da6bdd6fe32590cadb91134598197cca64eb4de0a098ecd17217bf2a4d0cce13',
    ('none', 's2_epal6_1000122'): '9cb68ab0272be408b8627939692bc8811c523eb15326e924a7c34568701f063c',
    ('none', 's2_t11_06_1000183'): 'a34b33e77410a0e9b4941e8f274f1869507d5f9cf0b858a816988493208cc15f',
    ('none', 's7_nonstandard_1000732'): '9c613ae1a4e47c894612130ac8b3724a6a8612bc1c8341157e5b435399de7109',
}


def _content_digest(scene_dir):
    s = load_recognition_sample(scene_dir)
    scene, h = s.input.scene, hashlib.sha256()
    for array in (np.nan_to_num(scene.depth_m, nan=-1.0), scene.rgb, s.instance_ids,
                  np.asarray(scene.base_from_optical.rotation), np.asarray(scene.base_from_optical.translation_m)):
        h.update(np.ascontiguousarray(array).tobytes())
    i = scene.intrinsics
    for obj in ([i.width, i.height, i.fx, i.fy, i.cx, i.cy, i.frame_id], s.origin, s.meta, s.truth, s.instruction):
        h.update(json.dumps(obj, sort_keys=True).encode())
    return h.hexdigest()


@pytest.mark.parametrize("noise", ["default", "none"])
def test_generator_output_is_unchanged_by_the_record_refactor(tmp_path, noise):
    gen.main(["--out", str(tmp_path), "--cases", "S2,S7,N", "--limit", "1", "--noise", noise])
    got = {(noise, d.name): _content_digest(d) for d in sorted(tmp_path.iterdir())}
    assert got == {k: v for k, v in GOLDEN.items() if k[0] == noise}


def test_mixed_noise_levels_are_refused(tmp_path):
    from tools import apply_recognition_noise as app

    level = {"k_iid": 0.0, "components": [{"k": 0.001, "correlation_px_source": [2.0, 1.0]}]}
    cfg = tmp_path / "noise.yaml"
    cfg.write_text(json.dumps({"source": "t", "license": "CC-BY-4.0", "revision": "r", "files_sha256": "x",
                               "role": "provisional", "source_fx": 465.741156, "dropout_fraction": 0.0,
                               "levels": {"low": level, "high": level}}))
    gen.main(["--out", str(tmp_path / "clean"), "--cases", "S1", "--limit", "2", "--noise", "none"])
    dirs = sorted((tmp_path / "clean").iterdir())
    app.apply_to_scene(dirs[0], tmp_path / "mixed", cfg, 1.0, "low")
    app.apply_to_scene(dirs[1], tmp_path / "mixed", cfg, 1.0, "high")
    for d in (tmp_path / "mixed").iterdir():  # both into dev so the split filter keeps them
        meta = json.loads((d / "scene.json").read_text())
        meta["split"] = "dev"
        (d / "scene.json").write_text(json.dumps(meta))
    proc = subprocess.run([sys.executable, "tools/evaluate_recognition.py", "--dataset", str(tmp_path / "mixed"),
                           "--split", "dev", "--detector", "oracle", "--output", str(tmp_path / "out")],
                          cwd=ROOT, capture_output=True, text=True)
    assert proc.returncode != 0 and "exactly one noise configuration" in proc.stderr
