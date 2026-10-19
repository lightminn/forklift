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


# Measured with the pre-refactor generator (stage 1B task 4, step 1). The two S7 entries were re-pinned on
# 2026-10-17 for the truth contract change (S7 side faces are entry faces): with truth.json left out, the
# digest of the same scene is identical before (286ecb3) and after (63d47d90... default, 7d1040b0... none).
GOLDEN = {
    ('default', 'n_negative_1000926'): '3c240092ca9418a25630c9ee135b960b25ad2e045dc312618485eb1347f0487d',
    ('default', 's2_epal6_1000122'): 'adcd96d2a651afcc9746690d56a8e6278f365722d4561cff7edc7335a5cb530c',
    ('default', 's2_t11_06_1000183'): '831d4bf609d3abc5da1bb7718b5578354998d0f8f9cf890d29a3284217f133fd',
    ('default', 's7_nonstandard_1000732'): '24cd3696342b11761e0c4956e51d38e693a7ca0da2158e715d3012f160bccc79',
    ('none', 'n_negative_1000926'): 'da6bdd6fe32590cadb91134598197cca64eb4de0a098ecd17217bf2a4d0cce13',
    ('none', 's2_epal6_1000122'): '9cb68ab0272be408b8627939692bc8811c523eb15326e924a7c34568701f063c',
    ('none', 's2_t11_06_1000183'): 'a34b33e77410a0e9b4941e8f274f1869507d5f9cf0b858a816988493208cc15f',
    ('none', 's7_nonstandard_1000732'): '09b1d47614e45fe0e3a9c514ce3617f135fc92b04efc1c1a86e03f66fa945396',
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


def _fit_fields(path):
    """Everything the recogniser decided -- candidates with their pockets and yaw, the selection, the
    failures -- and nothing the judge computed from truth (Codex 2026-10-16: compare pockets and the
    decision too, or a fit that copied truth pockets or a selection that read the target would pass)."""
    judged = {"case", "shape_key", "matched_instance", "errors", "truth_range_m", "dangerous"}
    rows = [json.loads(line) for line in (path / "candidates.jsonl").read_text().splitlines()]
    return [{k: v for k, v in r.items() if k not in judged} for r in rows]


def test_oracle_fit_judges_a_rig_dataset_and_writes_diagnostics(tmp_path):
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1,S2", "--limit", "2", "--noise", "none")
    out = tmp_path / "fit"
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "oracle-fit",
         "--output", str(out), "--diagnostics")
    metrics = json.loads((out / "metrics.json").read_text())
    assert metrics["detector"] == "oracle-fit" and metrics["fit_params"]["bin_m"] == 0.005
    rows = [json.loads(line) for line in (out / "candidates.jsonl").read_text().splitlines()]
    cands = [r for r in rows if r["row"] == "candidate"]
    assert cands and all(r["geometry"] in ("fitted", "ambiguous", "unfit") for r in cands)
    matched = [r for r in cands if r["errors"] is not None]
    assert matched and all(r["truth_range_m"] > 0 for r in matched)
    assert max(r["errors"][0] for r in matched) <= 0.02   # clean rig scenes: lateral within 20 mm
    dev = sum(json.loads(p.read_text())["split"] == "dev" for p in data.glob("*/scene.json"))
    assert sum(r["row"] == "decision" for r in rows) == dev   # one decision per judged scene


def test_oracle_fit_never_reads_truth_or_scene_labels(tmp_path):
    """Corrupt truth.json, the instruction truth and the scene's case/shape labels: the fit and the
    selection must not move (Codex 2026-10-16)."""
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1,S2", "--limit", "2", "--noise", "none")
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "oracle-fit",
         "--output", str(tmp_path / "a"), "--diagnostics")
    for scene_json in data.glob("*/scene.json"):
        meta = json.loads(scene_json.read_text())
        meta["shape_key"] = "t11_06" if meta["shape_key"] == "epal6" else "epal6"
        meta["case"] = "S1" if meta["case"] == "S2" else "S2"
        scene_json.write_text(json.dumps(meta))
        truth_json = scene_json.parent / "truth.json"
        truth = json.loads(truth_json.read_text())
        itruth = truth["instruction_truth"]
        if itruth["kind"] != "none" and len(truth["instances"]) > 1:
            # Point the instruction truth at another valid instance and flip its kind: a selection
            # that consults the target or the ambiguity label moves.
            itruth["target"] = (itruth["target"] + 1) % len(truth["instances"])
            itruth["kind"] = "ambiguous" if itruth["kind"] == "unambiguous" else "unambiguous"
        for inst in truth["instances"]:
            inst["spec"]["name"] = "x"
            for face in inst["faces"]:
                face["left_m"], face["right_m"] = [9.0, 9.0, 0.0], [9.0, 8.0, 0.0]
        truth_json.write_text(json.dumps(truth))
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "oracle-fit",
         "--output", str(tmp_path / "b"), "--diagnostics")
    a, b = _fit_fields(tmp_path / "a"), _fit_fields(tmp_path / "b")
    assert a == b and any(r["row"] == "candidate" for r in a) and any(r["row"] == "decision" for r in a)


def test_oracle_fit_proposes_every_instance_however_small(monkeypatch):
    """Codex review A round 2: masks under 50 px were dropped before the fit, so a mostly hidden pallet
    left no failure behind and the unresolved check could not see it."""
    from types import SimpleNamespace
    sys.path.insert(0, str(ROOT))
    from tools import evaluate_recognition as ev
    from forklift_core.perception.instance_fit import SceneFit

    ids = np.full((10, 10), -1, np.int32)
    ids[0:8, 0:8] = 0
    ids[9, 9] = 1                                    # a one-pixel visible part of a second pallet
    seen = []

    def fake_fit(scene, proposals, **kw):
        seen.extend(p.proposal_id for p in proposals)
        return SceneFit((), ())

    monkeypatch.setattr(ev, "fit_scene", fake_fit)
    sample = SimpleNamespace(instance_ids=ids, meta={"fork_spacing_m": 0.3725},
                             input=SimpleNamespace(scene=None, instruction={"present": False}))
    ev._oracle_fit(sample)
    assert seen == ["i0", "i1"]


def test_geometric_proposals_report_segmentation_and_never_select_automatically(tmp_path):
    # stage 3 tasks 1-2: the rig dataset is its own depth-aligned truth (no noise, no label shift)
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1,S2,N", "--limit", "3", "--noise", "none")
    seg = tmp_path / "seg"
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "geometric-proposals",
         "--segmentation-truth", str(data), "--output", str(seg))
    metrics = json.loads((seg / "metrics.json").read_text())["segmentation"]
    assert metrics["all"]["eligible"] > 0 and metrics["all"]["recall"] >= 0.6
    assert set(metrics["by_distance"]) == {"0.0-1.5", "1.5-2.0", "2.0-2.5", "2.5-3.0", "3.0-"}
    fit = tmp_path / "fit"
    _run("tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev", "--detector", "geometric-fit",
         "--segmentation-truth", str(data), "--output", str(fit))
    decisions = [json.loads(line) for line in (fit / "decisions.jsonl").read_text().splitlines()]
    assert decisions and all(d["decision"] != "selected" for d in decisions)  # unconfirmed: never auto-selected


def test_segmentation_needs_a_matching_truth_scene(tmp_path):
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1", "--limit", "1", "--noise", "none")
    other = tmp_path / "other"
    _run("tools/generate_recognition_scenes.py", "--out", str(other), "--cases", "S2", "--limit", "1", "--noise", "none")
    proc = subprocess.run([sys.executable, "tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev",
                           "--detector", "geometric-proposals", "--segmentation-truth", str(other),
                           "--output", str(tmp_path / "out")], cwd=ROOT, capture_output=True, text=True)
    assert proc.returncode != 0 and "segmentation truth" in proc.stderr


def test_segmentation_truth_with_shifted_noise_labels_is_refused(tmp_path):
    # Codex review 1: a noise pack shifts its labels with the RGB, so as truth it hides the registration error
    data = tmp_path / "data"
    _run("tools/generate_recognition_scenes.py", "--out", str(data), "--cases", "S1", "--limit", "1")
    proc = subprocess.run([sys.executable, "tools/evaluate_recognition.py", "--dataset", str(data), "--split", "dev",
                           "--detector", "geometric-proposals", "--segmentation-truth", str(data),
                           "--output", str(tmp_path / "out")], cwd=ROOT, capture_output=True, text=True)
    assert proc.returncode != 0 and "noise-free" in proc.stderr
