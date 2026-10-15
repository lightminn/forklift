#!/usr/bin/env python3
"""Judge a schema-v2 dataset with the current detector (baseline) or the truth (oracle)."""

import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
import time
from dataclasses import asdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from forklift_core.perception.evaluation_multi import CandidateView, TargetDecision, judge_scene, summarize
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.recognition_dataset import load_recognition_sample, read_meta, scene_digest

ROOT = Path(__file__).resolve().parents[1]
PRIORS = ("config/pallet_prior_epal6.yaml", "config/pallet_prior_t11_06.yaml")


def _baseline(sample):
    """The current detector, run once per known prior. It never reads scene labels."""
    cands = []
    for k, path in enumerate(PRIORS):
        prior = load_pallet_prior(ROOT / path)
        obs = detect_pockets(sample.input.scene, prior, DetectorParams.derived_for(prior)).observation
        if obs.status == "valid":
            cands.append(CandidateView(f"baseline{k}", obs.insertion_yaw_rad, obs.left.center_m, obs.right.center_m,
                                       True, True, True, (), None))
    if not cands:
        return cands, TargetDecision("none", None, ())
    instr = sample.input.instruction
    if instr.get("present"):
        def dist(c):
            return math.hypot((c.left_m[0] + c.right_m[0]) / 2 - instr["x_m"], (c.left_m[1] + c.right_m[1]) / 2 - instr["y_m"])
        return cands, TargetDecision("selected", min(cands, key=dist).candidate_id, ())
    return cands, TargetDecision("selected", cands[0].candidate_id, ())


def _oracle(sample):
    cands = []
    for inst in sample.truth["instances"]:
        faces = [f for f in inst["faces"] if f["accessible"]]
        if not faces:
            continue
        f = faces[0]
        cands.append(CandidateView(f"o{inst['instance_id']}", f["insertion_yaw_rad"], tuple(f["left_m"]),
                                   tuple(f["right_m"]), True, True, f["fit"]["pickable"], tuple(f["fit"]["reasons"]),
                                   inst["layer"]))
    itruth = sample.truth["instruction_truth"]
    if itruth["kind"] == "ambiguous":
        return cands, TargetDecision("ask_operator", None, ())
    if itruth["kind"] == "unambiguous":
        cid = f"o{itruth['target']}"
        cand = next((c for c in cands if c.candidate_id == cid), None)
        if cand is None:
            return cands, TargetDecision("none", None, ())
        return cands, TargetDecision("selected" if cand.pickable else "not_pickable", cid, cand.reasons)
    pickable = [c for c in cands if c.pickable]
    return cands, (TargetDecision("selected", pickable[0].candidate_id, ()) if pickable else TargetDecision("none", None, ()))


def _source(manifest):
    """The evaluated source: a manifest (remote runs), the git checkout, or an explicit unknown."""
    if manifest is not None:
        return json.loads(Path(manifest).read_text())
    rev = subprocess.run(["git", "rev-parse", "HEAD"], cwd=ROOT, capture_output=True, text=True)
    if rev.returncode != 0:
        return {"revision": None, "dirty": None,
                "reason": "no git checkout and no --source-manifest; this run is not tied to a revision"}
    dirty = subprocess.run(["git", "status", "--porcelain"], cwd=ROOT, capture_output=True, text=True, check=True)
    return {"revision": rev.stdout.strip(), "dirty": bool(dirty.stdout.strip())}


def main(argv=None):
    p = argparse.ArgumentParser()
    p.add_argument("--dataset", type=Path, required=True)
    p.add_argument("--split", choices=("dev", "eval"), required=True)
    p.add_argument("--detector", choices=("baseline", "oracle"), required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--source-manifest", type=Path, default=None)
    a = p.parse_args(argv)
    run = {"baseline": _baseline, "oracle": _oracle}[a.detector]
    verdicts, digest, sources = [], hashlib.sha256(), set()
    for scene_json in sorted(a.dataset.glob("*/scene.json")):
        if read_meta(scene_json.parent)["split"] != a.split:
            continue
        sample = load_recognition_sample(scene_json.parent)
        digest.update(scene_digest(scene_json.parent).encode())
        sources.add(sample.meta["render_source"])
        start = time.perf_counter()
        cands, decision = run(sample)
        verdicts.append(judge_scene(sample.meta, sample.truth, sample.input.instruction, cands, decision,
                                    elapsed_s=time.perf_counter() - start))
    if len(sources) != 1:
        raise SystemExit(f"dataset must hold exactly one render source, found {sorted(sources)}")
    a.output.mkdir(parents=True, exist_ok=True)
    metrics = summarize(verdicts, split=a.split, render_source=sources.pop())
    elapsed = sorted(v.elapsed_s for v in verdicts)
    metrics.update({"detector": a.detector, "dataset_sha256": digest.hexdigest(), "source": _source(a.source_manifest),
                    "elapsed_s": {"n": len(elapsed), "p50": elapsed[len(elapsed) // 2],
                                  "p95": elapsed[min(len(elapsed) - 1, int(0.95 * len(elapsed)))], "max": elapsed[-1]}})
    (a.output / "metrics.json").write_text(json.dumps(metrics, indent=1, default=float))
    with (a.output / "verdicts.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(asdict(verdicts[0]).keys()))
        w.writeheader()
        for v in verdicts:
            row = asdict(v)
            row["elapsed_s"] = ""
            w.writerow(row)


if __name__ == "__main__":
    main()
