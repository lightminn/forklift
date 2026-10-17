#!/usr/bin/env python3
"""Judge a schema-v2 dataset with the current detector (baseline), the truth (oracle), or the
stage 2 geometric fit on truth instance masks (oracle-fit)."""

import argparse
import csv
import hashlib
import json
import math
import subprocess
import sys
import time
from collections import Counter
from dataclasses import asdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from forklift_core.perception.evaluation_multi import (
    CandidateView, TargetDecision, _match, candidate_danger, judge_scene, summarize,
)
from forklift_core.perception.fork_fit import ForkSetup, LoadDeclaration, load_site_defaults
from forklift_core.perception.instance_fit import FitParams, fit_scene
from forklift_core.perception.instance_proposals import InstanceProposal
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from forklift_core.perception.recognition_dataset import load_recognition_sample, read_meta, scene_digest
from forklift_core.perception.target_selection import assign_layers, select_target

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


_KNOWN = {}


def _known():
    """Known shapes and their site default loads (loaded once; tools.recognition_scenes reads config)."""
    if not _KNOWN:
        from tools import recognition_scenes as rs

        specs = {k: rs.known_spec(k) for k in ("epal6", "t11_06")}
        site = load_site_defaults(ROOT / "config/site_load_defaults.yaml", specs)
        _KNOWN.update(specs=specs, site={specs[k].name: v for k, v in site.items()})
    return _KNOWN["specs"], _KNOWN["site"]


def _load_for(instruction, site):
    given = instruction.get("load") if instruction.get("present") else None
    declared = None if given is None else LoadDeclaration(**given)
    # A non-standard reading (spec None) has no site default: only an instructed load applies.
    return lambda spec: declared if declared is not None else (None if spec is None else site.get(spec.name))


def _oracle_fit(sample):
    """Truth instance masks as proposals (source oracle), then the stage 2 fit and selection.

    Reads the instance id image, the instruction and the scene's fork spacing (a robot setting) --
    never truth.json, the case or the shape key. Distractor and background pixels (ids < 0) are
    never proposed, so negative scenes cannot produce a candidate here: their false-positive rate
    under oracle-fit is zero by construction, not a result.
    """
    specs, site = _known()
    ids = sample.instance_ids
    # Every visible instance is proposed, however small: one too small to fit still leaves a
    # failure that keeps an instruction ambiguous (Codex review A round 2).
    proposals = [InstanceProposal(f"i{k}", ids == k, 1.0, "oracle") for k in sorted(int(v) for v in np.unique(ids) if v >= 0)]
    fitted = fit_scene(sample.input.scene, proposals, specs=specs, setup=ForkSetup(sample.meta["fork_spacing_m"]),
                       load_for=_load_for(sample.input.instruction, site))
    layers = assign_layers(fitted.candidates)
    # A pallet seen but not fitted inside the instruction's radius keeps the target ambiguous (Codex 2026-10-16)
    sel = select_target(fitted.candidates, sample.input.instruction, unresolved=fitted.failures)
    views = [CandidateView(c.candidate_id, c.insertion_yaw_rad, c.left.centre_m, c.right.centre_m,
                           c.geometry == "fitted", c.identity == "confirmed", c.pickable, c.fit_reasons,
                           layers[c.candidate_id][1]) for c in fitted.candidates]
    return views, TargetDecision(sel.status, sel.candidate_id, sel.reasons), fitted


def _diagnostics(sample, views, decision, fitted):
    """One row per candidate (with the judge's matching and danger), per decision and per failure."""
    meta, instances = sample.meta, sample.truth["instances"]
    counted = [v for v in views if v.counted]
    pairs, costs = _match(counted, instances)
    camera = np.asarray(sample.input.scene.base_from_optical.translation_m, float)[:2]
    by_id = {c.candidate_id: c for c in (fitted.candidates if fitted is not None else ())}
    rows = []
    for view in views:
        ci = next((k for k, c in enumerate(counted) if c.candidate_id == view.candidate_id), None)
        matched = None if ci is None else pairs.get(ci)
        errors = truth_range = None
        if matched is not None:
            (_, err), face = costs[ci, matched]
            errors = [float(e) for e in err]
            mid = (np.asarray(face["left_m"][:2]) + np.asarray(face["right_m"][:2])) / 2
            truth_range = float(np.hypot(*(mid - camera)))
        row = {"row": "candidate", "scene_id": meta["scene_id"], "case": meta["case"], "shape_key": meta["shape_key"],
               "candidate_id": view.candidate_id, "insertion_yaw_rad": view.insertion_yaw_rad,
               "left_m": list(view.left_m), "right_m": list(view.right_m), "pickable": view.pickable,
               "fit_reasons": list(view.reasons), "layer_abs": view.layer_abs, "counted": view.counted,
               "matched_instance": matched, "errors": errors, "truth_range_m": truth_range,
               "dangerous": None if ci is None else candidate_danger(ci, view, pairs, costs, instances)}
        cand = by_id.get(view.candidate_id)
        if cand is not None:
            row.update({"proposal_id": cand.proposal_id, "hypothesis": cand.hypothesis, "geometry": cand.geometry,
                        "identity": cand.identity, "reasons": list(cand.reasons), "range_m": cand.range_m,
                        "support_z_m": cand.support_z_m, "support_state": cand.support_state, "score": cand.score})
        rows.append(row)
    rows.append({"row": "decision", "scene_id": meta["scene_id"], "decision": decision.status,
                 "candidate_id": decision.candidate_id, "decision_reasons": list(decision.reasons)})
    for failure in (fitted.failures if fitted is not None else ()):
        rows.append({"row": "failure", "scene_id": meta["scene_id"], "proposal_id": failure.proposal_id,
                     "failure": failure.reason, "centre_xy": failure.centre_xy, "extent_m": failure.extent_m})
    return rows


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
    p.add_argument("--detector", choices=("baseline", "oracle", "oracle-fit"), required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--source-manifest", type=Path, default=None)
    p.add_argument("--diagnostics", action="store_true", help="write candidates.jsonl (one row per candidate, "
                   "decision and proposal failure)")
    a = p.parse_args(argv)
    run = {"baseline": _baseline, "oracle": _oracle, "oracle-fit": _oracle_fit}[a.detector]
    diag, failures = [], Counter()
    verdicts, digest, sources, signatures = [], hashlib.sha256(), set(), set()
    for scene_json in sorted(a.dataset.glob("*/scene.json")):
        if read_meta(scene_json.parent)["split"] != a.split:
            continue
        sample = load_recognition_sample(scene_json.parent)
        digest.update(scene_digest(scene_json.parent).encode())
        sources.add(sample.meta["render_source"])
        start = time.perf_counter()
        cands, decision, *extra = run(sample)
        verdicts.append(judge_scene(sample.meta, sample.truth, sample.input.instruction, cands, decision,
                                    elapsed_s=time.perf_counter() - start))
        fitted = extra[0] if extra else None
        if fitted is not None:
            failures.update(f.reason for f in fitted.failures)
        if a.diagnostics:
            diag += _diagnostics(sample, cands, decision, fitted)
        signatures.add(verdicts[-1].noise_signature)
    if len(sources) != 1:
        raise SystemExit(f"dataset must hold exactly one render source, found {sorted(sources)}")
    if len(signatures) != 1:
        raise SystemExit(f"dataset must hold exactly one noise configuration, found {sorted(map(str, signatures))}")
    a.output.mkdir(parents=True, exist_ok=True)
    metrics = summarize(verdicts, split=a.split, render_source=sources.pop())
    elapsed = sorted(v.elapsed_s for v in verdicts)
    metrics.update({"detector": a.detector, "dataset_sha256": digest.hexdigest(), "source": _source(a.source_manifest),
                    "elapsed_s": {"n": len(elapsed), "p50": elapsed[len(elapsed) // 2],
                                  "p95": elapsed[min(len(elapsed) - 1, int(0.95 * len(elapsed)))], "max": elapsed[-1]}})
    if a.detector == "oracle-fit":
        metrics.update({"fit_params": asdict(FitParams()), "proposal_failures": dict(failures)})
    (a.output / "metrics.json").write_text(json.dumps(metrics, indent=1, default=float))
    if a.diagnostics:
        (a.output / "candidates.jsonl").write_text("".join(json.dumps(r, default=float) + "\n" for r in diag))
    with (a.output / "verdicts.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(asdict(verdicts[0]).keys()))
        w.writeheader()
        for v in verdicts:
            row = asdict(v)
            row["elapsed_s"] = ""
            w.writerow(row)


if __name__ == "__main__":
    main()
