"""Shelf discrimination study (docs/plans/2026-10-27-bottomless-and-bundled-pallets.md §3.1), CPU rig, dev only.

With bottomless pallets accepted, is there evidence that separates the shelf family (and the box stand) from a
bottomless pallet? Three views, each printing JSON lines for tools/hpc_sweep.sh:

    front      the pocket detector at the near capture's settings (``DetectorParams.derived_for``) from the
               carriage_low_measured mount, with the lower band gating and waived, and its gate terms on the front
               plane: supports, lower, the per-opening upper counts and upper / sum(supports) (ADR 0003 B-16), over a
               grid of distances and yaws, noise-free (1 mm rounding), the team noise table (frame_640) and, as a diagnostic, iid sigma = k d^2
    side       the bottom-deck side evidence (perception.bottom_deck_evidence) on each side face from the side stand
               (tools/bottom_deck_scenes), team noise frame_640: present / absent / unobserved and why
    summarise  per-scene outcome of a dataset evaluated by tools/evaluate_recognition (--diagnostics), keyed by the
               negative's kind: whether a candidate was counted, and the geometric fit's reasons

The learned proposer is not run here (CPU rig, no RGB model); the geometric path stands in for it.

    python -m tools.shelf_discrimination_study front --shape t11_06 --structure shelf_rail
    python -m tools.shelf_discrimination_study side --group t11_06:x --structure shelf_rail
    python -m tools.shelf_discrimination_study summarise --evaluation OUT --catalogue bottomless
"""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

from forklift_core.perception import pocket_detector as detector
from forklift_core.perception.depth_noise import NoiseModel, apply_noise
from forklift_core.perception.pallet_prior import load_pallet_prior
from forklift_core.perception.pocket_detector import DetectorParams, detect_pockets
from tools import negative_structures, scene_rig
from tools.measure_pocket_evidence import _gate_terms, structure
from tools.recognition_scenes import _geometry

ROOT = Path(__file__).resolve().parents[1]
CAMERA = scene_rig.Camera((0.619, 0.0, 0.27), 0.10)  # carriage_low_measured
DISTANCES_M = (2.0, 2.5, 3.0, 3.5)  # camera to pallet centre, horizontal
YAWS_RAD = (0.0, 0.13, 0.26)
TEAM_NOISE = ROOT / "config/depth_noise_d435i_team.yaml"  # level frame_640: the team's single-frame measurement
NOISE_K = 0.006  # the iid diagnostic: sigma = k d^2, near the team table's frame k (0.0055-0.0065)
NOISE_SEEDS = (0, 1, 2)
FRONT_STRUCTURES = ("pallet", "grounded", "shelf", "shelf_closed", "shelf_rail", "shelf_raised", "shelf_legs",
                    "shelf_uprights", "box_stand")


def boxes_for(shape: str, kind: str):
    g = _geometry(shape)
    if kind in ("pallet", "grounded"):
        return structure(g, kind)
    return negative_structures.build(g, kind, np.random.default_rng(0))


def front_terms(scene, prior, params):
    """Gate terms on the front plane: the candidate plane with the largest support count."""
    points, _ = detector._base_points(scene)
    cam = scene.base_from_optical.translation_m
    workspace = detector._filter_workspace(points, cam, prior, params)
    best = None
    for plane in detector._vertical_plane_candidates(workspace, cam, params):
        terms = _gate_terms(plane, workspace, prior, params)
        if terms is not None and (best is None or sum(terms[0]) > sum(best[0])):
            best = terms
    if best is None:
        return None
    supports, lower, upper_left, upper_right = best
    total = sum(supports)
    return {"supports": list(supports), "lower": lower, "upper_left": upper_left, "upper_right": upper_right,
            "upper_over_supports": (upper_left + upper_right) / total if total else None}


def cmd_front(a) -> int:
    prior = load_pallet_prior(ROOT / f"config/pallet_prior_{a.shape}.yaml")
    params = DetectorParams.derived_for(prior)
    boxes = boxes_for(a.shape, a.structure)
    noise_sha = hashlib.sha256(TEAM_NOISE.read_bytes()).hexdigest()
    for x in DISTANCES_M:
        for yaw in YAWS_RAD:
            placed = scene_rig.place(boxes, x_m=x + CAMERA.xyz_m[0], yaw_rad=yaw)
            clean = scene_rig.render(placed, camera=CAMERA, quantize=True)
            conditions = [("clean", None, clean)]
            for seed in NOISE_SEEDS:
                # the team table's single-frame level (spatially correlated terms, dropout, flying pixels) ...
                model = NoiseModel.from_yaml(TEAM_NOISE, seed=seed, target_fx=clean.intrinsics.fx, level="frame_640")
                noisy, _ = apply_noise(clean, np.zeros(np.shape(clean.depth_m), dtype=int), model)
                conditions.append(("team_frame_640", seed, noisy))
                # ... and, as a diagnostic only, independent sigma = k d^2
                conditions.append(("iid_k", seed, scene_rig.render(placed, camera=CAMERA, quantize=True,
                                                                   noise_k=NOISE_K, noise_seed=seed)))
            for noise, seed, scene in conditions:
                row = {"shape": a.shape, "structure": a.structure, "x_m": x, "yaw_rad": yaw, "noise": noise,
                       "noise_seed": seed, "noise_config_sha256": noise_sha if noise == "team_frame_640" else None,
                       "terms": front_terms(scene, prior, params)}
                for name, lower in (("gate", True), ("waive", False)):
                    obs = detect_pockets(scene, prior, params, lower_evidence=lower).observation
                    row[name] = {"status": obs.status, "reason": obs.reason}
                print(json.dumps(row), flush=True)
    return 0


def cmd_side(a) -> int:
    from tools import bottom_deck_scenes as S
    from forklift_core.perception import bottom_deck_evidence as B

    shape, axis = a.group.split(":")
    theta = {"x": 35.0, "y": 0.0}[axis]
    for face in (1, -1):
        for seed in range(a.seeds):
            case = S.SideCase(shape, a.structure, face=face, theta_e_deg=theta, range_m=1.35, noise_seed=seed)
            scene, target, _ = S.render_side(case)
            ev = B.evaluate_side(scene, target, face=face)
            print(json.dumps({"group": a.group, "structure": a.structure, "face": face, "seed": seed,
                              "verdict": ev.verdict, "reason": ev.reason}), flush=True)
    return 0


def kind_of(recipe) -> str:
    """The negative's kind (or the positive case) of a catalogue recipe."""
    from tools import recognition_scenes as rs

    if recipe.case == "N2":
        return rs.n2_kind(recipe.rank)
    if recipe.case == "N":
        return ("grounded", "shelf", "blocks", "crates")[recipe.rank % 4]
    return f"{recipe.case}:{recipe.shape_key}"


def cmd_summarise(a) -> int:
    from tools import recognition_scenes as rs

    import csv

    recipes = rs.catalogue_bottomless() if a.catalogue == "bottomless" else rs.catalogue()
    kinds = {r.scene_id: kind_of(r) for r in recipes}
    with (a.evaluation / "verdicts.csv").open() as f:
        verdicts = {row["scene_id"]: row for row in csv.DictReader(f)}
    cands, fails, decisions = {}, {}, {}
    for line in (a.evaluation / "candidates.jsonl").read_text().splitlines():
        r = json.loads(line)
        if r["row"] == "candidate":
            cands.setdefault(r["scene_id"], []).append(r)
        elif r["row"] == "failure":
            fails.setdefault(r["scene_id"], []).append(r["failure"])
        elif r["row"] == "decision":
            decisions[r["scene_id"]] = r["decision"]
    out = {}
    for sid, v in verdicts.items():
        o = out.setdefault(kinds.get(sid, "?"), {"scenes": 0, "negative_fp": 0, "miss": 0, "selected": 0,
                                                  "counted": Counter(), "identity": Counter(),
                                                  "candidate_reasons": Counter(), "fit_failures": Counter()})
        o["scenes"] += 1
        o["negative_fp"] += v.get("negative_fp") == "True"
        o["miss"] += v.get("miss") == "True"
        o["selected"] += decisions.get(sid) == "selected"
        for c in cands.get(sid, []):
            o["counted"][str(c.get("counted"))] += 1
            o["identity"][str(c.get("identity"))] += 1
            for reason in c.get("reasons") or []:
                o["candidate_reasons"][reason] += 1
        for failure in fails.get(sid, []):
            o["fit_failures"][failure] += 1
    print(json.dumps({k: {n: dict(x) if isinstance(x, Counter) else x for n, x in v.items()}
                      for k, v in sorted(out.items())}, indent=1))
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    f = sub.add_parser("front")
    f.add_argument("--shape", choices=("epal6", "t11_06"), required=True)
    f.add_argument("--structure", choices=FRONT_STRUCTURES, required=True)
    s = sub.add_parser("side")
    s.add_argument("--group", choices=("epal6:x", "t11_06:x", "t11_06:y"), required=True)
    s.add_argument("--structure", required=True)
    s.add_argument("--seeds", type=int, default=12)
    m = sub.add_parser("summarise")
    m.add_argument("--evaluation", type=Path, required=True)
    m.add_argument("--catalogue", choices=("main", "bottomless"), required=True)
    a = ap.parse_args(argv)
    return {"front": cmd_front, "side": cmd_side, "summarise": cmd_summarise}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
