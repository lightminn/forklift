"""Shelf discrimination study (docs/plans/2026-10-27-bottomless-and-bundled-pallets.md §3.1), CPU rig, dev only.

With bottomless pallets accepted, is there evidence that separates the shelf family (and the box stand) from a
bottomless pallet? Three views, each printing JSON lines for tools/hpc_sweep.sh:

    front      the pocket detector at the near capture's settings (``DetectorParams.derived_for``) from the
               carriage_low_measured mount, with the lower band gating and waived: its status and the gate terms of
               the pattern it selects (supports, lower, the per-opening upper counts, upper / sum(supports) of ADR
               0003 B-16), over a grid of distances and yaws, noise-free (1 mm rounding), the team noise table
               (frame_640) and, as a diagnostic, iid sigma = k d^2
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
from tools.measure_pocket_evidence import structure
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


def detector_setup(shape: str):
    prior = load_pallet_prior(ROOT / f"config/pallet_prior_{shape}.yaml")
    return prior, DetectorParams.derived_for(prior)


def front_scene(shape: str, kind: str, x_m: float, yaw_rad: float, noise: str, seed: int | None):
    """One front view: clean (1 mm rounding), the team table's frame_640, or the iid sigma = k d^2 diagnostic."""
    placed = scene_rig.place(boxes_for(shape, kind), x_m=x_m + CAMERA.xyz_m[0], yaw_rad=yaw_rad)
    if noise == "iid_k":
        return scene_rig.render(placed, camera=CAMERA, quantize=True, noise_k=NOISE_K, noise_seed=seed)
    clean = scene_rig.render(placed, camera=CAMERA, quantize=True)
    if noise == "clean":
        return clean
    # the team table's single-frame level: spatially correlated terms, dropout, flying pixels
    model = NoiseModel.from_yaml(TEAM_NOISE, seed=seed, target_fx=clean.intrinsics.fx, level="frame_640")
    return apply_noise(clean, np.zeros(np.shape(clean.depth_m), dtype=int), model)[0]


def selected_terms(scene, prior, params, lower_evidence: bool) -> dict:
    """Gate terms of the pattern the detector itself selects, and its status.

    Replays pocket_detector._detect's selection (planes within the residual bound, their opening patterns, score =
    (supports + deck) x (1 + behind fractions), then the nearer plane) and checks the winner against the detector's
    diagnostics (inliers, widths, support_min, lower, per-opening upper); ``replica_matches`` False means the terms
    are not the detector's and are left out. upper_over_supports is (upper_left + upper_right) / sum(supports), the
    per-opening form of ADR 0003 B-16.
    """
    result = detect_pockets(scene, prior, params, lower_evidence=lower_evidence)
    d = result.diagnostics
    out = {"status": result.observation.status, "reason": result.observation.reason}
    points, rays = detector._base_points(scene)
    cam = scene.base_from_optical.translation_m
    best = None
    if np.isfinite(points).all(axis=1).sum() >= params.min_plane_points:
        workspace = detector._filter_workspace(points, cam, prior, params)
        for index, plane in enumerate(detector._vertical_plane_candidates(workspace, cam, params)):
            if plane.residual_p95_m > params.max_plane_residual_m:
                continue
            patterns, _ = detector._opening_candidates(plane, prior, params, workspace, plane_index=index,
                                                       lower_evidence=lower_evidence)
            for pattern in patterns:
                behind = sum(detector._classify_opening_rays(scene, rays, plane, gap, prior, params).behind_fraction
                             for gap in pattern.gaps)
                key = ((pattern.support_count + pattern.deck_count) * (1 + behind),
                       -float(np.linalg.norm((plane.point - cam)[:2])))
                if best is None or key > best[0]:
                    best = (key, plane, pattern)
    if best is None:
        out["replica_matches"] = d.selected_lower is None
        return out
    _, plane, pattern = best
    out["replica_matches"] = (
        len(plane.points) == d.plane_inlier_count
        and tuple(high - low for low, high in reversed(pattern.gaps)) == tuple(d.opening_width_raw_m or ())
        and (pattern.support_min, pattern.lower, pattern.upper_left, pattern.upper_right)
        == (d.selected_support_min, d.selected_lower, d.selected_upper_left, d.selected_upper_right))
    if out["replica_matches"]:
        out.update({"support_sum": pattern.support_count, "support_min": pattern.support_min, "lower": pattern.lower,
                    "upper_left": pattern.upper_left, "upper_right": pattern.upper_right,
                    "upper_over_supports": (pattern.upper_left + pattern.upper_right) / pattern.support_count})
    return out


def cmd_front(a) -> int:
    prior, params = detector_setup(a.shape)
    noise_sha = hashlib.sha256(TEAM_NOISE.read_bytes()).hexdigest()
    conditions = [("clean", None)] + [(n, s) for s in NOISE_SEEDS for n in ("team_frame_640", "iid_k")]
    for x in a.distances:
        for yaw in YAWS_RAD:
            for noise, seed in conditions:
                scene = front_scene(a.shape, a.structure, x, yaw, noise, seed)
                row = {"shape": a.shape, "structure": a.structure, "x_m": x, "yaw_rad": yaw, "noise": noise,
                       "noise_seed": seed, "noise_config_sha256": noise_sha if noise == "team_frame_640" else None}
                for name, lower in (("gate", True), ("waive", False)):
                    row[name] = selected_terms(scene, prior, params, lower)
                print(json.dumps(row), flush=True)
    return 0


POSITIVES = ("pallet", "grounded")


def front_table(rows) -> dict:
    """Per (shape, noise, distance, structure) over the ``waive`` mode: valid count, cells, the valid ratio range, and
    the per-cell margin -- how far the structure's ratio lies outside the span of the positives' ratios in the same
    cell (distance, yaw, noise seed), negative when inside; cells where a positive or the structure is not valid are
    not counted. ``gate_valid`` is the same count with the lower band gating."""
    cells = {}
    for r in rows:
        cells.setdefault((r["shape"], r["noise"], r["x_m"], r["yaw_rad"], r["noise_seed"]), {})[r["structure"]] = r
    out = {}
    for r in rows:
        key = (r["shape"], r["noise"], r["x_m"], r["structure"])
        o = out.setdefault(key, {"cells": 0, "valid": 0, "gate_valid": 0, "ratios": [], "margins": []})
        o["cells"] += 1
        o["gate_valid"] += r["gate"]["status"] == "valid"
        if r["waive"]["status"] != "valid":
            continue
        o["valid"] += 1
        v = r["waive"]["upper_over_supports"]
        o["ratios"].append(v)
        if r["structure"] in POSITIVES:
            continue
        same = cells[(r["shape"], r["noise"], r["x_m"], r["yaw_rad"], r["noise_seed"])]
        pos = [same[p]["waive"]["upper_over_supports"] for p in POSITIVES
               if p in same and same[p]["waive"]["status"] == "valid"]
        if pos:
            o["margins"].append(max(min(pos) - v, v - max(pos)))
    return {k: {"cells": o["cells"], "valid": o["valid"], "gate_valid": o["gate_valid"],
                "ratio": [min(o["ratios"]), max(o["ratios"])] if o["ratios"] else None,
                "min_margin": min(o["margins"]) if o["margins"] else None, "margin_cells": len(o["margins"])}
            for k, o in sorted(out.items())}


def cmd_front_table(a) -> int:
    rows = [json.loads(line) for path in a.outputs for line in path.read_text().splitlines() if line.startswith("{")]
    mismatches = sum(not r[m]["replica_matches"] for r in rows for m in ("gate", "waive"))
    print(json.dumps({"rows": len(rows), "replica_mismatches": mismatches}))
    for (shape, noise, x, structure), v in front_table(rows).items():
        print(json.dumps({"shape": shape, "noise": noise, "x_m": x, "structure": structure, **v}))
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
    f.add_argument("--distances", type=float, nargs="+", default=DISTANCES_M,
                   help="camera to pallet centre, horizontal (m); the team table's single frames meet the pose bounds "
                        "only within 1.5 m (docs/validation/2026-10-17-recognition-measured-noise.md)")
    t = sub.add_parser("front-table")
    t.add_argument("outputs", type=Path, nargs="+", help="files of `front` JSON lines")
    s = sub.add_parser("side")
    s.add_argument("--group", choices=("epal6:x", "t11_06:x", "t11_06:y"), required=True)
    s.add_argument("--structure", required=True)
    s.add_argument("--seeds", type=int, default=12)
    m = sub.add_parser("summarise")
    m.add_argument("--evaluation", type=Path, required=True)
    m.add_argument("--catalogue", choices=("main", "bottomless"), required=True)
    a = ap.parse_args(argv)
    return {"front": cmd_front, "front-table": cmd_front_table, "side": cmd_side, "summarise": cmd_summarise}[a.cmd](a)


if __name__ == "__main__":
    sys.exit(main())
