#!/usr/bin/env python3
"""P0-5b offline check (plan 2026-11-11-sim-to-real-bringup 2.3b): the far recognition's fit run on the team's saved
D435i frames of the chassis's bundled pallet (ws1 data/d435i_noise/2026-10-16, Z16 depth only, 30 frames per capture).
Nothing in the recognition is changed; this reports, it does not tune.

Arms (proposer x mount):
  proposer  geometric: geometric_proposals.propose -- the depth-only proposer, the learned path's fallback (the learned
            YOLO11s proposer needs RGB, which these captures do not have). Its proposals are always unconfirmed
            (geometric_unsplit), so the selection can never pick them: what is judged is the fit, not the choice.
            roi: one oracle proposal per frame, the capture's hand-drawn ROI box cut to points above the levelled
            floor and before the front + 0.5 m -- a stand-in for a perfect segmentation, to separate the proposer
            from the fitter. It reads the capture's ROI, which no runtime has.
  mount     nominal: the note's level camera at 0.27 m (E4's). level: the floor fitted per capture (E4's floor_fit,
            pallet ROI left out) moved onto z = 0 -- a floor alignment, not an extrinsic calibration.

Per frame the toy proposal is the one covering most of the ROI's pallet points (at least half of them); its candidate
is compared with the capture's reference: the front x and middle y of the levelled 30-frame median (E4 front_of), yaw
0 (the pallet was set square by hand, not measured) and the tape's lane spacing. ``consistent`` is E4's
"valid_right" box: the two pocket centres' midpoint within 6 cm of the front along x and across, yaw within 15 deg;
spacing is reported apart (a non-standard reading measures its own lanes). There is no pose truth.

  PYTHONPATH=<repo>/src python real_offline.py DATA_DIR --json OUT.json [--jobs N]
"""

import argparse
import dataclasses
import json
import math
import multiprocessing
import sys
import time
import warnings
from pathlib import Path

import numpy as np
from PIL import Image

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO), str(REPO / "tools"), str(REPO / "docs/validation/2026-11-07-e4-toy-pallet-diagnosis")]

import e4_diagnose as E  # noqa: E402
import near_refit_compare as N  # noqa: E402
from forklift_core.perception import geometric_proposals as G  # noqa: E402
from forklift_core.perception.fork_fit import ForkSetup, load_site_defaults  # noqa: E402
from forklift_core.perception import instance_fit as F  # noqa: E402
from forklift_core.perception.instance_fit import FitParams, fit_scene  # noqa: E402
from forklift_core.perception.instance_proposals import InstanceProposal  # noqa: E402
from forklift_core.perception.pallet_geometry import load_pallet_geometry  # noqa: E402
from forklift_core.perception.pallet_model import KNOWN_ENTRY_AXES, spec_from_geometry  # noqa: E402
from forklift_core.perception.target_selection import select_target  # noqa: E402

FORK_SPACING_M = 0.3725  # sim/models/dls08_measured/parameters.yaml fork_spacing_m (the measured chassis)
LANES_M = {"toy": 0.19, "toy30": 0.12}  # the tape's leg spacing (dimensions.txt)
DEPTH_M = {"toy": 0.30, "toy30": 0.42}  # the pallet's extent along the camera axis
FACE_M, YAW_DEG, SPACING_M = 0.06, 15.0, 0.03
COVER = 0.5
ZONE_RADIUS_M = 0.5
WIDE_SCALE_M = 0.025  # diagnostic arm only: the face's robust-scale cap opened (FitParams.max_face_scale_m 0.01)
ARMS = [(p, m) for p in ("geometric", "roi") for m in ("nominal", "level")] + [("roi_wide", "level"),
                                                                              ("geometric_wide", "level")]


def _wide_params(**over) -> FitParams:
    return FitParams(**{"max_face_scale_m": WIDE_SCALE_M, **over})


def face_diagnosis(points, cam, params: FitParams) -> dict:
    """Why fit_faces keeps or drops the first plane of these points: fit_faces' own first round, re-run with its
    checks recorded (rng seed 0, so the plane may differ from the one a fit_scene call drew; diagnostic only)."""
    out = {"points": int(len(points))}
    if len(points) < params.min_points:
        return {**out, "stage": "min_points"}
    zmax = float(points[:, 2].max())
    sub = points[points[:, 2] < zmax - params.top_exclude_m, :2]
    out["body_points"] = int(len(sub))
    if len(sub) < params.min_points:
        return {**out, "stage": "body_points"}
    reach0 = float(np.median(np.hypot(*(points[:, :2] - cam[:2]).T)))
    tol = max(params.ransac_tol_m, 0.5 * params.ray_margin_k * reach0 * reach0)
    rng, best = np.random.default_rng(0), None
    for _ in range(params.ransac_iters):
        i, j = rng.choice(len(sub), 2, replace=False)
        d = sub[j] - sub[i]
        length = float(np.hypot(*d))
        if length < 0.05:
            continue
        n = np.array([-d[1], d[0]]) / length
        inliers = np.abs((sub - sub[i]) @ n) < tol
        if best is None or int(inliers.sum()) > best[0]:
            best = (int(inliers.sum()), sub[i], n)
    if best is None or best[0] < params.min_points:
        return {**out, "stage": "ransac"}
    c0, n0 = F._snap_to_layer(sub, best[1], best[2], tol)
    near = np.abs((sub - c0) @ n0) < tol
    centre, normal, scale = F._irls_line(sub[near], c0, n0, cam[:2], params.irls_iters)
    if (cam[:2] - centre) @ normal < 0:
        normal = -normal
    reach = F._reach(cam, centre)
    limit = max(params.max_face_scale_m, 0.5 * params.ray_margin_k * reach * reach)
    margin = max(params.exterior_margin_m, 3 * scale, params.ray_margin_k * reach * reach)
    exterior = float(((points[:, :2] - centre) @ normal > margin).mean())
    out.update(inliers=best[0], scale_m=float(scale), scale_limit_m=float(limit), exterior=exterior,
               yaw_deg=float(np.degrees(np.arctan2(normal[1], normal[0]))))
    out["stage"] = "scale" if scale > limit else "exterior" if exterior > params.exterior_max_fraction else "kept"
    return out


def toy_component(scene, roi_mask, fit_params: FitParams) -> dict:
    """The geometric proposer's own view of the pallet: the depth component covering most of the ROI's pallet
    points (geometric_proposals.propose's first steps), its size and width, and its first face's fate."""
    params = G.GeometricParams()
    depth = np.asarray(scene.depth_m, dtype=float)
    valid = np.isfinite(depth) & (depth > 0)
    rays = F.pixel_rays(scene)
    cam = F.camera_xyz(scene)
    points = cam + np.where(valid, depth, 0.0)[..., None] * rays
    r = np.hypot(points[..., 0] - cam[0], points[..., 1] - cam[1])
    keep = valid & (r <= params.range_max_m) & (points[..., 2] > params.floor_a_m + params.floor_b_per_m * r * r)
    n_roi = int(roi_mask.sum())
    out = {"roi_kept": float((keep & roi_mask).sum() / max(1, n_roi))}
    labels = G._components(keep, depth, params)
    hit = labels[roi_mask & (labels >= 0)]
    if not len(hit):
        return {**out, "stage": "no_component"}
    lab, count = np.unique(hit, return_counts=True)
    lab = int(lab[np.argmax(count)])
    comp = labels == lab
    pts = points[comp]
    xy = pts[:, :2]
    bearing = np.arctan2(xy[:, 1] - cam[1], xy[:, 0] - cam[0])
    width = float(np.median(np.hypot(xy[:, 0] - cam[0], xy[:, 1] - cam[1])) * (bearing.max() - bearing.min()))
    out.update(pixels=int(comp.sum()), cover=float(count.max() / max(1, n_roi)),
               purity=float((comp & roi_mask).sum() / comp.sum()), width_m=width,
               x_range_m=[float(np.percentile(xy[:, 0], q)) for q in (2, 98)],
               faces=len(F.fit_faces(pts, cam, fit_params, np.random.default_rng(0))),
               first_face=face_diagnosis(pts, cam, fit_params))
    if int(comp.sum()) < params.min_pixels:
        out["stage"] = "min_pixels"
    elif not out["faces"]:
        out["stage"] = "no_face"
    elif width > params.width_max_m:
        out["stage"] = "oversize"
    else:
        out["stage"] = "proposed"
    return out


def known_specs() -> dict:
    return {k: spec_from_geometry(load_pallet_geometry(REPO / f"config/pallet_geometry_{k}.yaml"),
                                  entry_axes=KNOWN_ENTRY_AXES[k]) for k in ("epal6", "t11_06")}


def roi_pixels(scene, roi, front_x: float):
    """The ROI's pixels on the pallet in this scene: above the floor (z > 6 mm) and before front + 0.5 m."""
    x0, y0, x1, y1 = roi
    grid = E.base_grid(scene)
    mask = np.zeros(scene.depth_m.shape, bool)
    sub = grid[y0:y1, x0:x1]
    ok = np.isfinite(sub).all(axis=2) & (sub[..., 2] > 0.006) & (sub[..., 0] < front_x + 0.5)
    mask[y0:y1, x0:x1] = ok
    return mask


def candidate_record(c, ref: dict | None, lanes: float | None) -> dict:
    left, right = np.asarray(c.left.centre_m), np.asarray(c.right.centre_m)
    mid = (left + right) / 2
    rec = {"hypothesis": c.hypothesis, "geometry": c.geometry, "identity": c.identity, "axis": c.axis,
           "pickable": c.pickable, "reasons": list(c.reasons), "fit_reasons": list(c.fit_reasons),
           "pending": list(c.pending), "support_state": c.support_state, "mid_m": [float(v) for v in mid],
           "yaw_deg": float(np.degrees(c.insertion_yaw_rad)), "spacing_m": float(np.linalg.norm(left[:2] - right[:2])),
           "widths_m": [float(c.left.width_m), float(c.right.width_m)],
           "heights_m": [float(c.left.height_m), float(c.right.height_m)]}
    if ref is not None:
        yaw = abs((rec["yaw_deg"] + 180.0) % 360.0 - 180.0)
        rec["err"] = {"x_m": float(mid[0] - ref["front_x"]), "y_m": float(mid[1] - ref["front_y"]),
                      "yaw_deg": float(min(yaw, 180.0 - yaw)), "spacing_m": float(rec["spacing_m"] - lanes)}
        e = rec["err"]
        rec["consistent"] = abs(e["x_m"]) <= FACE_M and abs(e["y_m"]) <= FACE_M and e["yaw_deg"] <= YAW_DEG
        rec["spacing_ok"] = abs(e["spacing_m"]) <= SPACING_M
    return rec


def frame_record(scene, proposals, failures, specs, setup, load_for, ref, roi_mask, lanes, kind, threshold,
                 params: FitParams = FitParams()) -> dict:
    t0 = time.perf_counter()
    fit = fit_scene(scene, proposals, specs=specs, setup=setup, load_for=load_for, params=params,
                    identity_threshold=threshold)
    fit_ms = 1e3 * (time.perf_counter() - t0)
    unresolved = tuple(fit.failures) + tuple(failures)
    rec = {"proposals": len(proposals), "proposer_failures": [f.reason for f in failures],
           "fit_failures": [f.reason for f in fit.failures], "candidates": len(fit.candidates), "fit_ms": fit_ms}
    by_prop = {c.proposal_id: c for c in fit.candidates}
    fail_by_prop = {f.proposal_id: f.reason for f in fit.failures}
    if kind == "wall":
        rec["wall_candidates"] = [candidate_record(c, None, None) for c in fit.candidates]
        return rec
    n_roi = int(roi_mask.sum())
    cover = [(int((p.mask & roi_mask).sum()) / max(1, n_roi), p) for p in proposals]
    best = max(cover, key=lambda t: t[0], default=(0.0, None))
    rec["roi_points"] = n_roi
    rec["toy_cover"] = float(best[0])
    toy = best[1] if best[0] >= COVER else None
    if toy is None:
        rec["toy"] = "no_proposal"
    else:
        rec["toy_proposal_px"] = int(toy.mask.sum())
        rec["toy_purity"] = float((toy.mask & roi_mask).sum() / max(1, toy.mask.sum()))
        if toy.proposal_id in by_prop:
            rec["toy"] = "candidate"
            rec["toy_candidate"] = candidate_record(by_prop[toy.proposal_id], ref, lanes)
        else:
            rec["toy"] = "fit_failure:" + fail_by_prop.get(toy.proposal_id, "unknown")
    rec["others"] = [candidate_record(c, None, None) for c in fit.candidates
                     if toy is None or c.proposal_id != toy.proposal_id]
    instruction = {"present": True, "x_m": ref["front_x"] + DEPTH_M[kind] / 2, "y_m": ref["front_y"],
                   "radius_m": ZONE_RADIUS_M}
    sel = select_target(fit.candidates, instruction, unresolved=unresolved)
    rec["decision"] = {"status": sel.status, "reasons": list(sel.reasons),
                       "toy": bool(toy is not None and sel.candidate_id is not None
                                   and by_prop.get(toy.proposal_id) is not None
                                   and by_prop[toy.proposal_id].candidate_id == sel.candidate_id)}
    return rec


def run_capture(task: tuple) -> dict:
    root, name = task
    capture = Path(root) / name
    meta = json.loads((capture / "capture.json").read_text())
    kind = name.split("_")[0]
    distance = float(meta["setup"]["distance_m"])
    roi = meta["setup"]["roi"]
    frames = [np.asarray(Image.open(p)) for p in sorted(capture.glob("z16_*.png"))]
    scenes = [N.toy_input(meta, f)[0] for f in frames]
    fits = []
    for scene in scenes[:: max(1, len(scenes) // 6)]:
        grid = E.base_grid(scene).copy()
        x0, y0, x1, y1 = roi
        grid[y0:y1, x0:x1] = np.nan
        p = grid.reshape(-1, 3)
        fits.append(E.floor_fit(p[np.isfinite(p).all(axis=1)]))
    floor = tuple(float(np.mean([f[i] for f in fits])) for i in range(3))
    stacked = np.stack(frames).astype(float)
    stacked[stacked == 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        median = N.toy_input(meta, np.nan_to_num(np.nanmedian(stacked, axis=0)).astype(np.uint16))[0]
    specs = known_specs()
    site = load_site_defaults(REPO / "config/site_load_defaults.yaml", specs)
    site = {specs[k].name: v for k, v in site.items()}
    setup = ForkSetup(FORK_SPACING_M)

    def load_for(spec):
        return None if spec is None else site.get(spec.name)

    out = {"capture": name, "kind": kind, "distance_m": distance, "width": int(frames[0].shape[1]),
           "floor": {"pitch_deg": float(np.degrees(np.arctan(floor[0]))),
                     "roll_deg": float(np.degrees(np.arctan(floor[1]))), "offset_mm": floor[2] * 1e3},
           "arms": {}}
    for proposer, mount in ARMS:
        if kind == "wall" and proposer != "geometric":
            continue
        ref, lanes = None, LANES_M.get(kind)
        m_scene = E.levelled(median, floor) if mount == "level" else median
        if kind != "wall":
            fx, fy = E.front_of(m_scene, roi, distance)
            ref = {"front_x": fx, "front_y": fy}
        rows = []
        for scene in scenes:
            s = E.levelled(scene, floor) if mount == "level" else scene
            roi_mask = roi_pixels(s, roi, ref["front_x"]) if ref else None
            t0 = time.perf_counter()
            if proposer.startswith("geometric"):
                # geometric_wide: the proposer's own face check (geometric_proposals.propose calls FitParams())
                # opened the same way, by swapping the name it looks up -- diagnostic only, restored at once.
                G.FitParams = _wide_params if proposer == "geometric_wide" else FitParams
                try:
                    got = G.propose(s)
                finally:
                    G.FitParams = FitParams
                proposals, failures, threshold = list(got.proposals), tuple(got.failures), None
            else:
                proposals, failures, threshold = [InstanceProposal("roi", roi_mask, 1.0, "oracle")], (), None
            prop_ms = 1e3 * (time.perf_counter() - t0)
            params = _wide_params() if proposer.endswith("_wide") else FitParams()
            rec = frame_record(s, proposals, failures, specs, setup, load_for, ref, roi_mask, lanes, kind, threshold,
                               params)
            rec["proposer_ms"] = prop_ms
            if ref is not None and proposer.startswith("geometric"):
                rec["toy_component"] = toy_component(s, roi_mask, params)
            elif ref is not None:
                pts, _ = F.proposal_points(s, F.pixel_rays(s), roi_mask, params.erode_px)
                rec["toy_face"] = face_diagnosis(pts, F.camera_xyz(s), params)
            rows.append(rec)
        out["arms"][f"{proposer}/{mount}"] = {"reference": ref, "frames": rows}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("data", type=Path)
    ap.add_argument("--json", type=Path, required=True)
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--only", nargs="*", help="capture names (default all)")
    a = ap.parse_args(argv)
    names = sorted(p.name for p in a.data.iterdir() if p.is_dir() and (p / "capture.json").exists())
    if a.only:
        names = [n for n in names if n in a.only]
    tasks = [(str(a.data), n) for n in names]
    t0 = time.time()
    if a.jobs > 1:
        with multiprocessing.Pool(a.jobs) as pool:
            captures = pool.map(run_capture, tasks)
    else:
        captures = [run_capture(t) for t in tasks]
    meta = {"data": str(a.data), "fork_spacing_m": FORK_SPACING_M, "cover": COVER, "zone_radius_m": ZONE_RADIUS_M,
            "geometric_params": dataclasses.asdict(G.GeometricParams()), "wall_s": time.time() - t0}
    a.json.write_text(json.dumps({"meta": meta, "captures": captures}, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
