#!/usr/bin/env python3
"""Why the team's D435i frames of the chassis's bundled pallet give A0 no valid observation (near-refit E4,
docs/validation/2026-11-06-near-capture-plane-refit.md): the frozen detector run on the saved Z16 frames under a
factorial of what could be blocking it, nothing in the detector changed. It reports, it does not tune:

  mount   nominal: E4's level camera at the noted 0.27 m. level: the same frames with the floor fitted (capture by
          capture, the pallet's ROI left out) moved onto z = 0.
  crop    full: every pixel. range: the workspace ends 0.8 m behind the measured front (range_max_m): the background
          taken out of the workspace alone, the rays unchanged. roi: depth outside the capture's hand-drawn ROI set
          unknown -- an oracle that also changes what the opening rays see. The runtime has neither.
  prior   provisional: E4's frozen values. heights: height, deck and opening as measured here from the 0.5 m captures,
          widths still the tape's. open_widths: heights, and the lateral gates opened far beyond the tape, to see
          whether they are what stops it -- not a proposal for values.
  inlier  derived: derived_for(prior). wide: plane_inlier_m 0.02 and max_plane_residual_m 0.025, the catalogue's
          inlier -- not a proposal either.

Frames: the 240 of E4 (toy 1.0, 1.5, 2.0 m and toy30 1.0 m at 640 and 848); the 0.5 m captures with range_min_m 0.4,
reported apart (E4 left them out of range); the wall captures, 10 frames each under both priors, as negatives; one
per-pixel median of each capture's 30 frames and six medians of 5 consecutive frames, reported apart (temporal noise
reduced, with the frames' own unknown pixels filled by any return among them).

Each frame is put on one funnel step:
  no_face_plane     not valid, and no plane candidate at the pallet front (RANSAC took the background, or the face is
                    not one plane). A valid without a plane at the front is valid_wrong; ``face_plane_frames`` counts
                    the frames with a candidate there whatever the outcome
  face_no_pattern   a plane at the front, no pattern on it (its first failed gate is recorded)
  other_plane       a pattern was selected on a plane elsewhere
  face_rejected     the selected pattern is on the front and a later check refused it (its reason is recorded)
  valid_right       valid, within 6 cm of the front along x and of the pallet's centre across, yaw within 15 deg, pocket
                    spacing within 3 cm of the tape's: consistent with the pallet, not a pose truth (there is none)
  valid_wrong       any other valid

  PYTHONPATH=<repo>/src python e4_diagnose.py DATA_DIR --json OUT.json [--jobs N]
"""

import argparse
import collections
import dataclasses
import itertools
import json
import multiprocessing
import sys
import warnings
from pathlib import Path

import numpy as np
from PIL import Image

REPO = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(REPO), str(REPO / "tools")]

import near_refit_compare as N  # noqa: E402
from forklift_core.geometry import RigidTransform  # noqa: E402
from forklift_core.perception import pocket_detector as det  # noqa: E402
from forklift_core.perception.pallet_prior import PalletPrior  # noqa: E402
from forklift_core.perception.pocket_detector import DetectorParams  # noqa: E402

FACE_DISTANCE_M = 0.06  # a plane this close to the capture's measured front is "at the front"
FACE_YAW_DEG = 15.0
SPACING_TOL_M = 0.03
LANES_M = {"toy": 0.19, "toy30": 0.12}  # the tape's leg spacing, so the pockets' spacing
WALL_FRAMES = 10


def measured_prior(kind: str, open_widths: bool | str) -> PalletPrior:
    """Heights measured in the levelled 0.5 m medians of both faces (height 0.093, deck underside 0.069); widths the
    tape's, or opened to see whether the lateral gates decide: True both, "opening" or "spacer" the one. Evaluation
    only."""
    base = N.TOY_PRIORS[kind]
    widths = {}
    if open_widths:
        widths = ({"opening_width_min_m": 0.09, "opening_width_max_m": 0.16, "centre_spacer_min_m": 0.03,
                   "centre_spacer_max_m": 0.10} if kind == "toy" else
                  {"opening_width_min_m": 0.05, "opening_width_max_m": 0.10, "centre_spacer_min_m": 0.03,
                   "centre_spacer_max_m": 0.08})
        if open_widths == "opening":
            widths = {k: v for k, v in widths.items() if k.startswith("opening")}
        elif open_widths == "spacer":
            widths = {k: v for k, v in widths.items() if k.startswith("centre")}
    return dataclasses.replace(base, height_m=0.093, deck_top_m=0.024, opening_height_m=0.069,
                               source_provenance="evaluation_only_measured_in_the_0p5m_medians", **widths)


PRIORS = {"provisional": lambda k: N.TOY_PRIORS[k], "heights": lambda k: measured_prior(k, False),
          "open_widths": lambda k: measured_prior(k, True)}
CELLS = list(itertools.product(("nominal", "level"), ("full", "range", "roi"), tuple(PRIORS), ("derived", "wide")))
STEP_CODE = {"no_face_plane": "n", "face_no_pattern": "p", "other_plane": "o", "face_rejected": "r", "valid_right": "V",
             "valid_wrong": "w", "not_valid": "x"}


def params_for(prior, inlier: str, range_min: float | None, range_max: float | None = None) -> DetectorParams:
    over = {} if range_min is None else {"range_min_m": range_min}
    if range_max is not None:
        over["range_max_m"] = range_max
    if inlier == "wide":
        over.update(plane_inlier_m=0.02, max_plane_residual_m=0.025)
    return DetectorParams.derived_for(prior, **over)


def floor_fit(points: np.ndarray) -> tuple:
    """Robust z = a x + b y + c over the floor band, in the nominal base frame."""
    q = points[(points[:, 2] > -0.08) & (points[:, 2] < 0.03) & (points[:, 0] > 0.45) & (points[:, 0] < 3.0)]
    coef, residual = None, None
    for _ in range(8):
        a = np.c_[q[:, 0], q[:, 1], np.ones(len(q))]
        coef = np.linalg.lstsq(a, q[:, 2], rcond=None)[0]
        r = q[:, 2] - a @ coef
        residual = float(np.std(r))
        q = q[np.abs(r) < max(0.004, 2.5 * residual)]
    return float(coef[0]), float(coef[1]), float(coef[2]), residual, len(q)


def levelled(scene, fit: tuple):
    """The scene with base_link moved so the fitted floor is z = 0: p' = R p - c n_z, n the floor's normal."""
    a, b, c = fit[:3]
    n = np.array((-a, -b, 1.0))
    n /= np.linalg.norm(n)
    v = np.cross(n, (0.0, 0.0, 1.0))
    s = float(np.linalg.norm(v))
    cross = np.array(((0.0, -v[2], v[1]), (v[2], 0.0, -v[0]), (-v[1], v[0], 0.0)))
    r = np.eye(3) + cross + cross @ cross * ((1 - n[2]) / (s * s))
    old = scene.base_from_optical
    mount = RigidTransform(old.source_frame, old.target_frame, r @ old.rotation,
                           r @ old.translation_m - np.array((0.0, 0.0, c * n[2])))
    return dataclasses.replace(scene, base_from_optical=mount)


def base_grid(scene) -> np.ndarray:
    points, _ = det._base_points(scene)
    return points.reshape(*scene.depth_m.shape, 3)


def cropped(scene, roi):
    x0, y0, x1, y1 = roi
    depth = np.full_like(scene.depth_m, np.nan)
    depth[y0:y1, x0:x1] = scene.depth_m[y0:y1, x0:x1]
    return dataclasses.replace(scene, depth_m=depth)


def front_of(scene, roi, distance: float) -> tuple[float, float]:
    """The pallet front in a levelled scene: its x, the near edge of the ROI's points above the floor, and its y, the
    middle of that front layer."""
    x0, y0, x1, y1 = roi
    p = base_grid(scene)[y0:y1, x0:x1].reshape(-1, 3)
    p = p[np.isfinite(p).all(axis=1)]
    p = p[(p[:, 0] < distance + 0.5) & (p[:, 2] > 0.006)]
    x = float(np.percentile(p[:, 0], 2))
    layer = p[p[:, 0] < x + 0.04, 1]
    return x, float((np.percentile(layer, 2) + np.percentile(layer, 98)) / 2)


def at_face(plane, camera, front: float) -> bool:
    """Facing the camera within the yaw, and the plane itself (not its inliers' middle) at the front."""
    distance = abs(float((camera - plane.point) @ plane.normal))
    yaw = abs(np.degrees(np.arctan2(plane.normal[1], -plane.normal[0])))
    return abs(distance - front) <= FACE_DISTANCE_M and min(yaw, 180 - yaw) <= FACE_YAW_DEG


def classify(scene, prior, params, front: tuple, spacing: float) -> dict:
    """One detection on the funnel. The planes are re-extracted as the detector does (same seed, same order), which
    the assert at the end of ``run_capture`` checks by count."""
    result = det.detect_pockets_expected(scene, prior, params, None, lower_evidence=False)[0]
    obs, diag = result.observation, result.diagnostics
    points, _ = det._base_points(scene)
    camera = scene.base_from_optical.translation_m
    workspace = det._filter_workspace(points, camera, prior, params)
    planes = det._vertical_plane_candidates(workspace, camera, params)
    face = {i for i, plane in enumerate(planes) if at_face(plane, camera, front[0])}
    rec = {"status": obs.status, "reason": obs.reason, "planes": len(planes), "face_planes": len(face),
           "workspace": int(len(workspace)), "gates": {},
           "widths_m": [float(w) for w in diag.opening_width_raw_m] if diag.opening_width_raw_m else None}
    assert len(planes) == diag.candidate_plane_count
    for rejected in diag.rejected_patterns:
        if rejected.plane_index in face:
            rec["gates"][rejected.stage] = rec["gates"].get(rejected.stage, 0) + 1
    if obs.status == "valid":
        left, right = np.array(obs.left.center_m), np.array(obs.right.center_m)
        gap = float(np.linalg.norm(left - right))
        x, y = float((left[0] + right[0]) / 2), float((left[1] + right[1]) / 2)
        ok = (abs(gap - spacing) <= SPACING_TOL_M and abs(x - front[0]) <= FACE_DISTANCE_M
              and abs(y - front[1]) <= FACE_DISTANCE_M and abs(np.degrees(obs.insertion_yaw_rad)) <= FACE_YAW_DEG)
        rec.update(step="valid_right" if ok else "valid_wrong", spacing_m=gap, centre_m=[x, y])
    elif not face:
        rec["step"] = "no_face_plane"
    elif diag.plane_inlier_count == 0:
        rec["step"] = "face_no_pattern"
    else:
        selected = _selected_at_face(diag, planes, face)
        rec["step"] = "face_rejected" if selected else "other_plane"
    return rec


def _selected_at_face(diag, planes, face) -> bool:
    """The selected plane, by what the diagnostics keep of it: its inlier count and residual."""
    return any(len(planes[i].points) == diag.plane_inlier_count
               and planes[i].residual_p95_m == diag.plane_residual_p95_m for i in face)


def sensor_metrics(meta: dict, frames: list, median_scene, front: float, scale_m: float) -> dict:
    """What the sensor gave in the front layer, in the levelled frame."""
    x0, y0, x1, y1 = meta["setup"]["roi"]
    stack = np.stack([f[y0:y1, x0:x1].astype(float) for f in frames])
    filled = stack > 0
    stack[~filled] = np.nan
    p = base_grid(median_scene)[y0:y1, x0:x1]
    face = np.isfinite(p).all(axis=2) & (p[..., 2] > 0.012) & (p[..., 0] < front + 0.04)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        std = np.nanstd(stack, axis=0)
    roi = p.reshape(-1, 3)[face.ravel()]
    return {"roi_fill": float(filled.mean()), "face_pixels": int(face.sum()),
            "face_fill": float(filled[:, face].mean()) if face.any() else None,
            "temporal_std_mm_median": float(np.nanmedian(std[face])) * scale_m * 1e3 if face.any() else None,
            "front_x_p10_p90_m": [float(np.percentile(roi[:, 0], q)) for q in (10, 90)] if len(roi) else None,
            "front_m": front}


def run_capture(task: tuple) -> dict:
    root, name = task
    capture = Path(root) / name
    meta = json.loads((capture / "capture.json").read_text())
    kind = name.split("_")[0]
    distance = float(meta["setup"]["distance_m"])
    roi = meta["setup"]["roi"]
    paths = sorted(capture.glob("z16_*.png"))
    frames = [np.asarray(Image.open(p)) for p in paths]
    scenes = [N.toy_input(meta, f)[0] for f in frames]
    # the floor of this capture, the pallet left out (a wall capture's ROI is the wall)
    fits = []
    for scene in scenes[:: max(1, len(scenes) // 6)]:
        grid = base_grid(scene).copy()
        x0, y0, x1, y1 = roi
        grid[y0:y1, x0:x1] = np.nan
        p = grid.reshape(-1, 3)
        fits.append(floor_fit(p[np.isfinite(p).all(axis=1)]))
    fit = tuple(float(np.mean([f[i] for f in fits])) for i in range(3))
    stacked = np.stack(frames).astype(float)
    stacked[stacked == 0] = np.nan
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)  # pixels with no return in any frame stay unknown
        median_z16 = np.nan_to_num(np.nanmedian(stacked, axis=0)).astype(np.uint16)
    median_scene = N.toy_input(meta, median_z16)[0]
    groups = []
    for start in range(0, len(frames) - 4, 5):
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            groups.append(N.toy_input(meta, np.nan_to_num(np.nanmedian(stacked[start:start + 5], axis=0)).astype(np.uint16))[0])
    front = None if kind == "wall" else front_of(levelled(median_scene, fit), roi, distance)
    out = {"capture": name, "kind": kind, "distance_m": distance, "width": int(frames[0].shape[1]),
           "floor": {"pitch_deg": float(np.degrees(np.arctan(fit[0]))), "roll_deg": float(np.degrees(np.arctan(fit[1]))),
                     "offset_mm": fit[2] * 1e3, "residual_mm": float(np.mean([f[3] for f in fits])) * 1e3},
           "rows": []}
    if kind != "wall":
        out["sensor"] = sensor_metrics(meta, frames, levelled(median_scene, fit), front[0],
                                       float(meta["device"]["depth_scale_m"]))
        out["pallet_centre_y_m"] = front[1]
    range_min = 0.4 if distance < DetectorParams().range_min_m else None
    judged = "reference_0p5m" if range_min else "e4" if kind != "wall" else "wall"
    sets = [(judged, scenes if kind != "wall" else scenes[:WALL_FRAMES])]
    if kind != "wall":
        sets += [("median", [median_scene]), ("median5", groups)]
    prior_kinds = N.TOY_PRIORS if kind == "wall" else {kind: None}
    for label, group in sets:
        for prior_kind in prior_kinds:
            spacing = LANES_M[prior_kind]
            for mount, crop, prior_name, inlier in CELLS:
                prior = PRIORS[prior_name](prior_kind)
                behind = distance if front is None else front[0]
                params = params_for(prior, inlier, range_min, behind + 0.8 if crop == "range" else None)
                steps, reasons, gates, spacings, face_planes, workspace, widths, codes = (
                    collections.Counter(), collections.Counter(), collections.Counter(), [], [], [], [], [])
                for scene in group:
                    s = levelled(scene, fit) if mount == "level" else scene
                    s = cropped(s, roi) if crop == "roi" else s
                    if kind == "wall":
                        # no front to be at: only whether anything is called valid
                        result = det.detect_pockets_expected(s, prior, params, None, lower_evidence=False)[0]
                        step = "valid_wrong" if result.observation.status == "valid" else "not_valid"
                        steps[step] += 1
                        codes.append(STEP_CODE[step])
                        reasons[str(result.observation.reason)] += 1
                        continue
                    rec = classify(s, prior, params, front, spacing)  # the mounts differ by < 1 cm at the front
                    steps[rec["step"]] += 1
                    codes.append(STEP_CODE[rec["step"]])
                    reasons[str(rec["reason"])] += 1
                    gates.update(rec["gates"])
                    face_planes.append(rec["face_planes"])
                    workspace.append(rec["workspace"])
                    if "spacing_m" in rec:
                        spacings.append(rec["spacing_m"])
                    if rec["step"] in ("face_rejected", "valid_right") and rec["widths_m"]:
                        widths.append(sorted(rec["widths_m"]))
                row = {"set": label, "prior_kind": prior_kind, "mount": mount, "crop": crop, "prior": prior_name,
                       "inlier": inlier, "frames": len(group), "steps": dict(steps), "reasons": dict(reasons),
                       "gates": dict(gates), "frame_steps": "".join(codes)}
                if workspace:
                    row["workspace_median"] = int(np.median(workspace))
                    row["face_plane_frames"] = int(sum(n > 0 for n in face_planes))
                if spacings:
                    row["spacing_median_m"] = float(np.median(spacings))
                if widths:
                    row["selected_widths_median_m"] = [float(v) for v in np.median(widths, axis=0)]
                out["rows"].append(row)
    return out


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("data", type=Path)
    p.add_argument("--json", type=Path, required=True)
    p.add_argument("--jobs", type=int, default=1)
    p.add_argument("--only", nargs="*", help="capture names (a quick look)")
    args = p.parse_args()
    names = args.only or sorted(q.name for q in args.data.iterdir() if q.is_dir())
    with multiprocessing.Pool(args.jobs) as pool:
        out = pool.map(run_capture, [(str(args.data), n) for n in names], chunksize=1)
    args.json.write_text(json.dumps({"captures": out}, indent=1) + "\n")
    print(f"{len(out)} captures -> {args.json}")


if __name__ == "__main__":
    main()
